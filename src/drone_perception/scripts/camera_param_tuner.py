#!/usr/bin/env python3
# ==============================================================================
# 工业相机参数调试界面（独立于 OpenCV 调试窗口）
#
# 用途：验证地面站 IndustrialCameraParams 链路——参数上下限对不对、调了有没有用。
#   - 浏览器打开 http://<板子IP>:<http_port>/ 即可动态调参；
#   - 启动时用 v4l2-ctl 查询驱动的真实 min/max/step/default 生成控件（不写死）；
#   - 任何改动都发布"完整"的 IndustrialCameraParams 到 /industrial_camera/params，
#     QoS 为 reliable + transient_local 深度 1，与视觉节点订阅端严格一致；
#   - 每秒回读驱动实际值：请求值≠实际值标红；自动模式接管（inactive）标灰；
#   - 订阅 /rosout，把视觉节点的 "Industrial camera control result" 警告贴到页面上，
#     地面站看不到的拒绝原因在这里全部可见。
#
# 与视觉节点共存：本节点只在发布/回读瞬间用独立 fd 访问 V4L2 控制接口，
# 不碰视频流，可在节点采集中安全运行。
#
# 启动（跟随视觉端 launch）：
#   ros2 launch drone_perception industrial_animal_vision.launch.py camera_tuner_enabled:=true
# 或单独运行：
#   ros2 run drone_perception camera_param_tuner.py
# ==============================================================================
import glob
import json
import re
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from drone_msgs.msg import IndustrialCameraParams
from rcl_interfaces.msg import Log

PARAMS_TOPIC = "/industrial_camera/params"

# msg 字段 -> (V4L2 控制名, 中文标签, 控件类型)
# bool_menu：msg 是 bool 但驱动是菜单（曝光 1=手动/3=自动）
FIELDS = (
    ("auto_exposure", "exposure_auto", "自动曝光", "bool_menu"),
    ("exposure_absolute", "exposure_absolute", "曝光值（单位 0.1ms）", "int"),
    ("auto_exposure_priority", "exposure_auto_priority", "允许降帧换亮度", "bool"),
    ("gain", "gain", "增益", "int"),
    ("brightness", "brightness", "亮度", "int"),
    ("contrast", "contrast", "对比度", "int"),
    ("saturation", "saturation", "饱和度", "int"),
    ("gamma", "gamma", "伽马", "int"),
    ("sharpness", "sharpness", "锐度", "int"),
    ("backlight_compensation", "backlight_compensation", "逆光补偿", "int"),
    ("auto_white_balance", "white_balance_temperature_auto", "自动白平衡", "bool"),
    ("white_balance_temperature", "white_balance_temperature", "色温（K）", "int"),
    ("power_line_frequency", "power_line_frequency", "防闪烁", "menu"),
    ("auto_focus", "focus_auto", "自动对焦", "bool"),
    ("focus_absolute", "focus_absolute", "焦点位置", "int"),
    ("zoom_absolute", "zoom_absolute", "数字变焦（100=1.0x）", "int"),
)

CTRL_LINE = re.compile(
    r"^\s*(\w+)\s+0x[0-9a-f]+\s+\((int|bool|menu)\)\s*:\s*(.*)$")
MENU_ITEM = re.compile(r"^\s*(\d+):\s*(.*)$")
KEY_VALUE = re.compile(r"(\w+)=(-?\w+)")


def find_camera_device(configured):
    """优先用配置的路径；为空则按 by-id 通配 -> USB VID:PID 扫描兜底。"""
    if configured:
        return configured
    by_id = sorted(glob.glob(
        "/dev/v4l/by-id/usb-12MP_U3_Camera*video-index0"))
    if by_id:
        return by_id[0]
    for video in sorted(glob.glob("/dev/video*")):
        try:
            info = subprocess.run(
                ["udevadm", "info", "-q", "property", "-n", video],
                capture_output=True, text=True, timeout=5).stdout
        except (subprocess.SubprocessError, OSError):
            continue
        if "ID_VENDOR_ID=32e4" not in info or "ID_MODEL_ID=6577" not in info:
            continue
        try:
            formats = subprocess.run(
                ["v4l2-ctl", "-d", video, "--list-formats"],
                capture_output=True, text=True, timeout=5).stdout
        except (subprocess.SubprocessError, OSError):
            continue
        if "MJPG" in formats:
            return video
    return ""


def query_controls(device):
    """解析 v4l2-ctl --list-ctrls-menus：{控制名: {type,min,max,step,default,value,inactive,menu}}"""
    try:
        output = subprocess.run(
            ["v4l2-ctl", "-d", device, "--list-ctrls-menus"],
            capture_output=True, text=True, timeout=5).stdout
    except (subprocess.SubprocessError, OSError):
        return {}
    controls = {}
    current = None
    for line in output.splitlines():
        matched = CTRL_LINE.match(line)
        if matched:
            name, ctrl_type, rest = matched.groups()
            entry = {"type": ctrl_type, "menu": {},
                     "inactive": "flags=inactive" in rest}
            for key, value in KEY_VALUE.findall(rest):
                if key in ("min", "max", "step", "default", "value"):
                    try:
                        entry[key] = int(value)
                    except ValueError:
                        pass
            controls[name] = entry
            current = entry
        elif current is not None:
            item = MENU_ITEM.match(line)
            if item:
                current["menu"][int(item.group(1))] = item.group(2).strip()
    return controls


class CameraParamTuner(Node):
    def __init__(self):
        super().__init__("camera_param_tuner")
        self.declare_parameter("camera_device", "")
        self.declare_parameter("http_port", 8899)
        self.device = find_camera_device(
            self.get_parameter("camera_device").value)
        self.http_port = int(self.get_parameter("http_port").value)
        if not self.device:
            raise RuntimeError("找不到相机设备：手动传 camera_device 参数")

        # QoS 必须与视觉节点订阅端一致（reliable + transient_local 深度 1），
        # 否则消息根本到不了——这也是地面站要核对的第一件事。
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(
            IndustrialCameraParams, PARAMS_TOPIC, qos)

        self.lock = threading.Lock()
        self.last_published = None      # dict：最近一次发布的完整字段集
        self.publish_count = 0
        self.warnings = []              # 视觉节点 rosout 里的相机相关日志
        self.controls_cache = {}
        self.controls_cache_at = 0.0

        self.create_subscription(Log, "/rosout", self.handle_rosout, 50)
        self.get_logger().info(
            f"camera_param_tuner 就绪：设备 {self.device}，"
            f"网页 http://0.0.0.0:{self.http_port}/")

    def handle_rosout(self, message):
        if "industrial_animal_vision" not in message.name:
            return
        text = message.msg
        if "camera" not in text.lower():
            return
        with self.lock:
            self.warnings.append(
                {"stamp": time.strftime("%H:%M:%S"), "text": text})
            del self.warnings[:-30]

    def controls(self, max_age_s=0.5):
        now = time.monotonic()
        with self.lock:
            if now - self.controls_cache_at > max_age_s:
                self.controls_cache = query_controls(self.device)
                self.controls_cache_at = now
            return self.controls_cache

    def publish_params(self, fields):
        message = IndustrialCameraParams()
        message.auto_exposure = bool(fields["auto_exposure"])
        message.exposure_absolute = int(fields["exposure_absolute"])
        message.auto_exposure_priority = bool(fields["auto_exposure_priority"])
        message.gain = int(fields["gain"])
        message.brightness = int(fields["brightness"])
        message.contrast = int(fields["contrast"])
        message.saturation = int(fields["saturation"])
        message.gamma = int(fields["gamma"])
        message.sharpness = int(fields["sharpness"])
        message.backlight_compensation = int(fields["backlight_compensation"])
        message.auto_white_balance = bool(fields["auto_white_balance"])
        message.white_balance_temperature = int(
            fields["white_balance_temperature"])
        message.power_line_frequency = int(fields["power_line_frequency"])
        message.auto_focus = bool(fields["auto_focus"])
        message.focus_absolute = int(fields["focus_absolute"])
        message.zoom_absolute = int(fields["zoom_absolute"])
        self.publisher.publish(message)
        with self.lock:
            self.last_published = dict(fields)
            self.publish_count += 1

    def state_snapshot(self):
        controls = self.controls()
        with self.lock:
            return {
                "device": self.device,
                "topic": PARAMS_TOPIC,
                "fields": [
                    {"field": field, "ctrl": ctrl, "label": label,
                     "widget": widget, "ctrl_info": controls.get(ctrl)}
                    for field, ctrl, label, widget in FIELDS
                ],
                "last_published": self.last_published,
                "publish_count": self.publish_count,
                "warnings": list(self.warnings),
            }


PAGE = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>相机参数调试台</title>
<style>
  body { font-family: system-ui, sans-serif; margin: 16px; background: #14171c; color: #dde3ea; }
  h1 { font-size: 18px; } h1 small { color: #7d8894; font-weight: normal; }
  table { border-collapse: collapse; width: 100%; max-width: 980px; }
  th, td { padding: 6px 10px; border-bottom: 1px solid #2a3038; text-align: left; font-size: 14px; }
  th { color: #8fa1b3; font-weight: normal; }
  tr.inactive td { color: #5c6672; }
  input[type=range] { width: 180px; vertical-align: middle; }
  input[type=number] { width: 70px; background: #1d232b; color: #dde3ea; border: 1px solid #3a424d; border-radius: 4px; padding: 2px 4px; }
  select { background: #1d232b; color: #dde3ea; border: 1px solid #3a424d; border-radius: 4px; }
  .name { white-space: nowrap; } .name code { color: #6e7a87; font-size: 11px; display: block; }
  .range { color: #8fa1b3; white-space: nowrap; font-size: 12px; }
  .actual { font-variant-numeric: tabular-nums; }
  .bad { color: #ff6b6b; font-weight: bold; }
  .auto { color: #e8b339; }
  .ok { color: #58c26e; }
  #meta { color: #7d8894; font-size: 13px; margin-bottom: 10px; }
  #warnings { max-width: 980px; background: #1a1f26; border: 1px solid #2a3038; border-radius: 6px;
              padding: 8px 12px; margin-top: 14px; font-size: 13px; }
  #warnings h2 { font-size: 14px; margin: 2px 0 6px; color: #8fa1b3; }
  #warnings div { color: #e8b339; font-family: monospace; }
  #warnings .empty { color: #5c6672; }
</style></head><body>
<h1>相机参数调试台 <small id="meta">加载中……</small></h1>
<table id="grid"><thead><tr>
  <th>参数</th><th>驱动范围</th><th>设定</th><th>驱动实际值</th><th>状态</th>
</tr></thead><tbody></tbody></table>
<div id="warnings"><h2>视觉节点应用日志（/rosout）</h2><div class="empty">暂无</div></div>
<script>
let FIELDS = [], VALUES = {}, timer = null;

function ctrlToField(f, info) {           // 驱动当前值 -> msg 字段值
  if (!info || info.value === undefined) return f.widget.startsWith('bool') ? false : 0;
  if (f.widget === 'bool_menu') return info.value === 3;
  if (f.widget === 'bool') return info.value === 1;
  return info.value;
}
function fieldToCtrl(f, v) {              // msg 字段值 -> 驱动期望值（用于对账）
  if (f.widget === 'bool_menu') return v ? 3 : 1;
  if (f.widget === 'bool') return v ? 1 : 0;
  return Number(v);
}
function schedulePublish() {
  clearTimeout(timer);
  timer = setTimeout(() => fetch('/api/publish', {
    method: 'POST', headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(VALUES)}), 200);
}
function makeWidget(f, info) {
  const td = document.createElement('td');
  if (f.widget === 'bool' || f.widget === 'bool_menu') {
    const box = document.createElement('input');
    box.type = 'checkbox'; box.checked = VALUES[f.field];
    box.onchange = () => { VALUES[f.field] = box.checked; schedulePublish(); };
    td.appendChild(box);
  } else if (f.widget === 'menu' && info) {
    const sel = document.createElement('select');
    for (const [k, name] of Object.entries(info.menu)) {
      const opt = document.createElement('option');
      opt.value = k; opt.textContent = k + ' - ' + name;
      sel.appendChild(opt);
    }
    sel.value = VALUES[f.field];
    sel.onchange = () => { VALUES[f.field] = Number(sel.value); schedulePublish(); };
    td.appendChild(sel);
  } else if (info) {
    const bar = document.createElement('input');
    bar.type = 'range'; bar.min = info.min; bar.max = info.max;
    bar.step = info.step || 1; bar.value = VALUES[f.field];
    const num = document.createElement('input');
    num.type = 'number'; num.min = info.min; num.max = info.max;
    num.step = info.step || 1; num.value = VALUES[f.field];
    bar.oninput = () => { num.value = bar.value; VALUES[f.field] = Number(bar.value); schedulePublish(); };
    num.onchange = () => { bar.value = num.value; VALUES[f.field] = Number(num.value); schedulePublish(); };
    td.appendChild(bar); td.appendChild(num);
  } else {
    td.textContent = '驱动不支持';
  }
  return td;
}
function build(state) {
  FIELDS = state.fields;
  document.getElementById('meta').textContent =
    state.device + '  ->  ' + state.topic + '（已发布 ' + state.publish_count + ' 次）';
  const body = document.querySelector('#grid tbody');
  body.innerHTML = '';
  for (const f of FIELDS) {
    const info = f.ctrl_info;
    if (!(f.field in VALUES)) VALUES[f.field] = ctrlToField(f, info);
    const tr = document.createElement('tr');
    tr.id = 'row_' + f.field;
    const name = document.createElement('td');
    name.className = 'name';
    name.innerHTML = f.label + '<code>' + f.field + ' -> ' + f.ctrl + '</code>';
    tr.appendChild(name);
    const range = document.createElement('td');
    range.className = 'range';
    range.textContent = !info ? '-' : (info.type === 'int'
      ? info.min + ' ~ ' + info.max + ' 步长' + (info.step || 1) + ' 默认' + info.default
      : (info.type === 'menu' ? Object.entries(info.menu).map(e => e[0] + '=' + e[1]).join(' / ')
                              : '0 / 1 默认' + info.default));
    tr.appendChild(range);
    tr.appendChild(makeWidget(f, info));
    const actual = document.createElement('td');
    actual.className = 'actual'; actual.textContent = '-';
    tr.appendChild(actual);
    const status = document.createElement('td');
    tr.appendChild(status);
    body.appendChild(tr);
  }
}
function refresh(state) {
  document.getElementById('meta').textContent =
    state.device + '  ->  ' + state.topic + '（已发布 ' + state.publish_count + ' 次）';
  for (const f of state.fields) {
    const tr = document.getElementById('row_' + f.field);
    if (!tr) continue;
    const info = f.ctrl_info;
    const actual = tr.children[3], status = tr.children[4];
    tr.classList.toggle('inactive', !!(info && info.inactive));
    actual.textContent = info && info.value !== undefined ? info.value : '-';
    if (!info) { status.textContent = ''; continue; }
    if (info.inactive) {
      status.innerHTML = '<span class="auto">自动接管</span>'; continue;
    }
    const pub = state.last_published;
    if (!pub) { status.textContent = ''; continue; }
    status.innerHTML = fieldToCtrl(f, pub[f.field]) === info.value
      ? '<span class="ok">已生效</span>'
      : '<span class="bad">未生效（发 ' + fieldToCtrl(f, pub[f.field]) + ' 实 ' + info.value + '）</span>';
  }
  const box = document.querySelector('#warnings div');
  if (state.warnings.length) {
    box.className = '';
    box.innerHTML = state.warnings.slice(-8).map(
      w => '[' + w.stamp + '] ' + w.text).join('<br>');
  }
}
fetch('/api/state').then(r => r.json()).then(s => { build(s); refresh(s);
  setInterval(() => fetch('/api/state').then(r => r.json()).then(refresh), 1000); });
</script></body></html>
"""


class TunerHandler(BaseHTTPRequestHandler):
    node = None  # 由 main() 注入

    def log_message(self, *_):  # 静默 http 访问日志，别刷节点输出
        pass

    def _send(self, code, content_type, payload):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self._send(200, "text/html; charset=utf-8", PAGE.encode())
        elif self.path.startswith("/api/state"):
            self._send(200, "application/json",
                       json.dumps(self.node.state_snapshot()).encode())
        else:
            self._send(404, "text/plain", b"not found")

    def do_POST(self):
        if not self.path.startswith("/api/publish"):
            self._send(404, "text/plain", b"not found")
            return
        length = int(self.headers.get("Content-Length", "0"))
        try:
            fields = json.loads(self.rfile.read(length))
            self.node.publish_params(fields)
            self._send(200, "application/json", b'{"ok": true}')
        except (ValueError, KeyError) as error:
            self._send(400, "application/json",
                       json.dumps({"ok": False, "error": str(error)}).encode())


def main():
    rclpy.init()
    node = CameraParamTuner()
    TunerHandler.node = node
    server = ThreadingHTTPServer(("0.0.0.0", node.http_port), TunerHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

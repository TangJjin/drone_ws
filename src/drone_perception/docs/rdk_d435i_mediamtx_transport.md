# RDK X5 D435i MediaMTX transport

This path transports only the D435i RGB stream:

```text
D435i YUYV -> NV12 -> RDK hardware H.264 -> FFmpeg stream copy
-> MediaMTX -> RTSP client
```

The sender and receiver addresses are runtime settings. MediaMTX always runs on
the board connected to the D435i. The initial test uses `192.168.46.114` as the
sender and `192.168.46.253` as the receiver. The final deployment can reverse
those roles without rebuilding.

## Build on an RDK X5

The executable is built only when `hb_media_codec.h`, `libmultimedia`,
`libswscale`, and `libavutil` are available.

```bash
cd /path/to/drone_ws
colcon build --packages-select drone_perception --symlink-install
source install/setup.bash
```

## Start a sender

Install a verified ARM64 MediaMTX binary at
`/home/sunrise/mediamtx/mediamtx`, then run:

```bash
STREAM_CONFIG=/path/to/install/drone_perception/share/drone_perception/config/rdk_d435i_stream.yaml \
MEDIAMTX_BIN=/home/sunrise/mediamtx/mediamtx \
ros2 run drone_perception rdk_d435i_mediamtx_sender.sh
```

The sender config controls camera resolution, frame rate, encoder bitrate and
GOP, and the RTSP port, path, and transports. Changes take effect after the
sender is restarted and do not require rebuilding. The default config captures
and encodes D435i RGB directly at 320x240@30 without an intermediate resize.

For the standard RDK deployment at `/home/sunrise/drone_ws`, the equivalent
one-command launcher is:

```bash
/home/sunrise/drone_ws/install/drone_perception/lib/drone_perception/start_rdk_d435i_sender.sh
```

The stream URL is:

```text
rtsp://<sender-ip>:8554/d435i
```

The script starts MediaMTX for the foreground test and stops it when the script
exits. It does not install or enable a system service.

## Receive during the current test

On `192.168.46.253`, use the package receiver:

```bash
sudo systemctl stop lightdm
ros2 run drone_perception rdk_rtsp_display_receiver \
  --input rtsp://192.168.46.114:8554/d435i \
  --transport udp
```

Use `--transport tcp` only as a connectivity comparison.

## Reverse the roles later

When the D435i is moved to the other board, start the sender there. With
`192.168.46.114` as the final receiver, its URL becomes:

```text
rtsp://<future-sender-ip>:8554/d435i
```

No source-code address changes are required.

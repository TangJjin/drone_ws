#pragma once

#include <cstdint>

namespace drone_msgs::link_protocol
{
    constexpr uint8_t kSof1 = 0xAA;//帧头1，固定值0xAA，表示一帧数据的开始
    constexpr uint8_t kSof2 = 0x55;//帧头2，固定值0x55，与kSof1一起用于帧同步，确保接收端能够正确识别帧的起始位置
    constexpr uint8_t kVersion = 0x02;//V2显式规定float32/float64精度及长度前缀字符串
    constexpr uint8_t kFlagNeedAck = 0x01;//标志位，表示发送的请求需要对方回复确认帧
    constexpr uint8_t kFlagAck = 0x02;//标志位，表示为一个确认帧

    constexpr uint8_t kTypeUploadMissionSummaryReq = 0x01;//消息类型：上传 mission summary 请求
    constexpr uint8_t kTypeStartOffboardReq = 0x02;//消息类型：start_offboard 请求
    constexpr uint8_t kTypeStartTaskReq = 0x03;//消息类型：start_task 请求
    constexpr uint8_t kTypeStopPushReq = 0x04;//消息类型：stop_push 请求
    constexpr uint8_t kTypeAck = 0x05;//消息类型：确认帧

    constexpr uint8_t kTypeUploadMissionSummaryResp = 0x81;//消息类型：上传 mission summary 响应
    constexpr uint8_t kTypeStartOffboardResp = 0x82;//消息类型：start_offboard 响应
    constexpr uint8_t kTypeStartTaskResp = 0x83;//消息类型：start_task 响应
    constexpr uint8_t kTypeStopPushResp = 0x84;//消息类型：stop_push 响应
    
    constexpr uint8_t kTypeDroneStatus = 0x90;//消息类型：无人机状态
    constexpr uint8_t kTypePathReady = 0x91;//消息类型：路径准备就绪
    constexpr uint8_t kTypeTaskStatus = 0x92;//消息类型：任务状态
    constexpr uint8_t kTypeReturnWorldGroup = 0x93;//消息类型：返回世界坐标系下的无人机位姿和状态信息
    constexpr uint8_t kTypeVisionBarcode = 0x94;//消息类型：视觉二维码信息
    constexpr uint8_t kTypeDelta = 0x95;//消息类型：delta信息
    constexpr uint8_t kTypeLocalPosition = 0x96;//消息类型：本地位置信息
}


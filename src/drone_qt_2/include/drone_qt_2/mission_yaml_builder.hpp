#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <QString>
#include "drone_msgs/msg/mission_summary.hpp"

struct AirborneWorldCoord
{
    double x;
    double y;
    double z;
    double yaw;
};

class AirborneMissionYamlBuilder
{
public:
    struct Options
    {
        double takeoff_altitude{1.2};
        double move_altitude{1.2};
        double start_altitude{0.0};
        double yaw{0.0};
        double tolerance{0.10};
        double yaw_tolerance_deg{4.0};
        double max_xy_speed_mps{0.50};
        double max_z_speed_mps{0.30};
        double max_yaw_rate_deg_s{40.0};
        double takeoff_hover_duration{5.0};
        double landing_hover_duration{5.0};
        double move_hover_duration{5.0};
        bool add_hover_between_takeoff{true};
        bool add_hover_between_landing{false};
        bool add_hover_between_moves{true};
        bool auto_start_mission{false};
        bool compress_straight_segments{false};
        std::string frame{"world_body"};

        // Defaults are transported as one ROS message and emitted as flat system keys.
        drone_msgs::msg::VisualServoConfig visual_servo;
    };

    //压缩路径中的直线段，返回一个新的路径点列表，其中连续共线的点被压缩为起点和终点两个点
    static std::vector<AirborneWorldCoord> compressStraightSegments(const std::vector<AirborneWorldCoord> &points);
    //从MissionSummary对象中提取参数，构建一个Options对象
    static Options fromMissionSummary(const drone_msgs::msg::MissionSummary &summary);
    //根据路径点列表和选项参数构建一个符合机载端要求的任务yaml字符串
    static QString buildMissionYaml(const std::vector<AirborneWorldCoord> &points, const Options &options);
    //计算任务yaml中包含的动作数量，参数为路径点列表和选项参数
    static uint32_t countMissionActions(const std::vector<AirborneWorldCoord> &points, const Options &options);
};
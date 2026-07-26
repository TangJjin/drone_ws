#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "drone_msgs/msg/ready_status.hpp"
#include "drone_msgs/msg/task_status.hpp"
#include "drone_msgs/msg/world_group.hpp"
#include "drone_msgs/msg/world_point.hpp"

namespace
{

  class RouteCommNode : public rclcpp::Node
  {
  public:
    RouteCommNode() : rclcpp::Node("route_comm_node")
    {
      mission_config_path_ = declare_parameter<std::string>("mission_config_path", "");
      enable_offboard_control_ =
          declare_parameter<bool>("enable_offboard_control", false);

      const auto qos =
          rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();

      ready_status_pub_ =
          create_publisher<drone_msgs::msg::ReadyStatus>("/control/path_ready", qos);

      return_route_pub_ =
          create_publisher<drone_msgs::msg::WorldGroup>(
              "/return/drone/world_group", qos);

      // 创建订阅器：订阅任务状态消息
      task_status_sub_ = create_subscription<drone_msgs::msg::TaskStatus>(
          "/task/status", qos,
          std::bind(&RouteCommNode::taskStatusCb, this, std::placeholders::_1));

      // 加载任务路线配置
      loadMissionRoute();
      // 初始发布 false 状态，表示航线未就绪
      publishReadyStatus(false);
      // 创建定时器，每 500ms 评估一次就绪状态
      using namespace std::chrono_literals;
      ready_timer_ = create_wall_timer(
          500ms, std::bind(&RouteCommNode::evaluateReadyState, this));
    }

  private:
    void taskStatusCb(const drone_msgs::msg::TaskStatus::SharedPtr msg)
    {
      latest_task_status_ = *msg;
      task_status_received_ = true;
    }

    void loadMissionRoute()
    {
      cached_route_.points.clear();
      route_loaded_ = false;
      mission_valid_action_count_ = 0;

      if (mission_config_path_.empty())
      {
        ready_block_reason_ = "mission_config_path 为空。";
        RCLCPP_ERROR(get_logger(), "%s", ready_block_reason_.c_str());
        return;
      }

      std::ifstream file(mission_config_path_);
      if (!file.good())
      {
        ready_block_reason_ =
            "任务 YAML 文件不可读：" + mission_config_path_;
        RCLCPP_ERROR(get_logger(), "%s", ready_block_reason_.c_str());
        return;
      }

      YAML::Node config;
      try
      {
        config = YAML::LoadFile(mission_config_path_);
      }
      catch (const YAML::Exception &e)
      {
        ready_block_reason_ = std::string("任务 YAML 加载失败：") + e.what();
        RCLCPP_ERROR(get_logger(), "%s", ready_block_reason_.c_str());
        return;
      }


      if (!config["mission"] || !config["mission"]["actions"])
      {
        ready_block_reason_ = "任务 YAML 缺少 mission/actions 配置。";
        RCLCPP_ERROR(get_logger(), "%s", ready_block_reason_.c_str());
        return;
      }

      const YAML::Node actions = config["mission"]["actions"];
      for (std::size_t i = 0; i < actions.size(); ++i)
      {
        const YAML::Node item = actions[i];
        if (!item["type"])
        {
          RCLCPP_WARN(get_logger(), "第 %zu 个动作缺少 type 字段，已跳过。", i);
          continue;
        }

        const std::string type = item["type"].as<std::string>();

        if (type == "takeoff" || type == "land")
        {
          ++mission_valid_action_count_;
          continue;
        }

        if (type == "hover")
        {
          ++mission_valid_action_count_;
          continue;
        }

        if (type == "move") {
          if (!appendMovePoint(item, i)) {
            continue;
          }
          ++mission_valid_action_count_;
          continue;
        }

        if (type == "visual_servo")
        {
          ++mission_valid_action_count_;
          continue;
        }
        RCLCPP_WARN(get_logger(), "未知动作类型 '%s'，已跳过。", type.c_str());
      }

      if (cached_route_.points.empty()) {
        ready_block_reason_ = "任务 YAML 中没有可用 move 路线点，保持 air_route_ready=false。";
        RCLCPP_WARN(get_logger(), "%s", ready_block_reason_.c_str());
        return;
      }

      if (mission_valid_action_count_ == 0) {
        ready_block_reason_ = "任务 YAML 没有有效动作。";
        RCLCPP_WARN(get_logger(), "%s", ready_block_reason_.c_str());
        return;
      }

      route_loaded_ = true;
      ready_block_reason_.clear();
      RCLCPP_INFO(get_logger(),
              "路线解析完成：move 点数=%zu，有效动作数=%u，等待任务控制器状态确认。",
              cached_route_.points.size(), mission_valid_action_count_);
    }

    bool appendMovePoint(const YAML::Node &item, std::size_t index)
    {
      if (!hasValidPosition(item, index, "移动"))
      {
        return false;
      }

      const YAML::Node pos = item["position"];
      drone_msgs::msg::WorldPoint point;
      point.x = pos[0].as<double>();
      point.y = pos[1].as<double>();
      cached_route_.points.push_back(point);
      return true;
    }

    bool hasValidPosition(const YAML::Node &item, std::size_t index,
                          const char *action_name)
    {
      const YAML::Node pos = item["position"];
      if (!pos || pos.size() != 3)
      {
        RCLCPP_WARN(get_logger(), "第 %zu 个%s动作缺少 position: [x, y, z].", index, action_name);
        return false;
      }

      const double x = pos[0].as<double>();
      const double y = pos[1].as<double>();
      const double z = pos[2].as<double>();
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      {
        RCLCPP_WARN(get_logger(), "第 %zu 个%s动作的 position 存在非法数值。", index,
                    action_name);
        return false;
      }
      return true;
    }

    bool missionControllerReady() const
    {
      if (!task_status_received_)
      {
        return false;
      }

      if (latest_task_status_.action_num != mission_valid_action_count_)
      {
        return false;
      }

      return latest_task_status_.action_name == "idle" ||
             latest_task_status_.action_name == "waiting_start" ||
             latest_task_status_.action_name == "queued" ||
             latest_task_status_.task_running;
    }

    void evaluateReadyState()
    {
      if (!enable_offboard_control_)
      {
        setBlockedReason("enable_offboard_control=false，未启动 OFFBOARD 控制，不发布 ready。");
        publishReadyStatus(false);
        return;
      }

      if (!route_loaded_)
      {
        publishReadyStatus(false);
        return;
      }

      if (!task_status_received_)
      {
        setBlockedReason("尚未收到 /task/status，等待 mission_controller_node 完成初始化。");
        publishReadyStatus(false);
        return;
      }

      if (!missionControllerReady())
      {
        setBlockedReason("已收到 /task/status，但动作总数与任务 YAML 不一致，或任务控制器尚未就绪。");
        publishReadyStatus(false);
        return;
      }

      if (!route_echo_published_)
      {
        return_route_pub_->publish(cached_route_);
        route_echo_published_ = true;
        RCLCPP_INFO(get_logger(), "已回传处理后的路线到 /return/drone/world_group。");
      }

      setBlockedReason("");
      publishReadyStatus(true);      
    }

    void setBlockedReason(const std::string &reason)
    {
      if (ready_block_reason_ == reason)
      {
        return;
      }
      ready_block_reason_ = reason;
      if (!ready_block_reason_.empty())
      {
        RCLCPP_INFO(get_logger(), "%s", ready_block_reason_.c_str());
      }
    }

    void publishReadyStatus(bool ready)
    {
      if (last_ready_status_valid_ && last_ready_status_ == ready)
      {
        return;
      }

      drone_msgs::msg::ReadyStatus msg;
      msg.air_route_ready = ready;
      ready_status_pub_->publish(msg);

      last_ready_status_ = ready;
      last_ready_status_valid_ = true;

      if (ready)
      {
        RCLCPP_INFO(get_logger(),
                    "路线与任务控制器状态校验通过，发布 /control/path_ready=true。");
      }
      else
      {
        RCLCPP_INFO(get_logger(),
                    "路线尚未就绪，发布 /control/path_ready=false。");
      }
    }

    std::string mission_config_path_;
    bool enable_offboard_control_ = false;
    bool route_loaded_ = false;
    bool task_status_received_ = false;
    bool route_echo_published_ = false;
    bool last_ready_status_ = false;
    bool last_ready_status_valid_ = false;
    std::string ready_block_reason_;
    uint8_t mission_valid_action_count_ = 0;

    drone_msgs::msg::WorldGroup cached_route_;
    drone_msgs::msg::TaskStatus latest_task_status_;

    rclcpp::Publisher<drone_msgs::msg::ReadyStatus>::SharedPtr ready_status_pub_;
    rclcpp::Publisher<drone_msgs::msg::WorldGroup>::SharedPtr return_route_pub_;
    rclcpp::Subscription<drone_msgs::msg::TaskStatus>::SharedPtr task_status_sub_;
    rclcpp::TimerBase::SharedPtr ready_timer_;
  };

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);                      // 初始化 ROS 2
  auto node = std::make_shared<RouteCommNode>(); // 创建节点实例
  rclcpp::spin(node);                            // 进入事件循环
  rclcpp::shutdown();                            // 关闭 ROS 2
  return 0;
}

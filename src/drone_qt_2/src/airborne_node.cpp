#include "drone_qt_2/airborne_node.hpp"

#include <chrono>
#include <functional>
#include <sstream>
#include <fstream>
#include <filesystem>

//使用std::chrono_literals来方便地使用时间单位，如1s表示1秒
using namespace std::chrono_literals;

AirborneNode::AirborneNode()
    : rclcpp::Node("airborne_node")
{
    //设置ROS接口，包括发布者、服务和定时器
    setupInterfaces();
    RCLCPP_INFO(this->get_logger(), "airborne node started");
}

AirborneNode::~AirborneNode()
{
    std::string error_message;
    if (!stopOffboardProcess(error_message) && !error_message.empty()) {
        RCLCPP_WARN(this->get_logger(), "cleanup offboard process on shutdown: %s", error_message.c_str());
    }
}

void AirborneNode::setupInterfaces()
{
    auto status_pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个发布者，发布无人机状态话题，消息类型为自定义消息
    status_pub_ = this->create_publisher<drone_msgs::msg::DroneStatus>(
        "/drone/status", status_pub_qos);

    auto barcode_pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个发布者，发布条形码捕获话题，消息类型为自定义消息
    barcode_pub_ = this->create_publisher<drone_msgs::msg::BarcodeCapture>(
        "/drone/barcode_capture", barcode_pub_qos);

    auto position_pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个发布者，发布无人机本地位置话题，消息类型为geometry_msgs::msg::PoseStamped
    local_position_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/drone/local_position", position_pub_qos);

    auto drone_control_status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    //创建一个发布者，发布无人机控制状态话题，消息类型为自定义消息
    return_status_pub_ = this->create_publisher<drone_msgs::msg::TaskStatus>(
        "/drone/task/status", drone_control_status_qos);

    auto drone_control_path_ready_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    //创建一个发布者，发布无人机控制路径就绪话题，消息类型为自定义消息
    return_path_ready_pub_ = this->create_publisher<drone_msgs::msg::ReadyStatus>(
        "/drone/control/path_ready", drone_control_path_ready_qos);

    auto drone_return_world_group_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    //创建一个发布者，发布无人机回传路线话题，消息类型为自定义消息
    return_world_group_pub_ = this->create_publisher<drone_msgs::msg::WorldGroup>(
        "/drone/return/world_group", drone_return_world_group_qos);

    auto drone_return_delta_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个发布者，发布无人机dxdyaw比较结果，消息类型为geometry_msgs::msg::Vector3
    return_delta_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "/drone/pose_yaw_compare/delta", drone_return_delta_qos);

    auto vision_barcode_pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    //创建一个发布者，发布视觉系统捕获的条形码消息，消息类型为自定义消息
    vision_barcode_pub_ = this->create_publisher<drone_msgs::msg::BarcodeCapture>(
        "/drone/vision/barcode", vision_barcode_pub_qos);

    //创建一个订阅者，订阅视觉系统发布的条形码捕获话题，消息类型为自定义消息
    auto barcode_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    vision_barcode_sub_ = this->create_subscription<drone_msgs::msg::BarcodeCapture>(
        "/drone/image", barcode_qos,
        //使用std::bind将成员函数绑定为订阅回调函数
        std::bind(&AirborneNode::handleBarcodeCapture, this, std::placeholders::_1));

    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个订阅者，订阅无人机状态话题，消息类型为mavros_msgs::msg::State
    status_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", status_qos,
        [this](const mavros_msgs::msg::State::SharedPtr msg) {
            if (msg->mode == "MANUAL") {
                flight_mode = drone_msgs::msg::DroneStatus::MODE_MANUAL;
            } else if (msg->mode == "OFFBOARD") {
                flight_mode = drone_msgs::msg::DroneStatus::MODE_OFFBOARD;
            } else if (msg->mode == "STABILIZE") {
                flight_mode = drone_msgs::msg::DroneStatus::MODE_STABILIZE;
            } else if (msg->mode == "AUTO") {
                flight_mode = drone_msgs::msg::DroneStatus::MODE_AUTO;
            } else if (msg->mode == "LOITER") {
                flight_mode = drone_msgs::msg::DroneStatus::MODE_LOITER;
            } else if (msg->mode == "RTL") {
                flight_mode = drone_msgs::msg::DroneStatus::MODE_RTL;
            } else {
                flight_mode = drone_msgs::msg::DroneStatus::MODE_UNKNOWN;
            }
            connected = msg->connected ? true : false;
            armed = msg->armed;
        });

    auto capture_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    //创建一个订阅者，订阅无人机电量，消息类型为sensor_msgs::BatteryState
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/mavros/battery", capture_qos,
        [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
             if (msg->voltage <= 0.0f) {
                return;
            }

            if (msg->percentage < 0.0f) {
                return;
            }
            battery_voltage = msg->voltage;
            battery_percent = msg->percentage;
        });

    auto position_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    //创建一个订阅者，订阅无人机本地位置，消息类型为geometry_msgs::PoseStamped
    local_position_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose", position_qos,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            position_x = msg->pose.position.x;
            position_y = msg->pose.position.y;
            position_z = msg->pose.position.z;

            position_qx = msg->pose.orientation.x;
            position_qy = msg->pose.orientation.y;
            position_qz = msg->pose.orientation.z;
            position_qw = msg->pose.orientation.w;
        });

    auto control_status_qos =
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    //创建一个订阅者，订阅控制程序状态话题，消息类型为自定义消息
    task_status_sub_ = this->create_subscription<drone_msgs::msg::TaskStatus>(
        "/task/status", 
        control_status_qos, 
        [this](const drone_msgs::msg::TaskStatus::SharedPtr msg) {
            return_status_pub_->publish(*msg);
        });

    auto control_path_ready_qos =
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    //创建一个订阅者，订阅控制程序的确认消息，消息类型为自定义消息
    ready_status_sub_ = this->create_subscription<drone_msgs::msg::ReadyStatus>(
        "/control/path_ready", 
        control_path_ready_qos, 
        [this](const drone_msgs::msg::ReadyStatus::SharedPtr msg) {
            const bool ready = msg->air_route_ready;

            return_path_ready_pub_->publish(*msg);

            if (ready) {
                return;
            } else {
                //如果收到ready信号为false，说明控制程序确认失败，可能是因为无人机未进入offboard模式或者其他原因导致无法执行任务，此时需要重置状态以允许重新上传路线和启动任务
                offboard_started_ = false;
                task_started_ = false;
            }
        });

    auto return_world_group_qos =
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    //创建一个订阅者，订阅控制程序的路线消息，消息类型为自定义消息
    return_world_group_sub_ = this->create_subscription<drone_msgs::msg::WorldGroup>(
        "/return/drone/world_group", 
        return_world_group_qos, 
        [this](const drone_msgs::msg::WorldGroup::SharedPtr msg) {
            return_world_group_pub_->publish(*msg);
        });

    auto delta_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个订阅者，订阅无人机dxdyaw比较结果，消息类型为geometry_msgs::msg::Vector3
    return_delta_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
        "/pose_yaw_compare/delta",
        delta_qos,
        [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
            return_delta_pub_->publish(*msg);
        });


    //创建一个服务，提供任务启动接口，服务类型为自定义服务
    start_task_srv_ = this->create_service<drone_msgs::srv::StartTask>(
        "/drone/start_task",
        //使用std::bind将成员函数绑定为服务回调函数
        std::bind(
            &AirborneNode::handleStartTask,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    //创建一个服务，提供路线重传接口，服务类型为自定义服务
    stop_push_srv_ = this->create_service<drone_msgs::srv::StartTask>(
        "/drone/stop_push",
        //使用std::bind将成员函数绑定为服务回调函数
        std::bind(
            &AirborneNode::handleStopPush,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    start_offboard_srv_ = this->create_service<drone_msgs::srv::StartOffboard>(
        "/drone/start_offboard",
        std::bind(
            &AirborneNode::handleStartOffboard,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    upload_mission_summary_srv_ = this->create_service<drone_msgs::srv::UploadMissionSummary>(
        "/drone/upload_mission_summary",
        std::bind(
            &AirborneNode::handleUploadMissionSummary,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    timer_ = this->create_wall_timer(
        0.2s,
        std::bind(&AirborneNode::onTimer, this));
}

void AirborneNode::onTimer()
{
    publishStatus();
}

void AirborneNode::publishStatus()
{
    //构建一个状态消息，并发布到话题上
    drone_msgs::msg::DroneStatus status_msg;
    status_msg.connected = connected;
    status_msg.armed = armed;
    status_msg.flight_mode = flight_mode;
    status_msg.battery_voltage = battery_voltage;
    status_msg.battery_percent = battery_percent;
    status_pub_->publish(status_msg);

    geometry_msgs::msg::PoseStamped position_msg;
    position_msg.pose.position.x = position_x;
    position_msg.pose.position.y = position_y;
    position_msg.pose.position.z = position_z;
    position_msg.pose.orientation.x = position_qx;
    position_msg.pose.orientation.y = position_qy;
    position_msg.pose.orientation.z = position_qz;
    position_msg.pose.orientation.w = position_qw;
    local_position_pub_->publish(position_msg);

    if (armed == true) {
        if (unlock_flag_ == false) {
            unlock_flag_ = true;       // 标记：本轮已经开锁过
            task_stoped_ = true;
            auto_stop_flag_ = false;
        }

        disarm_stable_count_ = 0;       // 只要又变回开锁，就清零去抖计数
    }

    if(unlock_flag_ == true && armed == false){
        disarm_stable_count_++;//只有在曾经开锁过的状态才加
    }
    else if(unlock_flag_ == false && armed == false){
        disarm_stable_count_ = 0;       // 一直没开锁过，不做自动 stop
    }

    //判断是否为从开锁到关索的状态并且判断是否是第一次运行
    if(unlock_flag_ == true && auto_stop_flag_ == false && disarm_stable_count_ >= 5){
        auto_stop_flag_ = true;
        unlock_flag_ = false;
        task_stoped_ = false;
    }
}

void AirborneNode::handleBarcodeCapture(
    const drone_msgs::msg::BarcodeCapture::SharedPtr msg)
{
    drone_msgs::msg::BarcodeCapture vision_msg;
    vision_msg.stamp = msg->stamp;
    vision_msg.barcode = msg->barcode;
    vision_barcode_pub_->publish(vision_msg);
    //接收到视觉系统发布的条形码捕获消息后，直接转发到自己的条形码捕获话题上
    barcode_pub_->publish(*msg);
}

void AirborneNode::handleStartOffboard(
    const std::shared_ptr<drone_msgs::srv::StartOffboard::Request> request,
    std::shared_ptr<drone_msgs::srv::StartOffboard::Response> response)
{
    (void)request;

    if (offboard_started_) {
        response->success = false;
        //response->message = "offboard 已经启动，拒绝重复启动";
        return;
    }

    if (!mission_uploaded_) {
        response->success = false;
        //response->message = "mission YAML 尚未上传";
        offboard_started_ = false;
        return;
    }

    if (current_mission_path_.empty()) {
        response->success = false;
        //response->message = "mission 路径为空";
        offboard_started_ = false;
        return;
    }

    if (!std::filesystem::exists(current_mission_path_)) {
        response->success = false;
        //response->message = "mission 文件不存在";
        offboard_started_ = false;
        return;
    }

    const bool ok = startOffboardCommand();
    if (!ok) {
        response->success = false;
        //response->message = "offboard 启动失败";
        offboard_started_ = false;
        return;
    }

    current_task_name_ = request->request_source;
    offboard_started_ = true;
    response->success = true;
    //response->message = "offboard 启动成功，等待ready信号确认";
    response->message = "初始化成功，准备巡检";
}

void AirborneNode::handleStartTask(
    //使用std::shared_ptr来接收请求和响应对象
    const std::shared_ptr<drone_msgs::srv::StartTask::Request> request,
    std::shared_ptr<drone_msgs::srv::StartTask::Response> response)
{
    (void)request;

    if (task_started_) {
        response->success = false;
        //response->message = "start 已经启动，拒绝重复启动";
        return;
    }

    const bool ok = startTaskCommand();
    if (!ok) {
        response->success = false;
        //response->message = "task启动失败";
        return;
    }

    current_task_name_ = request->task_name;

    task_started_ = true;
    response->success = true;
    response->message = "开始巡检";
}

void AirborneNode::handleStopPush(
    //使用std::shared_ptr来接收请求和响应对象
    const std::shared_ptr<drone_msgs::srv::StartTask::Request> request,
    std::shared_ptr<drone_msgs::srv::StartTask::Response> response)
{
    (void)request;

    if (task_stoped_) {
        response->success = false;
        //response->message = "当前没有启动offboard";
        return;
    }

    std::string error_message;
    const bool ok = stopTaskCommand(error_message);
    if (!ok) {
        response->success = false;
        //response->message = "offboard 停止失败";
        task_stoped_ = false;
        return;
    }

    current_task_name_ = request->task_name;

    task_stoped_ = true;
    offboard_started_ = false;
    response->success = true;
    response->message = "巡检任务结束";
}

bool AirborneNode::saveMissionYamlToFile(
    const std::string &yaml_text,
    std::string &saved_path,
    std::string &error_message)
{
    //将接收到的yaml字符串保存到文件中，并返回保存路径和错误信息
    const std::string dir_path = "/home/orangepi/drone_ws/src/drone_mission/config";
    const std::string file_path = dir_path + "/ground_mission.yaml";

    //使用std::filesystem创建目录，如果目录不存在的话
    try {
        std::filesystem::create_directories(dir_path);
    } catch (const std::exception &e) {
        //error_message = std::string("创建 mission 目录失败: ") + e.what();
        return false;
    }

    //使用std::ofstream打开文件进行写入，如果文件无法打开则返回错误信息
    std::ofstream out(file_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        error_message = "路线无法打开";
        return false;
    }

    //将yaml文本写入文件，并检查写入是否成功，如果写入失败则返回错误信息
    out << yaml_text;
    out.close();

    if (!out) {
        error_message = "路线解析错误";
        return false;
    }

    //保存路径并返回成功标志
    saved_path = file_path;
    return true;
}

void AirborneNode::handleUploadMissionSummary(
    const std::shared_ptr<drone_msgs::srv::UploadMissionSummary::Request> request,
    std::shared_ptr<drone_msgs::srv::UploadMissionSummary::Response> response)
{
    if (!request) {
        response->success = false;
        response->message = "请求为空";
        response->saved_path = "";
        response->action_count = 0;
        return;
    }

    //保存当前上传方式内容
    waypoint_or_button_ = request->summary.compress_straight_segments;

    if (request->points.empty() && waypoint_or_button_ == true) {
        response->success = false;
        response->message = "路线点为空";
        response->saved_path = "";
        response->action_count = 0;
        return;
    }

    std::vector<AirborneWorldCoord> path_points;
    path_points.reserve(request->points.size());
    for (const auto &point : request->points) {
        path_points.push_back(AirborneWorldCoord{point.x, point.y, point.z, point.yaw});
    }

    if(waypoint_or_button_ == true){
        //从mission summary中提取选项参数，并使用路径点和选项参数生成mission yaml文本，同时统计mission action的数量
        const auto options = AirborneMissionYamlBuilder::fromMissionSummary(request->summary);
        const QString mission_yaml = AirborneMissionYamlBuilder::buildMissionYaml(path_points, options);
        const uint32_t action_count = AirborneMissionYamlBuilder::countMissionActions(path_points, options);

        std::string saved_path;
        std::string error_message;
        //将生成的mission yaml文本保存到文件中，并获取保存路径和错误信息
        const bool ok = saveMissionYamlToFile(mission_yaml.toStdString(), saved_path, error_message);

        if (!ok) {
            response->success = false;
            response->message = error_message;
            response->saved_path = "";
            response->action_count = 0;
            mission_uploaded_ = false;
            current_mission_path_.clear();
            return;
        }
    }

    mission_uploaded_ = true;

    if(waypoint_or_button_ == false)
    {
        current_mission_path_ = "/home/sunrise/drone_ws/src/drone_mission/warehouse/mission.yaml";
    }
    else
    {
        current_mission_path_ = "/home/sunrise/drone_ws/src/drone_mission/config/ground_mission.yaml";
    }

    response->success = true;
    //response->message = "mission 摘要上传成功，机载端已生成 YAML";
    response->message = "正在初始化";
    response->saved_path = current_mission_path_;
    response->action_count = 0;
}

bool AirborneNode::startOffboardCommand()
{
    if (current_mission_path_.empty()) {
        return false;
    }

    if (offboard_process_ && offboard_process_->state() != QProcess::NotRunning) {
        RCLCPP_WARN(this->get_logger(), "offboard process is already running");
        return false;
    }

    if (!offboard_process_) {
        offboard_process_ = new QProcess();

        QObject::connect(offboard_process_, &QProcess::readyReadStandardOutput, [this]() {
            const QByteArray text = offboard_process_->readAllStandardOutput();
            if (!text.isEmpty()) {
                RCLCPP_INFO(this->get_logger(), "[offboard stdout]\n%s", text.constData());
            }
        });

        QObject::connect(offboard_process_, &QProcess::readyReadStandardError, [this]() {
            const QByteArray text = offboard_process_->readAllStandardError();
            if (!text.isEmpty()) {
                RCLCPP_ERROR(this->get_logger(), "[offboard stderr]\n%s", text.constData());
            }
        });

        QObject::connect(
            offboard_process_,
            QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            [this](int exit_code, QProcess::ExitStatus exit_status) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "offboard process finished, exit_code=%d, exit_status=%d",
                    exit_code,
                    static_cast<int>(exit_status));
                offboard_started_ = false;
            });
    }

    const QString command = QString(
        "source /opt/ros/humble/setup.bash && "
        "source ~/drone_ws/install/setup.bash && "
        "exec ros2 launch drone_bringup run_offboard.launch.py "
        "mission_config_path:=%1 "
        "enable_offboard_control:=true ")
        .arg(QString::fromStdString(current_mission_path_));

    RCLCPP_INFO(this->get_logger(), "starting offboard process with command: %s", command.toStdString().c_str());
    offboard_process_->start("bash", QStringList() << "-lc" << command);

    task_stoped_ = false;

    if (!offboard_process_->waitForStarted(3000)) {
        RCLCPP_ERROR(this->get_logger(), "failed to start offboard process");
        return false;
    }

    return true;
}

bool AirborneNode::startTaskCommand()
{
    const std::string command =
        "bash -lc 'source /opt/ros/humble/setup.bash && "
        "source ~/drone_ws/install/setup.bash && "
        "ros2 topic pub --once /start_mission std_msgs/msg/Empty \"{}\"'";

    std::thread([this, command]() {
        RCLCPP_INFO(this->get_logger(), "starting takeoff program command...");
        const int ret = std::system(command.c_str());
        RCLCPP_INFO(this->get_logger(), "takeoff program command exited with code=%d", ret);
        task_started_ = false;
    }).detach();

    //task_stoped_ = true;

    return true;
}

bool AirborneNode::killResidualOffboardNodes(std::string &error_message)
{
    const std::string soft_cmd =
        "bash -lc 'pkill -INT -f \"mission_controller_node|tf_bridge_node|route_comm_node\"'";
    const int soft_ret = std::system(soft_cmd.c_str());

    std::this_thread::sleep_for(std::chrono::seconds(1));

    const std::string hard_cmd =
        "bash -lc 'pkill -KILL -f \"mission_controller_node|tf_bridge_node|route_comm_node\"'";
    const int hard_ret = std::system(hard_cmd.c_str());

    if (soft_ret != 0 && hard_ret != 0) {
        error_message = "残留 offboard 节点清理失败";
        return false;
    }

    error_message.clear();
    return true;
}

bool AirborneNode::stopTaskCommand(std::string &error_message)
{
    std::string stop_error;
    const bool stop_ok = stopOffboardProcess(stop_error);

    std::string cleanup_error;
    const bool cleanup_ok = killResidualOffboardNodes(cleanup_error);

    offboard_started_ = false;

    if (stop_ok || cleanup_ok) {
        error_message.clear();
        return true;
    }

    error_message = stop_error;
    if (!cleanup_error.empty()) {
        error_message += "; " + cleanup_error;
    }
    return false;
}

bool AirborneNode::stopOffboardProcess(std::string &error_message)
{
    if (!offboard_process_) {
        error_message = "offboard 进程对象不存在";
        return false;
    }

    if (offboard_process_->state() == QProcess::NotRunning) {
        offboard_started_ = false;
        error_message = "offboard 当前未运行";
        return false;
    }

    offboard_process_->terminate();
    if (!offboard_process_->waitForFinished(5000)) {
        offboard_process_->kill();
        if (!offboard_process_->waitForFinished(3000)) {
            error_message = "offboard 进程无法停止";
            return false;
        }
    }

    offboard_started_ = false;
    error_message.clear();

    return true;
}

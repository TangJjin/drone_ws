#include "drone_warehouse/ros_manager.hpp"

/*********************ros移植部分***********************/
RosManager::RosManager(QObject *parent)
    : QObject(parent),
    node_(std::make_shared<rclcpp::Node>("ground_qt_node"))
{
    // 这里沿用来源工程的做法：RosManager 一创建就把所有 ROS 订阅、服务客户端和定时器接口建好。
    // 后面 MainWindow 只需要负责调用 start() 启动 spin 线程，并接收这些 Qt 信号即可。
    setupRosInterfaces();
}
/******************************************************/

RosManager::~RosManager()
{
    //取消执行器的 spinning，并等待线程结束
    executor_.cancel();

    //如果线程可连接，则连接线程
    if (spin_thread_.joinable())
    {
        spin_thread_.join();
    }
}

void RosManager::setupRosInterfaces()
{
    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个订阅者，订阅无人机状态话题，消息类型为自定义消息
    status_sub_ = node_->create_subscription<drone_msgs::msg::DroneStatus>(
        "/serial/drone/status", 
        status_qos, 
        [this](const drone_msgs::msg::DroneStatus::SharedPtr msg)//回调
        {
            connected = msg->connected;
            battery_percent = msg->battery_percent;
            flight_mode = static_cast<int>(msg->flight_mode);
            armed = msg->armed;
            const QString task_name = QString::fromStdString(msg->state);

            //使用Qt的信号槽机制在线程安全的方式下发状态更新信号，包含连接状态、电量百分比、飞行模式和解锁状态等信息
            QMetaObject::invokeMethod(
                this,
                [this, task_name]() {
                    emit statusUpdated(
                        connected,
                        battery_percent,
                        flight_mode,
                        armed,
                        task_name);
                },
                Qt::QueuedConnection);
        });

    auto control_status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    //创建一个订阅者，订阅控制程序状态话题，消息类型为自定义消息
    task_status_sub_ = node_->create_subscription<drone_msgs::msg::TaskStatus>(
        "/serial/drone/task/status", 
        control_status_qos, 
        [this](const drone_msgs::msg::TaskStatus::SharedPtr msg)//回调
        {
            const bool task_running = msg->task_running;
            const int action_step = msg->action_step;
            const int action_num = msg->action_num;
            const QString action_name = QString::fromStdString(msg->action_name);

            //使用Qt的信号槽机制在线程安全的方式下发状态更新信号，包含连接状态、电量百分比、飞行模式和解锁状态等信息
            QMetaObject::invokeMethod(
                this,
                [this, task_running, action_step, action_num, action_name]() {
                    emit action_statusUpdated(
                        task_running,
                        action_step,
                        action_num,
                        action_name);
                },
                Qt::QueuedConnection);
        });

    auto control_path_ready_qos =
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    //创建一个订阅者，订阅控制程序的确认消息，消息类型为自定义消息
    ready_status_sub_ = node_->create_subscription<drone_msgs::msg::ReadyStatus>(
        "/serial/drone/control/path_ready", 
        control_path_ready_qos, 
        [this](const drone_msgs::msg::ReadyStatus::SharedPtr msg)//回调
        {
            const bool ready = msg->air_route_ready;

            QMetaObject::invokeMethod(
                this,
                [this, ready]() {
                    emit pathReadyChanged(ready);
                },
                Qt::QueuedConnection);
        });

    auto return_world_group_qos =
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    //创建一个订阅者，订阅控制程序的路线消息，消息类型为自定义消息
    return_world_group_sub_ = node_->create_subscription<drone_msgs::msg::WorldGroup>(
        "/serial/drone/return/world_group", 
        return_world_group_qos, 
        [this](const drone_msgs::msg::WorldGroup::SharedPtr msg)//回调
        {
            QVector<WorldCoord> points;
            points.reserve(static_cast<int>(msg->points.size()));//预先分配空间以提高效率

            for (const auto &point : msg->points) {
                points.push_back(WorldCoord{point.x, point.y});
            }

            QMetaObject::invokeMethod(
                this,
                [this, points]() {
                    //参数为一个包含坐标点的QVector
                    emit returnWorldGroupUpdated(points);
                },
                Qt::QueuedConnection);
        });

    auto barcode_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个订阅者，订阅条形码捕获话题，消息类型为自定义消息
    barcode_sub_ = node_->create_subscription<drone_msgs::msg::BarcodeCapture>(
        "/drone/barcode_capture",
        barcode_qos,
        [this](const drone_msgs::msg::BarcodeCapture::SharedPtr msg)
        {
            //将接收到的条形码捕获消息中的图像数据转换为QByteArray格式，以便在Qt界面中使用
            const QByteArray image_bytes(
                reinterpret_cast<const char*>(msg->image_data.data()),
                static_cast<int>(msg->image_data.size()));

            //使用Qt的信号槽机制在线程安全的方式下发条形码捕获信号，包含捕获到的条形码数据、图像数据、图像格式和时间文本等信息
            const QString barcode = QString::fromStdString(msg->barcode);
            const QString image_format = QString::fromStdString(msg->image_format);
            const qint64 time_ms =
                static_cast<qint64>(msg->stamp.sec) * 1000LL +
                static_cast<qint64>(msg->stamp.nanosec) / 1000000LL;
            const QString time_text = 
                QDateTime::fromMSecsSinceEpoch(time_ms,Qt::LocalTime)
                .toString("yyyy-MM-dd hh:mm:ss");

            //使用QMetaObject::invokeMethod来在线程安全的方式下发信号，确保信号在Qt的事件循环中被正确处理
            QMetaObject::invokeMethod(
                this,
                [this, barcode, image_bytes, image_format, time_text]() {
                    emit barcodeCaptured(barcode, image_bytes, image_format, time_text);
                },
                Qt::QueuedConnection);
        });

    auto barcode_sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    //创建一个订阅者，订阅条形码捕获话题，消息类型为自定义消息
    vision_barcode_sub_ = node_->create_subscription<drone_msgs::msg::BarcodeCapture>(
        "/serial/drone/vision/barcode",
        barcode_sub_qos,
        [this](const drone_msgs::msg::BarcodeCapture::SharedPtr msg)
        {
            //使用Qt的信号槽机制在线程安全的方式下发条形码捕获信号，包含捕获到的条形码数据、图像数据、图像格式和时间文本等信息
            const QString barcode = QString::fromStdString(msg->barcode);
            const qint64 time_ms =
                static_cast<qint64>(msg->stamp.sec) * 1000LL +
                static_cast<qint64>(msg->stamp.nanosec) / 1000000LL;
            const QString time_text = 
                QDateTime::fromMSecsSinceEpoch(time_ms,Qt::LocalTime)
                .toString("yyyy-MM-dd hh:mm:ss");

            QMetaObject::invokeMethod(
                this,
                [this, barcode, time_text]() {
                    emit visionBarcodeCaptured(barcode, time_text);
                },
                Qt::QueuedConnection);
        });

    auto local_position_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    //创建一个订阅者，订阅无人机本地位置话题，消息类型为geometry_msgs::msg::PoseStamped
    local_position_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/serial/drone/local_position",
        local_position_qos,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            const double x = msg->pose.position.x;
            const double y = msg->pose.position.y;
            const double z = msg->pose.position.z;

            const double qx = msg->pose.orientation.x;
            const double qy = msg->pose.orientation.y;
            const double qz = msg->pose.orientation.z;
            const double qw = msg->pose.orientation.w;

            //使用Qt的信号槽机制在线程安全的方式下发位置更新信号，包含无人机的二维位置坐标和高度等信息
            QMetaObject::invokeMethod(
                this,
                [this, x, y, z, qx, qy, qz, qw]() {
                    emit positionUpdated(x, y, z, qx, qy, qz, qw);
                },
                Qt::QueuedConnection);
        });

auto delta_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    delta_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
        "/serial/drone/pose_yaw_compare/delta",
        delta_qos,
        [this](const geometry_msgs::msg::Vector3::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(delta_mutex_);
            latest_dx_ = msg->x;
            latest_dy_ = msg->y;
            latest_dyaw_ = msg->z;
            last_delta_msg_time_ = std::chrono::steady_clock::now();//记录最后一次接收到delta消息的时间点
            has_delta_msg_ = true;//设置标志，表示已经接收到过delta消息
        });

    //创建一个服务客户端，用于调用任务启动服务
    start_task_client_ = node_->create_client<drone_msgs::srv::StartTask>(
        "/serial/drone/start_task");

    //创建一个服务客户端，用于调用路线重传服务
    stop_push_client_ = node_->create_client<drone_msgs::srv::StartTask>(
        "/serial/drone/stop_push");

    //创建一个服务客户端，用于调用offboard启动服务
    start_offboard_client_ = node_->create_client<drone_msgs::srv::StartOffboard>(
        "/serial/drone/start_offboard");

    //创建一个服务客户端，用于调用任务路线与参数上传服务
    upload_mission_summary_client_ = node_->create_client<drone_msgs::srv::UploadMissionSummary>(
        "/serial/drone/upload_mission_summary");

    using namespace std::chrono_literals;
    timer_ = node_->create_wall_timer(
        0.5s,
        [this]()
        {
            double dx, dy, dyaw;
            bool delta_valid = false;
            {
                std::lock_guard<std::mutex> lock(delta_mutex_);//在访问共享数据时加锁，确保线程安全
                dx = latest_dx_;
                dy = latest_dy_;
                dyaw = latest_dyaw_;

                if (has_delta_msg_) {
                    const auto now = std::chrono::steady_clock::now();
                    //如果当前时间与最后一次接收到delta消息的时间点之间的间隔不超过1秒钟，则认为delta数据有效，否则认为无效
                    delta_valid = (now - last_delta_msg_time_) <= std::chrono::seconds(1);
                }
            }

            QMetaObject::invokeMethod(
                this,
                [this, dx, dy, dyaw, delta_valid]() {
                    emit deltaUpdated(dx, dy, dyaw, delta_valid);
                },
                Qt::QueuedConnection);
        });
}

void RosManager::start()
{
    //如果已经启动，则直接返回
    if (started_)
    {
        return;
    }

    //将ROS节点添加到执行器中，并启动一个线程来运行ROS spinning
    executor_.add_node(node_);
    spin_thread_ = std::thread([this]()
    {
        executor_.spin();
    });

    started_ = true;
}

/*********************ros移植部分***********************/
void RosManager::startTask()
{
    // 这一段严格沿用来源工程的语义：
    // 1. 调用 /drone/start_task 服务；
    // 2. 请求里的 task_name 固定写 action_task_run；
    // 3. 服务回包后，通过 commandResult 信号把结果回传给界面。
    if (!start_task_client_ || !start_task_client_->service_is_ready())
    {
        emit commandResult(false, "服务 /drone/start_task 未就绪");
        return;
    }

    auto request = std::make_shared<drone_msgs::srv::StartTask::Request>();
    request->task_name = "action_task_run";//来源工程里的固定任务名，表示开始执行任务

    start_task_client_->async_send_request(
        request,
        [this](rclcpp::Client<drone_msgs::srv::StartTask>::SharedFuture future)
        {
            const auto response = future.get();
            QMetaObject::invokeMethod(
                this,
                [this, response]() {
                    emit commandResult(
                        response->success,
                        QString::fromStdString(response->message));
                },
                Qt::QueuedConnection);
        });
}
/******************************************************/

/*********************ros移植部分***********************/
void RosManager::stopTask()
{
    // 这一段也保持来源工程逻辑不变：
    // 1. 调用 /drone/stop_push 服务；
    // 2. 请求体里沿用 stop_task 这个 task_name；
    // 3. 用 stopcommandResult 把回包结果下发给界面。
    if (!stop_push_client_ || !stop_push_client_->service_is_ready())
    {
        emit stopcommandResult(false, "服务 /drone/stop_push 未就绪");
        return;
    }

    auto request = std::make_shared<drone_msgs::srv::StartTask::Request>();
    request->task_name = "stop_task";//来源工程里的固定停止任务名

    stop_push_client_->async_send_request(
        request,
        [this](rclcpp::Client<drone_msgs::srv::StartTask>::SharedFuture future)
        {
            const auto response = future.get();
            QMetaObject::invokeMethod(
                this,
                [this, response]() {
                    emit stopcommandResult(
                        response->success,
                        QString::fromStdString(response->message));
                },
                Qt::QueuedConnection);
        });
}
/******************************************************/

/*********************ros移植部分***********************/
void RosManager::requestStartOffboard()
{
    // 这里保持来源工程的 offboard 启动逻辑：
    // 1. 调用 /drone/start_offboard 服务；
    // 2. request_source 固定写 ground_station_upload_flow；
    // 3. 用 offboardCommandResult 把执行结果回传给界面。
    if (!start_offboard_client_ || !start_offboard_client_->service_is_ready())
    {
        emit offboardCommandResult(false, "服务 /drone/start_offboard 未就绪");
        return;
    }

    auto request = std::make_shared<drone_msgs::srv::StartOffboard::Request>();
    request->request_source = "ground_station_upload_flow";//沿用来源工程的固定请求来源标识

    start_offboard_client_->async_send_request(
        request,
        [this](rclcpp::Client<drone_msgs::srv::StartOffboard>::SharedFuture future)
        {
            const auto response = future.get();
            QMetaObject::invokeMethod(
                this,
                [this, response]() {
                    emit offboardCommandResult(
                        response->success,
                        QString::fromStdString(response->message));
                },
                Qt::QueuedConnection);
        });
}
/******************************************************/

/*********************ros移植部分***********************/
void RosManager::uploadMissionSummary(const QVector<WorldCoord> &path_points,
                                      const drone_msgs::msg::MissionSummary &summary)
{
    // 这是四个服务里依赖最多的一条链路，保持来源工程的行为：
    // 1. 服务名固定是 /drone/upload_mission_summary；
    // 2. 先检查服务是否就绪、路径是否为空；
    // 3. 请求里同时带 MissionSummary 和 WorldPoint 列表；
    // 4. 返回时把 success / message / saved_path 通过 missionUploadFinished 发给界面。
    if (!upload_mission_summary_client_ || !upload_mission_summary_client_->service_is_ready())
    {
        emit missionUploadFinished(false, "服务 /drone/upload_mission_summary 未就绪", "");
        return;
    }

    auto request = std::make_shared<drone_msgs::srv::UploadMissionSummary::Request>();
    request->summary = summary;//先把整份 mission 参数摘要写进请求体

    for (const auto &point : path_points)
    {
        drone_msgs::msg::WorldPoint world_point;
        world_point.x = point.x;
        world_point.y = point.y;
        world_point.z = point.z;
        world_point.yaw = point.yaw;
        request->points.push_back(world_point);//再把每一个世界坐标点追加进 points 列表
    }

    upload_mission_summary_client_->async_send_request(
        request,
        [this](rclcpp::Client<drone_msgs::srv::UploadMissionSummary>::SharedFuture future)
        {
            const auto response = future.get();
            QMetaObject::invokeMethod(
                this,
                [this, response]() {
                    emit missionUploadFinished(
                        response->success,
                        QString::fromStdString(response->message),
                        QString::fromStdString(response->saved_path));
                },
                Qt::QueuedConnection);
        });
}
/******************************************************/

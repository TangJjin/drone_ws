#include "drone_warehouse/mainwindow.hpp"

#include "drone_warehouse/models.hpp"
#include "drone_warehouse/scene_view.hpp"
#include "drone_warehouse/shelf_info_dialog.hpp"
#include "drone_warehouse/top_status_bar.hpp"
#include "drone_warehouse/color_palette.hpp"
#include "drone_warehouse/ros_manager.hpp"
#include "drone_warehouse/gpio_output.hpp"
#include "drone_warehouse/ai_diff_analyzer.hpp"
#include "drone_warehouse/shelf_panel_storage.hpp"

#include <algorithm>
#include <cmath>
#include <QDialog>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QResizeEvent>
#include <QScrollArea>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QImage>
#include <QPixmap>
#include <QRegularExpression>
#include <stdexcept>
#include <QProcess>
#include <QStringList>
#include <QCoreApplication>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    /*********************ros移植部分***********************/
    // 注册跨线程 Qt 信号里会用到的 WorldCoord 元类型。
    // 原 drone_qt 工程里 returnWorldGroupUpdated 会跨线程发 QVector<WorldCoord>，
    // 这里先把类型注册好，避免后面一旦启用这条信号链时 Qt 不认识这个自定义类型。
    qRegisterMetaType<WorldCoord>("WorldCoord");
    qRegisterMetaType<QVector<WorldCoord>>("QVector<WorldCoord>");
    /******************************************************/
    //setWindowFlags(Qt::Window | Qt::FramelessWindowHint);//去掉主窗口系统标题栏，不再显示上方 warehouse_gcs 那一层
    //resize(1024, 600);
    //setFixedSize(1024, 540);//设置初始窗口大小
    showFullScreen();
    activateWindow();
    setWindowFlags(Qt::FramelessWindowHint);
    raise();

    setupUi();
    setupFloatingWidgets();
    setupConnections();
    applyWindowStyle();
    setupDemoData();

    /*********************ros移植部分***********************/
    // 本轮先把时间触发接口和状态变量接进来，但按用户要求默认关闭。
    // 这样后面如果你要补一个“设置触发时刻”的入口，只需要把这两个值改掉即可。
    mission_trigger_time_text_ = "";
    mission_time_trigger_enabled_ = true;
    top_status_bar_->setTriggerTime(mission_trigger_time_text_);//传入想要定的时间
    top_status_bar_->setTimeTriggerEnabled(mission_time_trigger_enabled_);//传入是否开启时间定时
    /******************************************************/
    //top_status_bar_->setConnected(true);
    updateOverlayGeometry();

    /*********************ros移植部分***********************/
    // 界面和信号槽都准备好之后，再启动 RosManager 的 spin 线程。
    // 这样一来，后面 ROS 回调一到，就能立刻把状态发到已经存在的 Qt 控件上。
    if (ros_manager_)
    {
        ros_manager_->start();
    }
    /******************************************************/
}

void MainWindow::setupUi()
{
    central_container_ = new QWidget(this);
    //设置主容器
    setCentralWidget(central_container_);

    //创建主场景视图和顶部状态栏，并把它们放在主容器里，方便统一管理布局和坐标
    scene_view_ = new SceneView(central_container_);
    //主场景视图占满整个主容器，悬浮控件会在上面调整位置
    scene_view_->setGeometry(central_container_->rect());
    
    top_status_bar_ = new TopStatusBar(central_container_);
    //创建货架信息弹窗模板，先由主窗口持有，点击顶部按钮时再弹出显示
    shelf_info_dialog_ = new ShelfInfoDialog(this);

    /*********************ros移植部分***********************/
    // RosManager 是本次从 drone_qt 移植过来的 ROS 入口。
    // 这里先在主窗口创建它，后面统一在 setupConnections() 里连信号，在构造末尾启动 spin 线程。
    ros_manager_ = new RosManager(this);
    /*****************************************************/
}

void MainWindow::setupFloatingWidgets()
{
    /***********************日志控件*************************/

    log_panel_ = new QWidget(central_container_);
    auto *log_layout = new QVBoxLayout(log_panel_);
    log_panel_->setObjectName("logSwitchPanel");
    log_panel_->setContentsMargins(5, 5, 5, 5);//姿态面板内部边框留白

    run_log_view_ = new QPlainTextEdit(log_panel_);
    run_log_view_->setReadOnly(true);
    run_log_view_->setMaximumBlockCount(1000);

    log_layout->addWidget(run_log_view_);

    clock_timer_ = new QTimer(this);//新建定时器
    run_log_view_->appendPlainText("日志初始化成功");
    clock_timer_->start(5000);


    logwaypoint_panel_ = new QWidget(central_container_);
    auto *logwaypoint_layout = new QVBoxLayout(logwaypoint_panel_);
    logwaypoint_panel_->setObjectName("logwaypointSwitchPanel");
    logwaypoint_panel_->setContentsMargins(5, 5, 5, 5);//姿态面板内部边框留白

    waypoint_log_view_ = new QPlainTextEdit(logwaypoint_panel_);
    waypoint_log_view_->setReadOnly(true);
    waypoint_log_view_->setMaximumBlockCount(1000);

    logwaypoint_layout->addWidget(waypoint_log_view_);

    //waypoint_log_view_->appendPlainText("航点日志初始化成功");

    ai_log_panel_ = new QWidget(central_container_);
    auto *ai_log_layout = new QVBoxLayout(ai_log_panel_);
    ai_log_panel_->setObjectName("aiLogPanel");
    ai_log_panel_->setContentsMargins(5, 5, 5, 5);

    // auto *ai_log_title = new QLabel("AI分析", ai_log_panel_);
    // ai_log_title->setObjectName("aiLogTitle");
    ai_log_view_ = new QPlainTextEdit(ai_log_panel_);
    ai_log_view_->setReadOnly(true);
    ai_log_view_->setMaximumBlockCount(1000);

    // ai_log_layout->addWidget(ai_log_title);
    ai_log_layout->addWidget(ai_log_view_);
    // ai_log_view_->appendPlainText("AI分析日志初始化成功");

    /*******************************************************/

    /*********************悬浮姿态控件***********************/

    attitude_panel_ = new QWidget(central_container_);
    auto *attitude_layout = new QGridLayout(attitude_panel_);
    attitude_panel_->setObjectName("attitudeSwitchPanel");
    attitude_layout->setContentsMargins(12, 12, 12, 12);//姿态面板内部边框留白

    attitude_layout->addWidget(new QLabel("高度", attitude_panel_), 0, 0);
    altitude_value_label_ = new QLabel("0.0 m", attitude_panel_);
    attitude_layout->addWidget(altitude_value_label_, 0, 1);

    attitude_layout->addWidget(new QLabel("航向", attitude_panel_), 1, 0);
    yaw_value_label_ = new QLabel("0.0°", attitude_panel_);
    attitude_layout->addWidget(yaw_value_label_, 1, 1);

    attitude_layout->addWidget(new QLabel("电量", attitude_panel_), 2, 0);
    battery_value_label_ = new QLabel("N/A", attitude_panel_);
    attitude_layout->addWidget(battery_value_label_, 2, 1);

    attitude_layout->addWidget(new QLabel("模式", attitude_panel_), 3, 0);
    mode_value_label_ = new QLabel("MODE_UNKNOWN", attitude_panel_);
    attitude_layout->addWidget(mode_value_label_, 3, 1);


    /*******************************************************/

    /*********************模式切换滑块***********************/

    view_mode_widget_ = new QWidget(central_container_);
    auto *slider_layout = new QHBoxLayout(view_mode_widget_);
    slider_layout->setContentsMargins(1, 1, 1, 1);//滑块模块内部留白
    slider_layout->setSpacing(1);//滑块中各元素间距

    view_mode_left_label_ = new QLabel("2D", view_mode_widget_);
    view_mode_right_label_ = new QLabel("3D", view_mode_widget_);
    view_mode_slider_ = new QSlider(Qt::Horizontal, view_mode_widget_);//水平滑动条
    view_mode_slider_->setRange(0, 1);//滑块取值范围0-1，0表示2D模式，1表示3D模式
    view_mode_slider_->setValue(1);//默认3D模式
    view_mode_slider_->setFixedHeight(35);

    slider_layout->addWidget(view_mode_left_label_);
    slider_layout->addWidget(view_mode_slider_);
    slider_layout->addWidget(view_mode_right_label_);

    /*******************************************************/

    /*********************视角切换滑块***********************/

    view_Perspective_widget_ = new QWidget(central_container_);
    auto *slider_Perspective_layout = new QHBoxLayout(view_Perspective_widget_);

    view_2D_widget_ = new QWidget(central_container_);
    auto *slider_2D_layout = new QHBoxLayout(view_2D_widget_);

    slider_Perspective_layout->setContentsMargins(1, 1, 1, 1);//滑块模块内部留白
    slider_Perspective_layout->setSpacing(1);//滑块中各元素间距

    slider_2D_layout->setContentsMargins(1, 1, 1, 1);//滑块模块内部留白
    slider_2D_layout->setSpacing(1);//滑块中各元素间距

    view_Perspective_slider_ = new QSlider(Qt::Horizontal, view_Perspective_widget_);//水平滑动条
    view_Perspective_slider_->setRange(0, 3);//滑块取值范围0-3
    view_Perspective_slider_->setValue(0);
    view_Perspective_slider_->setFixedHeight(35);

    view_2D_slider_ = new QSlider(Qt::Horizontal, view_2D_widget_);//水平滑动条
    view_2D_slider_->setRange(0, 2);//滑块取值范围0-2，0表示上视图，1表示左视图，2表示右视图
    view_2D_slider_->setValue(0);
    view_2D_slider_->setFixedHeight(35);

    //slider_Perspective_layout->addWidget(view_Perspective_left_label_);
    slider_Perspective_layout->addWidget(view_Perspective_slider_);
    //slider_Perspective_layout->addWidget(view_Perspective_right_label_);

    slider_2D_layout->addWidget(view_2D_slider_);

    /*******************************************************/

    top_status_bar_->raise();//确保悬浮控件在主场景视图上面
    log_panel_->raise();//确保日志区主场景视图上面
    logwaypoint_panel_->raise();//确保日志区主场景视图上面
    ai_log_panel_->raise();//确保AI分析日志区在主场景视图上面
    attitude_panel_->raise();//确保姿态面板在主场景视图上面
    view_mode_widget_->raise();//确保视图模式控件在主场景视图上面
    view_Perspective_widget_->raise();//确保视角切换控件在主场景视图上面
    view_2D_widget_->raise();//确保2D视角切换控件在主场景视图上面

    view_Perspective_widget_->show();
    view_2D_widget_->hide();

    }

void MainWindow::setupConnections()
{
    connect(top_status_bar_, &TopStatusBar::exitRequested, this, [this]() {
        close();
    });

    connect(clock_timer_, &QTimer::timeout, this, [this]() {//每秒触发刷新一次日志文本
        run_log_view_->clear();
        clock_timer_->stop();
    });

    //连接视图模式滑动条的值改变信号，根据值切换视图模式
    connect(view_mode_slider_, &QSlider::valueChanged, this, [this](int value) {
        if (value == 0)
        {
            scene_view_->setViewMode(ViewMode::Top2D);
            view_Perspective_widget_->hide();
            view_2D_widget_->show();
        }
        else
        {
            scene_view_->setViewMode(ViewMode::Pseudo3D);
            view_Perspective_widget_->show();
            view_2D_widget_->hide();
        }
    });

    //连接3D切换视角滑动条的值改变信号，根据值切换视图模式
    connect(view_Perspective_slider_, &QSlider::valueChanged, this, [this](int value) {
        if (value == 0)
        {
            scene_view_->setViewPerspectiveMode(ViewPerspectiveMode::Perspective225);
        }
        else if(value == 1)
        {
            scene_view_->setViewPerspectiveMode(ViewPerspectiveMode::Perspective315);
        }
        else if(value == 2)
        {
            scene_view_->setViewPerspectiveMode(ViewPerspectiveMode::Perspective45);
        }
        else if(value == 3)
        {
            scene_view_->setViewPerspectiveMode(ViewPerspectiveMode::Perspective135);
        }
    });

    //连接2D切换视角滑动条的值改变信号，根据值切换视图模式
    connect(view_2D_slider_, &QSlider::valueChanged, this, [this](int value) {
        if (value == 0)
        {
            scene_view_->setView2DMode(View2DMode::Perspectivetop);
        }
        else if(value == 1)
        {
            scene_view_->setView2DMode(View2DMode::Perspective90);
        }
        else if(value == 2)
        {
            scene_view_->setView2DMode(View2DMode::Perspective0);
        }
    });

    //连接顶部状态栏货架信息点击信号，打开货架信息窗口
    connect(top_status_bar_, &TopStatusBar::shelfButtonClicked, this, [this]() {
        //取出货物信息左下角的x，y坐标
        const QPoint button_bottom_left = top_status_bar_->shelfButtonBottomLeftGlobal();

        const int margin = 20;//让弹窗和顶部状态栏之间留一点空隙

        shelf_info_dialog_->adjustSize();
        shelf_info_dialog_->move(button_bottom_left.x(), button_bottom_left.y() + margin);//先把弹窗移动到货物信息下方
        shelf_info_dialog_->show();//显示弹窗
        shelf_info_dialog_->raise();//把弹窗提升到最前面
        shelf_info_dialog_->activateWindow();//让弹窗获取焦点，方便后续继续操作
    });

    connect(shelf_info_dialog_, &ShelfInfoDialog::slotDoubleClicked,
        this,
        [this](int shelf_index, const QString &side, int row, int col)
        {
            showShelfSlotImage(shelf_index, side, row, col);
        });

    // 地面站手动入库链路：
    // 1. 用户先在 ShelfInfoDialog 里选中一个格子。
    // 2. Dialog 通过串口拿到扫码文本后，发出 `manualStockInScanned(...)`。
    // 3. MainWindow 在这里接住信号，并把类别/包裹编号真正写回主数据 `shelf_panel_data_`。
    // 4. 写回完成后，再统一调用 `setShelfPanelData(...)` 把最新结果刷新回弹窗。
    connect(shelf_info_dialog_, &ShelfInfoDialog::manualStockInScanned,
        this,
        [this](int shelf_index, const QString &side, int row, int col,
               const QString &category_id, const QString &package_id)
        {
            applyManualStockIn(shelf_index, side, row, col, category_id, package_id);
        });

    // 地面站手动出库链路：
    // 1. 用户在弹窗里点击“出库”。
    // 2. Dialog 把“当前选中的货架/前后面/行列”发给 MainWindow。
    // 3. MainWindow 在这里统一执行清空，避免 Dialog 和主数据各改各的导致状态分叉。
    connect(shelf_info_dialog_, &ShelfInfoDialog::manualStockOutRequested,
        this,
        [this](int shelf_index, const QString &side, int row, int col, const QString &category_id, const QString &package_id)
        {
            applyManualStockOut(shelf_index, side, row, col, category_id, package_id);
        });

    /*********************ros通信相关***********************/
    
    if (ros_manager_)
    {
        connect(top_status_bar_, &TopStatusBar::scheduledcheckbuttonnClicked, 
            this, 
            [this](const QString &mission_trigger_time_text) {
            if(mission_trigger_time_text_flag_ == 1)
            {
                mission_trigger_time_text_ = mission_trigger_time_text;
                run_log_view_->appendPlainText(QString("已设置定时巡检：%1").arg(mission_trigger_time_text_));
                mission_trigger_time_text_flag_ = 0;
                top_status_bar_->setTriggerTime(mission_trigger_time_text_);
                triggerMissionUpload("time");
                clock_timer_->start(5000);
            }
            else
            {
                mission_trigger_time_text_ = "";
                run_log_view_->appendPlainText(QString("已关闭定时巡检"));
                mission_trigger_time_text_flag_ = 1;
                top_status_bar_->setTriggerTime(mission_trigger_time_text_);
                clock_timer_->start(5000);
            }
        });

        connect(top_status_bar_, &TopStatusBar::aiAnalyzeButtonClicked, this, [this]() {
            runClaudeApiDiffAnalysis();
        });

        connect(top_status_bar_, &TopStatusBar::executeButtonClicked, this, [this]() {
            triggerMissionUpload("button");
        });

        connect(top_status_bar_, &TopStatusBar::triggerTimeReached, this, [this](const QString &) {
            triggerMissionUpload("time");
        });

        connect(top_status_bar_, &TopStatusBar::waypointButtonClicked, this, [this]() {
            triggerMissionUpload("waypoint");
        });

        //清空航点
        connect(shelf_info_dialog_, &ShelfInfoDialog::clearWaypointRequested, this, [this]() {
            clearWaypointRequest();
        });

        //设置航点
        connect(shelf_info_dialog_, &ShelfInfoDialog::setWaypointRequested, 
            this, 
            [this](int shelf_index, const QString &side, int row, int col)
            {
                setWaypointRequest(shelf_index, side, row, col);
            });

        //查看任务yaml上传服务返回的内容
        connect(ros_manager_, &RosManager::missionUploadFinished,
            this,
            [this](bool success, const QString &message, const QString &saved_path)
            {
                handleMissionUploadFinished(success, message, saved_path);
            },
            Qt::QueuedConnection);

        //查看起飞启动服务返回的内容
        connect(ros_manager_, &RosManager::commandResult,
            this,
            [this](bool success, const QString &message)
            {
                //根据命令执行结果的成功与否，更新界面上的结果标签文本，显示相关消息
                updateCommandResult(success, message);
                run_log_view_->appendPlainText(QString("%1").arg(message));
                //clock_timer_->start(5000);
            },
            Qt::QueuedConnection);

        //查看停止服务返回的内容
        connect(ros_manager_, &RosManager::stopcommandResult,
            this,
            [this](bool success, const QString &message)
            {
                if(success){
                    // path_ready_ = false;
                    // waiting_push_result_ = false;
                    // start_button_->setEnabled(false);
                    // delta_result_ = true;
                    //push_button_->setEnabled(true);
                    run_log_view_->appendPlainText(QString("%1").arg(message));
                    clock_timer_->start(5000);
                }
            },
            Qt::QueuedConnection);

        //查看offboard启动服务返回的内容
        connect(ros_manager_, &RosManager::offboardCommandResult,
            this,
            [this](bool success, const QString &message)
            {
                if(success){
                    //push_button_->setEnabled(false);
                    //waiting_task_result_ = false;
                    run_log_view_->appendPlainText(QString("%1").arg(message));
                    //clock_timer_->start(5000);
                }
            },
            Qt::QueuedConnection);












        //连接rosmanager发出的无人机状态信号
        connect(ros_manager_, &RosManager::statusUpdated,
            this,
            [this](bool connected, float battery_percent, int flight_mode, bool armed, const QString &task_name)
            {
                updateStatus(connected, battery_percent, flight_mode, armed, task_name);
            },
            Qt::QueuedConnection);

        //连接rosmanager发出的动作状态信号
        connect(ros_manager_, &RosManager::action_statusUpdated,
            this,
            [this](bool task_running, int action_step, int action_num, const QString &action_name)
            {
                action_updateStatus(task_running, action_step, action_num, action_name);
            },
            Qt::QueuedConnection);

        //连接ROS管理器的状态更新信号到一个lambda槽，用于解析状态消息并更新界面上的状态标签
        connect(ros_manager_, &RosManager::barcodeCaptured,
            this,
            [this](const QString &barcode,
                const QByteArray &image_data,
                const QString &image_format,
                const QString &time_text)
            {
                appendBarcodeRecord(barcode, image_data, image_format, time_text);
            },
            Qt::QueuedConnection);

        connect(ros_manager_, &RosManager::visionBarcodeCaptured,
            this,
            [this](const QString &barcode, const QString &time_text)
            {
                appendVisionBarcodeCount(barcode, time_text);
            },
            Qt::QueuedConnection);

        //连接rosmanager发出的无人机位置信号
        connect(ros_manager_, &RosManager::positionUpdated,
            this,
            [this](double x, double y, double z, double qx, double qy, double qz, double qw)
            {
                // 当前仓储项目没有移植 position_view_，所以这里改成直接更新当前场景里的无人机位置。
                scene_data_.drone_state.pose.x = 1 * (x * 100 +150);
                scene_data_.drone_state.pose.y = -1 * (y * 100 -100);
                scene_data_.drone_state.pose.z = z;

                const double siny_cosp = 2.0 * (qw * qz + qx * qy);
                const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
                const double yaw = std::atan2(siny_cosp, cosy_cosp);          // 弧度
                const double yaw_deg = yaw * 180.0 / M_PI;                    // 角度

                scene_data_.drone_state.pose.yaw = yaw_deg;

                altitude_value_label_->setText(QString::number(scene_data_.drone_state.pose.z, 'f', 1) + " m");
                yaw_value_label_->setText(QString::number(scene_data_.drone_state.pose.yaw, 'f', 1) + "°");
                scene_view_->setSceneData(scene_data_);
            },
            Qt::QueuedConnection);

        //连接rosmanager发出的x,y偏移信号
        connect(ros_manager_, &RosManager::deltaUpdated,
            this,
            [this](double dx, double dy, double dyaw, bool valid)
            {
                updateDelta(dx, dy, dyaw, valid);
            },
            Qt::QueuedConnection);

        //链接判断控制程序返回内容
        connect(ros_manager_, &RosManager::pathReadyChanged,
            this,
            [this](bool ready)
            {
                updatePathReadyState(ready);
            },
            Qt::QueuedConnection);

        //连接rosmanager发出的会传路线数据信号
        connect(ros_manager_, &RosManager::returnWorldGroupUpdated,
            this,
            [this](const QVector<WorldCoord> &points)
            {
                updateWorldGroupState(points);
                ros_manager_->startTask();
                //run_log_view_->appendPlainText("初始化成功，准备巡检");
                //clock_timer_->start(5000);
            },
            Qt::QueuedConnection);
    }
}

/*********************ros移植部分***********************/

void MainWindow::triggerMissionUpload(const QString &trigger_source)
{
    if (!ros_manager_)
    {
        run_log_view_->appendPlainText("初始化失败,rosmanager未就绪");
        //clock_timer_->start(5000);
        return;
    }

    if (mission_upload_in_progress_)
    {
        run_log_view_->appendPlainText("无法初始化");
        //clock_timer_->start(5000);
        return;
    }

    drone_msgs::msg::MissionSummary summary;
    summary.takeoff_altitude = 0.0;//起飞高度
    summary.move_altitude = 1.2;//移动高度
    summary.start_altitude = 0.0;//解锁高度
    summary.yaw = 0.0;//偏航角
    summary.tolerance = 0.10;//误差容忍
    summary.takeoff_hover_duration = 0.0;//起飞悬停时长
    summary.landing_hover_duration = 1.0;//降落悬停时长
    summary.move_hover_duration = 3.0;//移动悬停时长
    summary.add_hover_between_takeoff = false;//是否在起飞后添加悬停
    summary.add_hover_between_landing = true;//是否在降落前添加悬停
    summary.add_hover_between_moves = true;//是否在移动之间添加悬停
    summary.use_camera_aim = false;//是否开启相机
    summary.auto_start_mission = false;//是否自动启动任务
    summary.compress_straight_segments = false;//是否压缩直线段
    summary.frame = "world_body";

    summary.cam_tolerance = 10.0;
    summary.camera_aim_pid_p = 0.01;
    summary.camera_aim_pid_i = 0.00;
    summary.camera_aim_pid_d = 0.01;
    summary.camera_aim_target_timeout_s = 1.0;
    summary.camera_aim_stable_cycles = 15;
    summary.camera_aim_max_step = 0.05;
    summary.camera_aim_wait_first_targets_timeout_s = 8.0;
    summary.camera_aim_no_target_confirm_s = 3.0;
    summary.camera_aim_record_result_timeout_s = 10.0;
    summary.camera_aim_scan_point_timeout_s = 30.0;

    if(trigger_source == "waypoint"){
        summary.compress_straight_segments = true;

        if(path_points_.isEmpty()){
            run_log_view_->appendPlainText("航点为空，不允许航点飞行");
            return;
        }
        mission_upload_in_progress_ = true;
        ros_manager_->uploadMissionSummary(path_points_, summary);
    }
    else{
        summary.compress_straight_segments = false;
        QVector<WorldCoord> empty_points;
        mission_upload_in_progress_ = true;
        ros_manager_->uploadMissionSummary(empty_points, summary);
    }
}

void MainWindow::refreshWaypointLog()
{
    waypoint_log_view_->clear();

    if (waypoint_labels_.isEmpty()) {
        run_log_view_->appendPlainText("已清空航点");
        clock_timer_->start(5000);
        return;
    }

    QStringList labels;
    for (const auto &label : waypoint_labels_) {
        labels.append(label);
    }

    waypoint_log_view_->appendPlainText(labels.join("->"));
}

void MainWindow::clearWaypointRequest()
{
    path_points_.clear();
    waypoint_labels_.clear();
    refreshWaypointLog();
}

void MainWindow::setWaypointRequest(int shelf_index, const QString &side, int row, int col)
{
    if (shelf_index < 0) {
        run_log_view_->appendPlainText("添加航点失败：货架索引非法");
        return;
    }

    if (side != "front" && side != "back") {
        run_log_view_->appendPlainText(QString("添加航点失败：side 非法：%1").arg(side));
        return;
    }

    if (row < 0 || row > 3) {
        run_log_view_->appendPlainText(QString("添加航点失败：row 非法：%1").arg(row));
        return;
    }

    if (col < 0 || col > 2) {
        run_log_view_->appendPlainText(QString("添加航点失败：col 非法：%1").arg(col));
        return;
    }

    static const double z_map[4] = {1.50, 1.10, 0.70, 0.30};
    static const double x_front_map[3] = {0.75, 1.25, 1.75};
    static const double x_back_map[3]  = {1.75, 1.25, 0.75};

    const double y = (side == "front")
        ? static_cast<double>(shelf_index) * 1.5
        : static_cast<double>(shelf_index + 1) * 1.5;

    const double x = (side == "front")
        ? x_front_map[col]
        : x_back_map[col];

    const double z = z_map[row];
    const double yaw = (side == "front") ? 1.57 : 4.71;

    path_points_.push_back({x, y, z, yaw});

    //run_log_view_->appendPlainText(
        // QString("已添加航点：x=%5 y=%6 z=%7 yaw=%8")
        //     .arg(x, 0, 'f', 2)
        //     .arg(y, 0, 'f', 2)
        //     .arg(z, 0, 'f', 2)
        //     .arg(yaw, 0, 'f', 2));
    waypoint_labels_.push_back(QString("R%1C%2").arg(row + 1).arg(col + 1));
    refreshWaypointLog();
}

void MainWindow::handleMissionUploadFinished(bool success, const QString &message, const QString &saved_path)
{
    Q_UNUSED(saved_path);

    mission_upload_in_progress_ = false;

    if (success)
    {
        run_log_view_->appendPlainText(QString("%1").arg(message));
        //clock_timer_->start(5000);
        ros_manager_->requestStartOffboard();
    }
    else if (!message.isEmpty())
    {
        run_log_view_->appendPlainText(QString("初始化失败：%1").arg(message));
        //clock_timer_->start(5000);
    }
    else
    {
        run_log_view_->appendPlainText("初始化失败");
        //clock_timer_->start(5000);
    }
}

void MainWindow::updateCommandResult(bool success, const QString &message)
{
    
}











bool MainWindow::updateStatus_connected()  const
{
    return connect_status;
}

void MainWindow::updateStatus(
    bool connected,
    float battery_percent,
    int flight_mode,
    bool armed,
    const QString &task_name)
{
    switch (flight_mode)
    {
        case drone_msgs::msg::DroneStatus::MODE_MANUAL:
            scene_data_.drone_state.flight_mode = "MANUAL";
            break;
        case drone_msgs::msg::DroneStatus::MODE_OFFBOARD:
            scene_data_.drone_state.flight_mode = "OFFBOARD";
            break;
        case drone_msgs::msg::DroneStatus::MODE_STABILIZE:
            scene_data_.drone_state.flight_mode = "STABILIZE";
            break;
        case drone_msgs::msg::DroneStatus::MODE_AUTO:
            scene_data_.drone_state.flight_mode = "AUTO";
            break;
        case drone_msgs::msg::DroneStatus::MODE_LOITER:
            scene_data_.drone_state.flight_mode = "LOITER";
            break;
        case drone_msgs::msg::DroneStatus::MODE_RTL:
            scene_data_.drone_state.flight_mode = "RTL";
            break;
        default:
            scene_data_.drone_state.flight_mode = "UNKNOWN";
            break;
    }

    connect_status = connected;

    // 先把 ROS 返回的基础飞行状态同步到当前场景数据里。
    scene_data_.drone_state.connected = connected;
    scene_data_.drone_state.battery = battery_percent;
    // scene_data_.drone_state.flight_mode = task_name;

    // 当前项目的 TopStatusBar 只直接展示“连接状态”和“任务文本”，
    // 所以这里先把最关键的信息映射过去。
    top_status_bar_->setConnected(connected);
    //top_status_bar_->setTaskText(task_name);

    battery_value_label_->setText(QString::number(scene_data_.drone_state.battery*100, 'f', 1) + "%");
    mode_value_label_->setText(scene_data_.drone_state.flight_mode);

    // flight_mode 目前在当前仓储界面里没有单独的枚举显示控件，
    // 先把它拼进速度/航向旁的任务文本体系里，不额外造新控件。
    Q_UNUSED(flight_mode);

    scene_view_->setSceneData(scene_data_);



    if (armed == true) {
        if (unlock_flag_ == false) {
            unlock_flag_ = true;       // 标记：本轮已经开锁过
            auto_stop_flag_ = false;    // 新一轮允许自动 stop
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
    if(unlock_flag_ == true && auto_stop_flag_ == false && disarm_stable_count_ >= 15){
        auto_stop_flag_ = true;
        unlock_flag_ = false;
        //waiting_push_result_ = false;//重置等待上传结果的标志，允许下一次上传
        if (ros_manager_) {
            ros_manager_->stopTask();
            //run_log_view_->appendPlainText("巡检任务结束");
            top_status_bar_->setTaskText("任务待命");
        }
    }
}

void MainWindow::action_updateStatus(
    bool task_running,
    int action_step,
    int action_num,
    const QString &action_name)
{
    // 当前仓储界面没有原 drone_qt 那套 action 进度控件，
    // 这里先把 action 状态收敛成一条顶部任务文本，便于看到“当前执行到哪一步”。
    const QString task_text = task_running
        ? QString("%1 (%2/%3)").arg(action_name).arg(action_step).arg(action_num)
        : QString("任务待命");
    top_status_bar_->setTaskText(task_text);
}

void MainWindow::appendBarcodeRecord(
    const QString &barcode,
    const QByteArray &image_data,
    const QString &image_format,
    const QString &time_text)
{
    // 机载巡检回填链路：
    // 1. ROS 管理器把无人机识别结果转成 `barcodeCaptured(...)`，最终调用到这里。
    // 2. `barcode` 正常格式是 `PKG-001|SKU-002|A-1-1`，前两段是识别出的类别/包裹编号，第三段是无人机返回的位置码。
    // 3. 这里先拆条码，再同时准备两套定位来源：
    //    - `code_location`：直接按第三段位置码解析格子。
    //    - `pose_location`：按无人机当前位姿估算格子。
    // 4. 规则上位置码优先，位姿只做兜底和冲突对照日志。
    // 5. 最终定位到格子后，把巡检字段和图片都写进主数据，再刷新弹窗显示。

    //先定义空字符串
    auto normalize_field = [](const QString &value) {
        const QString normalized = value.trimmed();
        if (normalized.compare("NA", Qt::CaseInsensitive) == 0 ||
            normalized.compare("N/A", Qt::CaseInsensitive) == 0) {
            return QString();
        }
        return normalized;
    };

    const QStringList parts = barcode.split('|', Qt::KeepEmptyParts);
    const QString package_id =
        (parts.size() >= 1) ? normalize_field(parts[0]) : QString();
    const QString category_id =
        (parts.size() >= 2) ? normalize_field(parts[1]) : QString();
    const QString slot_code =
        (parts.size() >= 3) ? normalize_field(parts[2]) : QString();

    // 全空保护
    if (package_id.isEmpty() && category_id.isEmpty() && slot_code.isEmpty())
    {
        run_log_view_->appendPlainText("收到空条码消息，已忽略");
        return;
    }

    const SlotLocation pose_location = resolveSlotFromPose(scene_data_.drone_state.pose);
    const SlotLocation code_location = resolveSlotFromCode(slot_code);

    SlotLocation target_location;
    if (code_location.valid)
    {
        target_location = code_location;
        if (pose_location.valid &&
            (pose_location.shelf_index != code_location.shelf_index ||
             pose_location.side != code_location.side ||
             pose_location.row != code_location.row ||
             pose_location.col != code_location.col))
        {
            run_log_view_->appendPlainText(
                QString("位置码%1与位姿映射不一致，采用位置码：货架%2 %3 R%4C%5，位姿映射为货架%6 %7 R%8C%9")
                    .arg(slot_code)
                    .arg(code_location.shelf_index + 1)
                    .arg(code_location.side)
                    .arg(code_location.row + 1)
                    .arg(code_location.col + 1)
                    .arg(pose_location.shelf_index + 1)
                    .arg(pose_location.side)
                    .arg(pose_location.row + 1)
                    .arg(pose_location.col + 1));
        }
    }
    else
    {
        target_location = pose_location;
    }

    if (!target_location.valid)
    {
        run_log_view_->appendPlainText(slot_code.isEmpty() ? "收到巡检结果，无法映射" : QString("收到巡检结果，位置码 %1 无法解析且位姿映射失败").arg(slot_code));
        return;
    }

    //根据槽位索引找到对应的货物信息
    ShelfSlotItem *slot = findShelfSlot(target_location.shelf_index, target_location.side, target_location.row, target_location.col);
    if (!slot)
    {
        run_log_view_->appendPlainText("收到巡检结果，目标货架槽位无效");
        return;
    }

    // 先落文本识别结果：严格按视觉真实顺序 PKG|SKU|SHELF
    slot->observed_package_id = package_id;
    slot->observed_category_id = category_id;
    slot->position_package_id = slot_code;
    slot->observed_time_text = time_text;

    //解析完其他数据后独立处理图片
    if (!image_data.isEmpty())
    {
        slot->has_image = true;//标记这个槽位已经有图
        slot->latest_image.image_data = image_data;
        slot->latest_image.image_format = image_format;
        slot->latest_image.barcode = barcode;
        slot->latest_image.time_text = time_text;
    }
    else
    {
        slot->has_image = false;
        slot->latest_image = SlotImageData{};
    }

    shelf_info_dialog_->setShelfPanelData(shelf_panel_data_);
    QString storage_error;
    if (!ShelfPanelStorage::save(shelf_panel_data_, &storage_error))
    {
        run_log_view_->appendPlainText(QString("货架数据保存失败：%1").arg(storage_error));
    }
}

void MainWindow::appendVisionBarcodeCount(
    const QString &barcode,
    const QString &time_text)
{
    Q_UNUSED(barcode);
    Q_UNUSED(time_text);

    // 当前轮次先不使用这条统计信号，只保留接口避免后续接线时再次改动。
}

void MainWindow::applyManualStockIn(int shelf_index, const QString &side, int row, int col,
                                    const QString &category_id, const QString &package_id)
{
    // 地面站手动入库真正落库的地方：
    // 1. Dialog 只负责把“当前选中格 + 扫码结果”发过来。
    // 2. 这里根据格子坐标拿到主数据里的 `ShelfSlotItem`。
    // 3. 只更新手动台账字段 `category_id/package_id`。
    // 4. 更新完成后再统一刷新弹窗，保证主数据和界面显示始终一致。
    ShelfSlotItem *slot = findShelfSlot(shelf_index, side, row, col);
    if (!slot)
    {
        run_log_view_->appendPlainText("手动入库失败：目标槽位无效");
        return;
    }

    slot->category_id = category_id;
    slot->package_id = package_id;
    shelf_info_dialog_->setShelfPanelData(shelf_panel_data_);
    QString storage_error;
    if (!ShelfPanelStorage::save(shelf_panel_data_, &storage_error))
    {
        run_log_view_->appendPlainText(QString("货架数据保存失败：%1").arg(storage_error));
    }
}

void MainWindow::applyManualStockOut(int shelf_index, const QString &side, int row, int col, const QString &category_id, const QString &package_id)
{
    // 地面站手动出库真正清空数据的地方：
    // 1. 先按当前选中格定位到主数据里的槽位。
    // 2. 再把手动台账、巡检结果、位置码、时间、图片统一清空。
    // 3. 这样做的目的，是让“出库”后的格子完整回到空位状态，避免残留旧巡检痕迹误导界面。
    ShelfSlotItem *slot = findShelfSlot(shelf_index, side, row, col);
    if (!slot)
    {
        run_log_view_->appendPlainText("手动出库失败：目标槽位无效");
        return;
    }

    const QString scanned_category_id = category_id.trimmed();
    const QString scanned_package_id = package_id.trimmed();

    if (slot->category_id.isEmpty() || slot->package_id.isEmpty())
    {
        run_log_view_->appendPlainText("台帐数据缺失");
        return;
    }
    if (slot->category_id != scanned_category_id ||
        slot->package_id != scanned_package_id)
    {
        run_log_view_->appendPlainText("台帐数据不对应");
        return;
    }

    slot->category_id.clear();
    slot->package_id.clear();
    slot->observed_category_id.clear();
    slot->observed_package_id.clear();
    slot->position_package_id.clear();
    slot->observed_time_text.clear();
    slot->has_image = false;
    slot->latest_image = SlotImageData{};
    shelf_info_dialog_->setShelfPanelData(shelf_panel_data_);
    QString storage_error;
    if (!ShelfPanelStorage::save(shelf_panel_data_, &storage_error))
    {
        run_log_view_->appendPlainText(QString("货架数据保存失败：%1").arg(storage_error));
    }

    run_log_view_->appendPlainText(
    QString("手动出库成功：货架%1 %2 R%3C%4")
        .arg(shelf_index + 1)
        .arg(side)
        .arg(row + 1)
        .arg(col + 1));
}

SlotLocation MainWindow::resolveSlotFromCode(const QString &slot_code) const
{
    SlotLocation location;
    static const QRegularExpression pattern("^([A-Z])-(\\d+)-(\\d+)$");
    const QRegularExpressionMatch match = pattern.match(slot_code.trimmed());
    if (!match.hasMatch())
    {
        return location;
    }

    const QString shelf_code = match.captured(1);
    const int encoded_row = match.captured(2).toInt();
    const int encoded_col = match.captured(3).toInt();

    if (encoded_row < 0 || encoded_row > 3 || encoded_col < 0 || encoded_col > 2)
    {
        return location;
    }

    const int shelf_bucket = shelf_code.at(0).unicode() - QChar('A').unicode();
    if (shelf_bucket < 0)
    {
        return location;
    }

    location.shelf_index = shelf_bucket / 2;
    location.side = (shelf_bucket % 2 == 0) ? "front" : "back";
    location.row = encoded_row;
    location.col = encoded_col;
    location.valid = location.shelf_index >= 0 && location.shelf_index < shelf_panel_data_.size();
    return location;
}

ShelfSlotItem *MainWindow::findShelfSlot(int shelf_index, const QString &side, int row, int col)
{
    //根据货架索引、前后面、行列号，找到真正的 `ShelfSlotItem*`
    if (shelf_index < 0 || shelf_index >= shelf_panel_data_.size())
    {
        return nullptr;
    }

    if (row < 0 || row >= 4 || col < 0 || col >= 3)
    {
        return nullptr;
    }

    // 决定当前到底是在访问 front 还是 back
    QVector<ShelfSlotItem> *slot_list = nullptr;
    if (side == "front")
    {
        slot_list = &shelf_panel_data_[shelf_index].front_slots;
    }
    else
    {
        slot_list = &shelf_panel_data_[shelf_index].back_slots;
    }

    const int index = row * 3 + col;//行列转一维下标
    if (!slot_list || index < 0 || index >= slot_list->size())
    {
        return nullptr;
    }

    return &(*slot_list)[index];
}

SlotLocation MainWindow::resolveSlotFromPose(const Pose3D &pose) const
{
    SlotLocation location;//准备一个默认无效结果

    if (shelf_panel_data_.size() < 2)
    {
        return location;
    }

    if (((pose.x >= 110.0) && (pose.x <= 140.0) && (pose.yaw >= 80.0) && (pose.yaw <= 100.0)) ||
        ((pose.x >= -10.0) && (pose.x <= 10.0) && (pose.yaw >= -100.0) && (pose.yaw <= -80.0)))//第 1 个货架判定标准
    {
        location.shelf_index = 0;
    }
    else if (((pose.x >= -10.0) && (pose.x <= 10.0) && (pose.yaw >= 80.0) && (pose.yaw <= 100.0)) ||
        ((pose.x >= -140.0) && (pose.x <= -110.0) && (pose.yaw >= -100.0) && (pose.yaw <= -80.0)))//第 2 个货架判定标准
    {
        location.shelf_index = 1;
    }
    else
    {
        return location;
    }

    const double normalized_yaw = std::fmod(std::fmod(pose.yaw, 360.0) + 360.0, 360.0);//归一化 yaw
    location.side = (std::abs(normalized_yaw - 90.0) <= std::abs(normalized_yaw - 180.0)) ? "front" : "back";//比较离 90° 和 180° 哪个更近

    const double clamped_y = qBound(-100.0, pose.y, 50.0);//先把 `y` 限制到 `[-100, 50]`
    const double clamped_z = qBound(0.0, pose.z, 160.0);//先把 `z` 限制到 `[0, 160]`

    const double normalized_col = (clamped_y + 100.0) / 150.0;//把 `y` 从 `[-100, 50]` 映射到 `[0, 1]`
    const double normalized_row = clamped_z / 160.0;//把 `z` 从 `[0, 160]` 映射到 `[0, 1]`

    //判断行列
    location.col = qBound(0, static_cast<int>(normalized_col * 3.0), 2);
    location.row = qBound(0, static_cast<int>(normalized_row * 4.0), 3);
    location.valid = true;//标志成功定位
    return location;
}

void MainWindow::showShelfSlotImage(int shelf_index, const QString &side, int row, int col)
{
    ShelfSlotItem *slot = findShelfSlot(shelf_index, side, row, col);//找到对应的槽位
    //检查这个槽位是否真的有图
    if (!slot || !slot->has_image || slot->latest_image.image_data.isEmpty())
    {
        // run_log_view_->appendPlainText("当前槽位暂无图片");
        // clock_timer_->start(5000);
        return;
    }

    //处理照片
    QImage image;
    const QByteArray format_bytes = slot->latest_image.image_format.toUtf8();
    const char *format_ptr = format_bytes.isEmpty() ? nullptr : format_bytes.constData();
    if (!image.loadFromData(slot->latest_image.image_data, format_ptr))
    {
        // run_log_view_->appendPlainText("当前槽位图片解析失败");
        // clock_timer_->start(5000);
        return;
    }

    if (!image_preview_dialog_)
    {
        image_preview_dialog_ = new QDialog(this);//图片预览窗口本体
        image_preview_dialog_->resize(600, 400);

        auto *layout = new QVBoxLayout(image_preview_dialog_);//垂直布局，把里面的滚动区域铺进去
        auto *scroll_area = new QScrollArea(image_preview_dialog_);//滚动区域，避免大图把窗口撑爆
        image_preview_label_ = new QLabel(scroll_area);//真正承载图片的控件
        image_preview_label_->setAlignment(Qt::AlignCenter);
        scroll_area->setWidget(image_preview_label_);
        scroll_area->setWidgetResizable(true);
        layout->addWidget(scroll_area);
    }

    //写弹窗标题
    image_preview_dialog_->setWindowTitle(
        QString("货架%1 %2 R%3C%4 | %5 | %6")
            .arg(shelf_index + 1)
            .arg(side)
            .arg(row + 1)
            .arg(col + 1)
            .arg(slot->latest_image.barcode)
            .arg(slot->latest_image.time_text));

    //显示图片
    image_preview_label_->setPixmap(QPixmap::fromImage(image));
    image_preview_label_->adjustSize();
    image_preview_dialog_->show();
    image_preview_dialog_->raise();
    image_preview_dialog_->activateWindow();
}

void MainWindow::updateDelta(double dx, double dy, double dyaw, bool valid)
{
    if (!top_status_bar_)
    {
        return;
    }

    top_status_bar_->updateDelta(dx, dy, dyaw, valid);
}

void MainWindow::updatePathReadyState(bool ready)
{
    
}

void MainWindow::updateWorldGroupState(const QVector<WorldCoord> &points)
{
    
}

/******************************************************/

void MainWindow::applyWindowStyle()
{
    setStyleSheet(
        "QMainWindow, QWidget {"
        "background: #0c1018;"
        "color: #d7e3f4;"
        "}"
        "QSlider::groove:horizontal {"//滑块槽道样式
        "height: 30px;"//槽道高度
        "background: rgba(70, 90, 120, 120);"//槽道背景颜色
        "border-radius: 8px;"//槽道圆角
        "}"
        "QSlider::handle:horizontal {"//滑块样式
        "width: 40px;"//滑块宽度
        "height: 30px;"//滑块高度
        "background: #00c8ff;" //滑块颜色
        "margin: -6px 0;"//滑块垂直居中
        "border-radius: 8px;"//滑块圆角
        "}"
    );

    log_panel_->setStyleSheet(
        "background: rgba(18, 24, 34, 0);"//透明深色背景
        "border: none;"//标签无边框
        "border-radius: 10px;"

        "border: none;"//无边框
        "padding: 6px 10px;"//内边距
        "}"
    );

    logwaypoint_panel_->setStyleSheet(
        "background: rgba(18, 24, 34, 0);"//透明深色背景
        "font-size: 16px;"
        "border: none;"//标签无边框
        "border-radius: 10px;"

        "border: none;"//无边框
        "padding: 6px 10px;"//内边距
        "}"
    );

    ai_log_panel_->setStyleSheet(
        // "#aiLogPanel {"
        // "background: rgba(18, 24, 34, 150);"
        // "border: 1px solid rgba(90, 130, 180, 100);"
        // "border-radius: 10px;"
        // "}"
        // "#aiLogTitle {"
        // "background: transparent;"
        // "border: none;"
        // "font-size: 16px;"
        // "font-weight: 600;"
        // "color: #8fe7ff;"
        // "}"
        // "QPlainTextEdit {"
        // "background: rgba(10, 14, 22, 170);"
        // "border: none;"
        // "color: #d7e3f4;"
        // "font-size: 14px;"
        // "padding: 6px;"
        // "}"

        "background: rgba(18, 24, 34, 0);"//透明深色背景
        "font-size: 16px;"
        "border: none;"//标签无边框
        "border-radius: 10px;"

        "border: none;"//无边框
        "padding: 6px 10px;"//内边距
        "}"
    );

    attitude_panel_->setStyleSheet(
        "background: rgba(18, 24, 34, 100);"//半透明深色背景
        "border: 1px solid rgba(90, 130, 180, 100);"//边框颜色和透明度
        "border-radius: 10px;"

        "border: none;"//无边框
        "padding: 6px 10px;"//内边距
        "}"
    );

    view_mode_widget_->setStyleSheet(
        "background: rgba(18, 24, 34, 170);"
        //"border: 1px solid rgba(90, 130, 180, 120);"
        "border: none;"
        "border-radius: 10px;"
    );

    view_mode_left_label_->setStyleSheet(
        "border: none;"//标签无边框
        "font-size: 18px;"
    );

    view_mode_right_label_->setStyleSheet(
        "border: none;"
        "font-size: 18px;"
    );
}

void MainWindow::setupDemoData()
{

    /*********************无人机位置修改***********************/

    WarehouseSceneData data;

    data.drone_state.flight_mode = "OFFBOARD";

    data.drone_state.pose.z = 0.0;
    data.drone_state.speed = 4.2;
    data.drone_state.battery = 87.0;

    scene_data_ = data;//先把演示数据复制到成员里，后面 ROS 实时更新时会直接改这份成员数据

    /*********************************************************/

    /**********************货架尺寸更改************************/

    //QRectF(-200, -120, 80, 160)：左上角x，左上角y，底面宽，底面长
    //height是货架的高度，name是货架的名字
    ShelfBlock shelf1;
    shelf1.base_rect = QRectF(-90, -100, 30, 150);
    shelf1.height = 160;
    shelf1.name = "A01";
    shelf1.color = ColorPalette::withAlpha(ColorPalette::BlueGrayDark, 180);
    data.shelves.push_back(shelf1);

    ShelfBlock shelf2;
    shelf2.base_rect = QRectF(60, -100, 30, 150);
    shelf2.height = 160;
    shelf2.name = "A02";
    shelf2.color = ColorPalette::withAlpha(ColorPalette::BlueGrayDark, 180);
    data.shelves.push_back(shelf2);

    /*********************************************************/

    /**********************轨迹位置更改************************/

    // data.trajectory.push_back({140, 125, 0});

    /*********************************************************/

    scene_view_->setSceneData(data);
    scene_data_ = data;//把最终演示场景保存成成员，后面 ROS 位置/状态更新时直接在这份数据上改

    top_status_bar_->setConnected(data.drone_state.connected);
    top_status_bar_->setTaskText("任务待命");


    /**********************货架信息更改************************/

    // 这里开始统一准备“货架弹窗”需要的全部演示数据。
    // 现在不再像之前那样拆成很多 QStringList，而是一个货架对应一个 ShelfPanelData。
    // 这样做的好处是：
    // 1. 所有假数据都集中在 MainWindow 里，后面查起来不容易乱。
    // 2. 一个货架的名称、前面点位、后面点位都装在一起，结构更清楚。
    // 3. 后面如果你要把假数据换成 ROS / 数据库 / 接口数据，也更容易整体替换。

    // -------------------- 货架1数据 --------------------
    shelf_panel_data_.clear();

    ShelfPanelData shelf1_panel;
    shelf1_panel.display_name = "货架1";//顶部按钮和弹窗标题里显示的名称
    shelf1_panel.button_status_color = "#00d48a";//顶部按钮前面的状态灯颜色，这里交给主窗口统一决定
    shelf1_panel.front_slots.resize(12);//前面固定16个点位，对应4x4网格
    shelf1_panel.back_slots.resize(12);//后面固定16个点位，对应4x4网格

    // -------------------- 货架2数据 --------------------
    ShelfPanelData shelf2_panel;
    shelf2_panel.display_name = "货架2";//第二个顶部按钮和标题显示名称
    shelf2_panel.button_status_color = "#f0b400";//第二个顶部按钮前面的状态灯颜色
    shelf2_panel.front_slots.resize(12);
    shelf2_panel.back_slots.resize(12);

    // 把两个货架的数据一起压进总列表。
    shelf_panel_data_.push_back(shelf1_panel);
    shelf_panel_data_.push_back(shelf2_panel);

    QString storage_error;
    if (ShelfPanelStorage::load(shelf_panel_data_, &storage_error))
    {
        run_log_view_->appendPlainText(
            QString("已加载货架持久化数据：%1，文件：%2")
                .arg(shelf_panel_data_.size())
                .arg(ShelfPanelStorage::defaultFilePath()));
    }
    else
    {
        run_log_view_->appendPlainText(
            QString("未加载历史货架数据：%1，文件：%2")
                .arg(storage_error)
                .arg(ShelfPanelStorage::defaultFilePath()));
    }

    // 最后把整包货架弹窗数据一次性传给 ShelfInfoDialog。
    // 从这里开始，弹窗只负责显示，不再自己写任何假数据。
    shelf_info_dialog_->setShelfPanelData(shelf_panel_data_);

    /*********************************************************/

}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
    updateOverlayGeometry();//窗口大小改变时调整悬浮控件的位置和大小
}

void MainWindow::updateOverlayGeometry()
{
    if (!central_container_)
    {
        return;
    }

    const QRect area = central_container_->rect();

    const QPoint top_left = top_status_bar_->shelfButtonBottomLeftGlobal();

    scene_view_->setGeometry(area);//主场景视图占满整个主容器
    //左边距；上边距；宽度；高度
    top_status_bar_->setGeometry(20, 16, area.width() - 40, 52);
    log_panel_->setGeometry(5, top_left.y()+10, 310, 200);
    logwaypoint_panel_->setGeometry(250, area.height() - 90, 600, 200);
    ai_log_panel_->setGeometry(area.width() - 320, 260, 330, 280);
    attitude_panel_->setGeometry(area.width() - 220, 84, 220, 160);
    view_mode_widget_->setGeometry(100, area.height() - 70, 160, 40);
    view_Perspective_widget_->setGeometry(area.width() - 220, area.height() - 70, 160, 40);
    view_2D_widget_->setGeometry(area.width() - 220, area.height() - 70, 160, 40);
}






























QVector<SlotAnalysisInput> MainWindow::collectSlotAnalysisInputs() const
{
    QVector<SlotAnalysisInput> inputs;

    for (int shelf_index = 0; shelf_index < shelf_panel_data_.size(); ++shelf_index)
    {
        const ShelfPanelData &shelf = shelf_panel_data_[shelf_index];

        auto append_slot_items = [&](const QVector<ShelfSlotItem> &slot_items, const QString &side) {
            for (int row = 0; row < 4; ++row)
            {
                for (int col = 0; col < 3; ++col)
                {
                    const int index = row * 3 + col;
                    if (index < 0 || index >= slot_items.size())
                    {
                        continue;
                    }

                    const ShelfSlotItem &slot = slot_items[index];

                    SlotAnalysisInput input;
                    input.shelf_name = shelf.display_name;
                    input.shelf_index = shelf_index;
                    input.side = side;
                    input.row = row;
                    input.col = col;

                    input.manual_category_id = slot.category_id;
                    input.manual_package_id = slot.package_id;

                    input.observed_category_id = slot.observed_category_id;
                    input.observed_package_id = slot.observed_package_id;
                    input.observed_slot_code = slot.position_package_id;
                    input.observed_time_text = slot.observed_time_text;
                    input.has_image = slot.has_image;

                    inputs.push_back(input);
                }
            }
        };

        append_slot_items(shelf.front_slots, "front");
        append_slot_items(shelf.back_slots, "back");
    }

    return inputs;
}

QVector<SlotRuleAnalysis> MainWindow::buildRuleAnalysisResults() const
{
    const QVector<SlotAnalysisInput> inputs = collectSlotAnalysisInputs();
    return AiDiffAnalyzer::analyzeAll(inputs);
}

QString MainWindow::buildRuleAnalysisReport(const QVector<SlotRuleAnalysis> &results) const
{
    QStringList lines;
    lines << "=== AI差异巡检助手（规则预分析） ===";

    int high_count = 0;
    int medium_count = 0;
    int low_count = 0;

    for (const SlotRuleAnalysis &result : results)
    {
        if (result.priority >= 80)
        {
            ++high_count;
        }
        else if (result.priority >= 50)
        {
            ++medium_count;
        }
        else
        {
            ++low_count;
        }
    }

    lines << QString("高优先级：%1 项").arg(high_count);
    lines << QString("中优先级：%1 项").arg(medium_count);
    lines << QString("低优先级：%1 项").arg(low_count);
    lines << "";

    for (const SlotRuleAnalysis &result : results)
    {
        if (result.status == SlotDiffStatus::Matched || result.status == SlotDiffStatus::Empty)
        {
            continue;
        }

        lines << result.summary;
        lines << QString("原因：%1").arg(result.reason);
        lines << QString("优先级：%1").arg(result.priority);
        lines << QString("建议复查：%1").arg(result.should_revisit ? "是" : "否");
        lines << "";
    }

    return lines.join('\n');
}

void MainWindow::runAiDiffAnalysis()
{
    const QVector<SlotRuleAnalysis> results = buildRuleAnalysisResults();
    const QString report = buildRuleAnalysisReport(results);
    if (ai_log_view_)
    {
        ai_log_view_->clear();
        ai_log_view_->appendPlainText(report);
    }
}

QString MainWindow::buildAiPrompt(const SlotRuleAnalysis &result) const
{
    const SlotAnalysisInput &input = result.input;

    QStringList lines;
    lines << "你是仓储槽位图片巡检助手。";
    lines << "请直接查看图片，并结合槽位上下文判断：";
    lines << "1. 是否能确认图中有包裹；";
    lines << "2. 包裹主体是否清晰可见；";
    lines << "3. 标签/条码/身份信息是否可辨认；";
    lines << "4. 当前图片证据是否足以支持判断；";
    lines << "5. 如无法确认，请明确写无法确认。";
    lines << "";
    lines << "请只输出 JSON 数组。";
    lines << "每项字段固定为：slot,severity,summary,reason,action,should_revisit。";
    lines << "severity 只能是 low / medium / high。";
    lines << "should_revisit 只能是 true / false。";
    lines << "";
    lines << "约束：";
    lines << "- 优先依据图片本身判断，不要只复述上下文字段。";
    lines << "- 不要臆造未看见的内容。";
    lines << "- 不要输出 markdown。";
    lines << "- 不要长篇解释。";
    lines << "- summary：一句话结论，尽量简短。";
    lines << "- reason：只写最关键事实。";
    lines << "- action：只给一个动作。";
    lines << "";
    lines << "槽位上下文：";
    lines << QString("- slot=%1 %2 R%3C%4")
                .arg(input.shelf_name)
                .arg(input.side)
                .arg(input.row + 1)
                .arg(input.col + 1);
    lines << QString("- manual_package=%1").arg(input.manual_package_id);
    lines << QString("- manual_category=%1").arg(input.manual_category_id);
    lines << QString("- observed_package=%1").arg(input.observed_package_id);
    lines << QString("- observed_category=%1").arg(input.observed_category_id);
    lines << QString("- observed_slot=%1").arg(input.observed_slot_code);
    lines << QString("- observed_time=%1").arg(input.observed_time_text);
    lines << QString("- local_summary=%1").arg(result.summary);
    lines << QString("- local_reason=%1").arg(result.reason);

    return lines.join('\n');
}

void MainWindow::runClaudeApiDiffAnalysis()
{
    const QVector<SlotRuleAnalysis> results = buildRuleAnalysisResults();

    if (!ai_log_view_)
    {
        return;
    }

    ai_log_view_->clear();
    // ai_log_view_->appendPlainText("AI分析开始...");

    auto slot_label = [](const SlotRuleAnalysis &result) {
        const QString shelf_short = result.input.shelf_name.endsWith("A") ? "A" :
                                    (result.input.shelf_name.endsWith("B") ? "B" : result.input.shelf_name);
        const QString side_short = (result.input.side == "front") ? "F" :
                                   ((result.input.side == "back") ? "B" : result.input.side);
        return QString("%1 %2 R%3C%4")
            .arg(shelf_short)
            .arg(side_short)
            .arg(result.input.row + 1)
            .arg(result.input.col + 1);
    };

    auto status_text = [](SlotDiffStatus status) {
        switch (status)
        {
        case SlotDiffStatus::Mismatch:
            return QString("台账与巡检不一致");
        case SlotDiffStatus::PartialObserved:
            return QString("巡检结果不完整");
        case SlotDiffStatus::ObservedOnly:
            return QString("巡检识别到货物，但台账为空");
        case SlotDiffStatus::ManualOnly:
            return QString("存在台账，但巡检为空");
        case SlotDiffStatus::PositionOnly:
            return QString("仅识别到位置，没有识别到货物身份");
        case SlotDiffStatus::ObservedWithoutImage:
            return QString("巡检有结果，但缺少图片证据");
        case SlotDiffStatus::Matched:
            return QString("台账与巡检一致");
        case SlotDiffStatus::Empty:
            return QString("空槽位");
        }
        return QString("未知状态");
    };

    auto field_text = [](const QString &value) {
        return value.isEmpty() ? QString("空") : value;
    };

    auto risk_level_text = [](int priority) {
        if (priority >= 80)
        {
            return QString("高");
        }
        if (priority >= 60)
        {
            return QString("中");
        }
        return QString("低");
    };

    auto short_reason_text = [](SlotDiffStatus status) {
        switch (status)
        {
        case SlotDiffStatus::Mismatch:
            return QString("台账与巡检货物信息冲突");
        case SlotDiffStatus::PartialObserved:
            return QString("包裹或类别信息识别不完整");
        case SlotDiffStatus::ObservedOnly:
            return QString("识别到货物但无台账记录");
        case SlotDiffStatus::ManualOnly:
            return QString("台账货物未被巡检识别");
        case SlotDiffStatus::PositionOnly:
            return QString("仅识别到位置，货物信息缺失");
        case SlotDiffStatus::ObservedWithoutImage:
            return QString("识别结果缺少图片证据");
        case SlotDiffStatus::Matched:
        case SlotDiffStatus::Empty:
            return QString("无异常");
        }
        return QString("待复查");
    };

    auto action_text = [](SlotDiffStatus status) {
        switch (status)
        {
        case SlotDiffStatus::Mismatch:
            return QString("核对实物并更新台账");
        case SlotDiffStatus::PartialObserved:
        case SlotDiffStatus::PositionOnly:
            return QString("补拍并重新识别");
        case SlotDiffStatus::ObservedOnly:
            return QString("核对实物并补登记");
        case SlotDiffStatus::ManualOnly:
            return QString("复查货物是否在位");
        case SlotDiffStatus::ObservedWithoutImage:
            return QString("补拍并留存证据");
        case SlotDiffStatus::Matched:
        case SlotDiffStatus::Empty:
            return QString("无需处理");
        }
        return QString("人工复查");
    };

    QVector<const SlotRuleAnalysis *> abnormal_results;

    int abnormal_count = 0;
    int high_count = 0;
    for (const SlotRuleAnalysis &result : results)
    {
        if (result.status == SlotDiffStatus::Matched || result.status == SlotDiffStatus::Empty)
        {
            continue;
        }

        ++abnormal_count;
        if (result.priority >= 80)
        {
            ++high_count;
        }
        abnormal_results.push_back(&result);
    }

    if (abnormal_count == 0)
    {
        // ai_log_view_->appendPlainText(QString("仓库状态：正常，异常0/%1。").arg(results.size()));
        return;
    }

    std::sort(abnormal_results.begin(), abnormal_results.end(),
        [](const SlotRuleAnalysis *left, const SlotRuleAnalysis *right) {
            return left->priority > right->priority;
        });

    QStringList ai_input_lines;
    QStringList problem_lines;
    QStringList fallback_problem_lines;
    QStringList fallback_action_lines;
    QStringList expected_slots;
    for (int index = 0; index < abnormal_results.size(); ++index)
    {
        const SlotRuleAnalysis &result = *abnormal_results.at(index);
        const QString slot = slot_label(result);

        if (index >= 3)
        {
            continue;
        }

        ai_input_lines << QString("槽位：%1").arg(slot);
        ai_input_lines << QString("- 异常类型：%1").arg(status_text(result.status));
        ai_input_lines << QString("- 规则结论：%1").arg(result.summary);
        ai_input_lines << QString("- 规则依据：%1").arg(result.reason);
        ai_input_lines << QString("- 风险等级：%1").arg(risk_level_text(result.priority));
        ai_input_lines << QString("- 台账类别：%1").arg(field_text(result.input.manual_category_id));
        ai_input_lines << QString("- 台账包裹：%1").arg(field_text(result.input.manual_package_id));
        ai_input_lines << QString("- 巡检类别：%1").arg(field_text(result.input.observed_category_id));
        ai_input_lines << QString("- 巡检包裹：%1").arg(field_text(result.input.observed_package_id));
        ai_input_lines << QString("- 巡检位置码：%1").arg(field_text(result.input.observed_slot_code));
        ai_input_lines << QString("- 巡检时间：%1").arg(field_text(result.input.observed_time_text));
        ai_input_lines << QString("- 图片证据：%1").arg(result.input.has_image ? "有" : "无");
        ai_input_lines << QString("- 规则建议复查：%1").arg(result.should_revisit ? "是" : "否");
        ai_input_lines << "";

        if (index < 3)
        {
            problem_lines << QString("%1 %2").arg(slot).arg(status_text(result.status));
            fallback_problem_lines << QString("%1 %2").arg(slot).arg(short_reason_text(result.status));
            fallback_action_lines << QString("%1 %2").arg(slot).arg(action_text(result.status));
            expected_slots << slot;
        }
    }

    QStringList lines;
    // lines << QString("仓库状态：异常%1/%2，高优先级%3。")
    //              .arg(abnormal_count)
    //              .arg(results.size())
    //              .arg(high_count);
    // lines << QString("问题槽位：%1").arg(problem_lines.join("；"));
    lines << "AI正在生成简短建议...";
    ai_log_view_->appendPlainText(lines.join('\n'));

    QStringList prompt_lines;
    prompt_lines << "你是仓储巡检值班助手。";
    prompt_lines << "只能依据以下事实回答，不能增加槽位、数量或现场信息。";
    prompt_lines << "必须只输出3行，不要markdown，不要解释规则。";
    prompt_lines << QString("第一行必须以“仓库状态：”开头，并使用：异常%1处，高优先级%2处。")
                        .arg(abnormal_count)
                        .arg(high_count);
    prompt_lines << "第二行必须以“问题槽位：”开头，只能使用下列允许槽位，说明台账和巡检的实际差异。";
    prompt_lines << "第三行必须以“下一步：”开头，针对最高风险槽位给出具体复查动作。";
    prompt_lines << "禁止输出槽位1、槽位2、原因、动作1等占位词；不得重复三行内容。";
    prompt_lines << "允许输出的异常槽位：";
    for (const QString &slot : expected_slots)
    {
        prompt_lines << QString("- %1").arg(slot);
    }
    prompt_lines << "";
    prompt_lines << "槽位事实：";
    prompt_lines << ai_input_lines;

    const QString fallback_text = QStringList{
        QString("仓库状态：异常%1处，高优先级%2处。").arg(abnormal_count).arg(high_count),
        QString("问题槽位：%1").arg(fallback_problem_lines.join("；")),
        QString("下一步：%1").arg(fallback_action_lines.join("；"))
    }.join('\n');

    const QString temp_dir = QDir::tempPath();
    const QString prompt_path = temp_dir + "/warehouse_ai_prompt.txt";
    const QString image_meta_path = temp_dir + "/warehouse_ai_image_meta.json";
    const QString output_path = temp_dir + "/warehouse_ai_output.txt";
    const QString script_path = QCoreApplication::applicationDirPath() + "/warehouse_ai_runner.py";

    QFile prompt_file(prompt_path);
    if (!prompt_file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        ai_log_view_->appendPlainText("AI总结失败：无法写入 prompt 文件");
        return;
    }
    prompt_file.write(prompt_lines.join('\n').toUtf8());
    prompt_file.close();

    QJsonObject image_meta;
    image_meta["mode"] = "text_summary";

    QFile image_meta_file(image_meta_path);
    if (!image_meta_file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        ai_log_view_->appendPlainText("AI总结失败：无法写入输入元数据文件");
        return;
    }
    image_meta_file.write(QJsonDocument(image_meta).toJson(QJsonDocument::Compact));
    image_meta_file.close();

    auto *process = new QProcess(this);
    connect(process,
        QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
        this,
        [this, process, output_path, fallback_text, expected_slots, abnormal_count, high_count](int exit_code, QProcess::ExitStatus exit_status) {
        Q_UNUSED(exit_status);

        if (exit_code != 0)
        {
            ai_log_view_->appendPlainText("AI总结失败：调用脚本退出异常，已显示规则保底结果。");
            ai_log_view_->appendPlainText(QString::fromUtf8(process->readAllStandardError()));
            ai_log_view_->appendPlainText(fallback_text);
            process->deleteLater();
            return;
        }

        QFile output_file(output_path);
        if (!output_file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            ai_log_view_->appendPlainText("AI总结失败：无法读取输出结果，已显示规则保底结果。");
            ai_log_view_->appendPlainText(fallback_text);
            process->deleteLater();
            return;
        }

        const QString output_text = QString::fromUtf8(output_file.readAll()).trimmed();
        output_file.close();

        const QStringList output_lines = output_text.split('\n', Qt::SkipEmptyParts);
        bool output_is_valid = output_lines.size() == 3
            && output_lines.at(0).trimmed().startsWith("仓库状态：")
            && output_lines.at(1).trimmed().startsWith("问题槽位：")
            && output_lines.at(2).trimmed().startsWith("下一步：");

        const QRegularExpression placeholder_pattern("槽位\\s*[0-9]+|动作\\s*[0-9]+");
        if (output_is_valid && placeholder_pattern.match(output_text).hasMatch())
        {
            output_is_valid = false;
        }

        auto normalize_text = [](QString text) {
            text.remove(QRegularExpression("\\s+"));
            return text;
        };

        const QString normalized_output = normalize_text(output_text);

        bool has_real_slot = false;
        for (const QString &slot : expected_slots)
        {
            if (normalized_output.contains(normalize_text(slot)))
            {
                has_real_slot = true;
                break;
            }
        }

        const QString normalized_status = normalize_text(output_lines.at(0));
        const bool has_correct_counts =
            normalized_status.contains(QString("异常%1").arg(abnormal_count))
            && normalized_status.contains(QString("高优先级%1").arg(high_count));

        bool has_unknown_slot = false;
        const QRegularExpression slot_pattern("R\\d+C\\d+");
        QRegularExpressionMatchIterator matches =
            slot_pattern.globalMatch(normalized_output);

        while (matches.hasNext())
        {
            const QString detected_code = matches.next().captured(0);
            bool is_allowed = false;

            for (const QString &slot : expected_slots)
            {
                if (normalize_text(slot).contains(detected_code))
                {
                    is_allowed = true;
                    break;
                }
            }

            if (!is_allowed)
            {
                has_unknown_slot = true;
                break;
            }
        }

        output_is_valid = output_is_valid
            && has_real_slot
            && has_correct_counts
            && !has_unknown_slot;

        ai_log_view_->appendPlainText("");
        if (output_is_valid)
        {
            ai_log_view_->appendPlainText(output_text);
        }
        else
        {
            ai_log_view_->appendPlainText("AI输出无效，已显示规则保底结果。");
            ai_log_view_->appendPlainText("AI原始输出：");
            ai_log_view_->appendPlainText(
                output_text.isEmpty() ? "（空输出）" : output_text);
            ai_log_view_->appendPlainText("规则保底结果：");
            ai_log_view_->appendPlainText(fallback_text);
        }

        process->deleteLater();
    });

    QStringList args;
    args << script_path << prompt_path << image_meta_path << output_path;
    process->start("python3", args);
}




#pragma once

#include <QFrame>
#include <QString>

class QLabel;
class QPushButton;
class RosManager;

class QTimer;

class TopStatusBar : public QFrame
{
    Q_OBJECT

public:
    explicit TopStatusBar(QWidget *parent = nullptr);

    void setConnectionText(const QString &text);//设置连接状态文本
    void setTaskText(const QString &text);//设置任务状态文本
    void setShelfText(const QString &text);//设置货架信息文本
    void setTimeText(const QString &text);//设置时间文本

    void setConnected(bool connected);

    /*********************ros移植部分***********************/
    void setTriggerTime(const QString &text);//设置时间触发的目标时刻，格式先按 HH:mm:ss 使用
    void setTimeTriggerEnabled(bool enabled);//设置是否启用到点触发上传
    void updateDelta(double dx, double dy, double dyaw, bool valid);//更新无人机位置增量
    /******************************************************/

    QPoint shelfButtonBottomLeftGlobal() const;//返回货架信息按钮左下角的全局坐标

signals:
    void titleClicked();//标题按钮被点击
    void connectionClicked();//连接状态按钮被点击
    void taskClicked();//任务状态按钮被点击
    void shelfButtonClicked();//货架按钮被点击
    void aiAnalyzeButtonClicked();//分析按钮被点击
    void executeButtonClicked();//执行按钮被点击
    void waypointButtonClicked();//航点飞行按钮被点击
    void scheduledcheckbuttonnClicked(QString mission_trigger_time);//巡检按钮被点击

    void exitRequested();//退出信号

    /*********************ros移植部分***********************/
    void triggerTimeReached(const QString &time_text);//顶部时钟到达目标时刻时发出信号，交给主窗口决定是否执行上传
    /******************************************************/

private:
    QPushButton *title_button_ = nullptr;//标题按钮
    QPushButton *connection_button_ = nullptr;//连接状态按钮
    QPushButton *shelf_button_ = nullptr;//货架状态按钮
    QPushButton *task_button_ = nullptr;//任务状态按钮
    QLabel *dx_indicator_label_{nullptr};//dx指示灯标签
    QLabel *dy_indicator_label_{nullptr};//dy指示灯标签
    QLabel *dyaw_indicator_label_{nullptr};//dyaw指示灯标签
    QLabel *dx_value_label_{nullptr};//dx数值标签
    QLabel *dy_value_label_{nullptr};//dy数值标签
    QLabel *dyaw_value_label_{nullptr};//dyaw数值标签
    QPushButton *analysis_button_ = nullptr;//分析按钮
    QPushButton *execute_button_ = nullptr;//执行按钮
    QPushButton *waypoint_button_ = nullptr;//航点飞行按钮
    QPushButton *scheduled_check_button_ = nullptr;//执行按钮
    QLabel *time_label_ = nullptr;//时间标签

    RosManager *ros_manager_{nullptr};//ROS管理器

    QTimer *clock_timer_ = nullptr;//用于每秒刷新一次顶部时间

    /*********************ros移植部分***********************/
    QString trigger_time_text_;//目标触发时刻，当前先按 HH:mm:ss 保存
    QString last_triggered_time_text_;//记录最近一次已经触发过的时刻文本，避免同一秒重复触发
    bool time_trigger_enabled_ = false;//当前是否启用到点触发上传
    /******************************************************/

    QTimer *exit_long_press_timer_{nullptr};
    bool stop_button_pressed_{false};
    bool long_press_triggered_{false};
    int stop_press_token_{0};
};
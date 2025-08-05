#pragma once

#include <memory>

#include <QObject>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QProcess>
#include <QFileSystemWatcher>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "custom_msgs/action/behavior_tree.hpp"
#include "custom_msgs/srv/get_target_list.hpp"


class QmlCommandLine : public QObject
{
    Q_OBJECT
    
    // Properties exposed to QML
    // Q_PROPERTY(QString workingDirectory READ workingDirectory WRITE setWorkingDirectory NOTIFY workingDirectoryChanged)
    // Q_PROPERTY(bool missionActive READ missionActive NOTIFY missionActiveChanged)
    // Q_PROPERTY(QString currentStatus READ currentStatus NOTIFY statusChanged)
    // Q_PROPERTY(QStringList recentFiles READ recentFiles NOTIFY recentFilesChanged)
    // Q_PROPERTY(int commandHistorySize READ commandHistorySize NOTIFY commandHistoryChanged)

public:
    using ActionT = custom_msgs::action::BehaviorTree;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

    explicit QmlCommandLine(QObject *parent = nullptr);
    ~QmlCommandLine();
    Q_INVOKABLE void executeCommand(const QString &command);

signals:
    void commandRunning(const QString &msg);
    void commandError(const QString &errorMsg);
    void commandWarning(const QString &warningMsg);
    void commandInfo(const QString &infoMsg);
    void commandSuccess(const QString &msg);
    void commandFailure(const QString &msg);

private slots:
    void spinRos2Node();

private:
    void processRunCommand(const QStringList &args);
    void processDoCommand(const QStringList &args);
    void sendGoal(
        const std::string & xml_text,
        const int timeout
    );

    // Action client callbacks
    void btGoalResponseCb(const std::shared_ptr<GoalHandle> & goal_handle);
    void btFeedbackCb(std::shared_ptr<GoalHandle>, const std::shared_ptr<const ActionT::Feedback> feedback);
    void btResultCb(const GoalHandle::WrappedResult & result);

    void requestCancel();

    // Service client callback
    void getTargetListSrvResponseCb(
        rclcpp::Client<custom_msgs::srv::GetTargetList>::SharedFuture future
    );

    // ROS2 members
    std::shared_ptr<rclcpp::Node> ros2_node_;
    std::shared_ptr<rclcpp_action::Client<ActionT>> action_client_;
    std::shared_ptr<GoalHandle> goal_handle_;
    std::shared_ptr<rclcpp::Client<custom_msgs::srv::GetTargetList>> srv_client_;
    QTimer *ros2_spin_timer_;

    // Command execution thread (detached) management
    bool running_ {false};  // break the while loop in the thread when exiting

    // GCS's side file management
    QString working_directory_;

    // utility functions
    QString getHelpString() const;
};

#include "qml_cli.hpp"
#include "helpers.hpp"
#include <QDir>
#include <QFileInfo>
#include <QStandardPaths>
#include <QDateTime>
#include <QDebug>
#include <QUrl>
#include <QDesktopServices>
#include <fstream>
#include <sstream>


QmlCommandLine::QmlCommandLine(QObject *parent):
    QObject(parent)
{
    ros2_node_ = std::make_shared<rclcpp::Node>(
        "qml_cli_node",
        rclcpp::NodeOptions()
    );

    action_client_ = rclcpp_action::create_client<ActionT>(
        ros2_node_,
        "/execute_mission"
    );

    srv_client_ = ros2_node_->create_client<custom_msgs::srv::GetTargetList>(
        "/get_target_list",
        rmw_qos_profile_services_default
    );

    // Initialize working directory
    QString homeDir = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
    working_directory_ = homeDir + "/data/bt/";

    // Setup ROS2 spinning timer to update the node
    ros2_spin_timer_ = new QTimer(this);
    connect(ros2_spin_timer_, &QTimer::timeout, this, &QmlCommandLine::spinRos2Node);
    ros2_spin_timer_->start(10); // Spin every 10 ms
}

QmlCommandLine::~QmlCommandLine()
{
    running_ = false;
}

void QmlCommandLine::spinRos2Node()
{
    if (ros2_node_)
        rclcpp::spin_some(ros2_node_);
}

QString QmlCommandLine::getHelpString() const
{
    QStringList helpLines;
    helpLines << "Available commands:";
    helpLines << "\trun <filename>\t\t- Execute a behavior tree";
    helpLines << "\tdo <node_name>\t\t- Execute a single node";
    helpLines << "\tcancel\t\t\t- Cancel the running operation";
    helpLines << "\tid\t\t\t- Get the list of registered targets";
    helpLines << "\thelp\t\t\t- Show this help message";
    return helpLines.join("\n");
}

void QmlCommandLine::executeCommand(const QString &command)
{
    QString cleanCommand = command.trimmed();
    // addToCommandHistory(cleanCommand);

    if (cleanCommand.isEmpty())
        return;
    
    // Parse command and arguments
    QStringList parts = cleanCommand.split(' ', Qt::SkipEmptyParts);
    QString cmd = parts.takeFirst().toLower();
    QStringList args = parts;

    if (cmd == "help" || cmd == "h") {
        auto helpText = getHelpString();
        emit commandInfo(helpText);
    }
    else if (cmd == "run" || cmd == "r") {
        // Execute the command asynchronously to avoid blocking the UI
        std::thread(&QmlCommandLine::processRunCommand, this, args).detach();
        running_ = true;
    }
    else if (cmd == "do" || cmd == "d") {
        std::thread(&QmlCommandLine::processDoCommand, this, args).detach();
        running_ = true;
    }
    else if (cmd == "cancel" || cmd == "c") {
        requestCancel();
    }
    else if (cmd == "id") {
        if (!srv_client_->service_is_ready())
            emit commandError("/get_target_list service not available.");

        auto request = std::make_shared<custom_msgs::srv::GetTargetList::Request>();
        srv_client_->async_send_request(
            request,
            std::bind(&QmlCommandLine::getTargetListSrvResponseCb, this, std::placeholders::_1)
        );
    }
    else {
        emit commandError(QString("Unknown command: %1").arg(cmd));
    }
}

void QmlCommandLine::processRunCommand(const QStringList &args)
{
    if (args.isEmpty()) {
        emit commandInfo(QString("Usage: run <filename> or <filename>.xml"));
        return;
    }

    QString filename = args.first();
    if (!filename.endsWith(".xml")) {
        filename += ".xml";
    }

    // Read XML content from file
    std::string xml_text = readXmlFile(filename, working_directory_).toStdString();
    if (xml_text.empty()) {
        emit commandError(QString("Failed to read XML file: %1").arg(filename));
        return;
    }

    if (running_)
        emit commandRunning(QString("run %1").arg(filename));
    else {
        emit commandWarning("Run command canceled.");
        return;
    }

    // Send goal to action server
    sendGoal(xml_text, 5);
}

void QmlCommandLine::processDoCommand(const QStringList &args)
{
    if (args.isEmpty()) {
        emit commandInfo(QString("Usage: do <node_name>, e.g., do Disarm"));
        return;
    }

    // Build a single-node tree XML
    QString node_name = args.first();
    std::string xml_text = buildSingleNodeTree(node_name).toStdString();

    if (running_)
        emit commandRunning(QString("do %1").arg(node_name));
    else {
        emit commandWarning("Do command canceled.");
        return;
    }

    // Send goal to action server
    sendGoal(xml_text, 5);
}

void QmlCommandLine::sendGoal(
    const std::string & xml_text,
    const int timeout)
{
    using namespace std::placeholders;

    // Ensure that BT action server is active
    auto start_time = std::chrono::steady_clock::now();
    while (!action_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
        if (!running_) {
            // Exit if command was canceled
            emit commandWarning("Run command canceled.");
            return;
        }

        emit commandWarning("mission server not available, waiting...");
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(timeout))
        {
            emit commandError("Timeout waiting for mission server.");
            return;
        }
    }

    ActionT::Goal goal;
    goal.xml_text = xml_text;

    emit commandInfo("Sending goal...");

    rclcpp_action::Client<ActionT>::SendGoalOptions options;
    options.goal_response_callback =
        std::bind(&QmlCommandLine::btGoalResponseCb, this, _1);
    options.feedback_callback =
        std::bind(&QmlCommandLine::btFeedbackCb, this, _1, _2);
    options.result_callback =
        std::bind(&QmlCommandLine::btResultCb, this, _1);

    action_client_->async_send_goal(goal, options);
    emit commandInfo("Command execution thread exited.");
}

// -----------------------------------
//      Action client callbacks
// -----------------------------------
void QmlCommandLine::btGoalResponseCb(
    const std::shared_ptr<GoalHandle> & goal_handle)
{
    if (!goal_handle)
    {
        emit commandError("Goal was rejected by mission server.");
        return;
    }
    
    goal_handle_ = goal_handle;
    emit commandInfo("Goal accepted by mission server, waiting for result...");
}

void QmlCommandLine::btFeedbackCb(
    std::shared_ptr<GoalHandle>,
    const std::shared_ptr<const ActionT::Feedback> feedback)
{
    emit commandInfo(QString("<%1> feedback: %2").arg(
        QString::fromStdString(feedback->running_node),
        QString::fromStdString(feedback->message))
    );
}

void QmlCommandLine::btResultCb(
    const GoalHandle::WrappedResult & result)
{
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            if (result.result->message.empty())
                emit commandSuccess("Mission completed successfully.");
            else
                emit commandSuccess(QString("Mission completed successfully: %1").arg(QString::fromStdString(result.result->message)));
            break;
        case rclcpp_action::ResultCode::ABORTED:
            if (result.result->message.empty())
                emit commandFailure("Mission aborted.");
            else
                emit commandFailure(QString("Mission aborted: %1").arg(QString::fromStdString(result.result->message)));
            break;
        case rclcpp_action::ResultCode::CANCELED:
            emit commandFailure("Mission cancelled.");
            break;
        default:
            emit commandError("Unknown result code.");
            break;
    }

    goal_handle_ = nullptr;
}

void QmlCommandLine::requestCancel()
{
    if (running_)
    {
        running_ = false;

        // Cancel the running mission if it presents
        if (goal_handle_) 
        {
            emit commandWarning("Requesting mission cancellation...");

            action_client_->async_cancel_goal(
                goal_handle_, 
                [this](std::shared_ptr<action_msgs::srv::CancelGoal_Response> response) {
                    if (!response) {
                        emit commandError("Cancel response was null.");
                        return;
                    }

                    if (response->goals_canceling.empty()) {
                        emit commandError("Failed to cancel the mission.");
                    }
                }
            );

            goal_handle_ = nullptr;
        }
    }
    else {
        emit commandWarning("No active command to cancel.");
    }
}

// -----------------------------------
//      Service client callbacks
// -----------------------------------
void QmlCommandLine::getTargetListSrvResponseCb(
    rclcpp::Client<custom_msgs::srv::GetTargetList>::SharedFuture future)
{
    auto response = future.get();
    QString msg;
    for (const auto & id : response->target_ids)
    {
        msg += QString::fromStdString(id) + " ";
    }
    emit commandSuccess("Registered targets: " + msg);
}


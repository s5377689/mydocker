#include <filesystem>
#include <chrono>
#include <cstdlib>

#include <readline/readline.h>
#include <readline/history.h>
#include "custom_msgs/srv/get_target_list.hpp"
#include "cli.hpp"

namespace fs = std::filesystem;


BtClient::BtClient(
    const rclcpp::NodeOptions & options
):
    Node("bt_client_node", options),
    waiting_command_(true)
{
    action_client_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );
    srv_client_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive
    );
    
    action_client_ = rclcpp_action::create_client<ActionT>(
        this,
        "/execute_mission",
        action_client_cb_group_
    );

    srv_client_ = this->create_client<custom_msgs::srv::GetTargetList>(
        "/get_target_list",
        rmw_qos_profile_services_default,
        srv_client_cb_group_
    );

    command_thread_ = std::thread(&BtClient::process_command, this);
}

BtClient::~BtClient()
{
    waiting_command_ = false;
    if (command_thread_.joinable())
        command_thread_.join();
}

void BtClient::sendGoal(
    const std::string & xml_text,
    const int timeout)
{
    using namespace std::placeholders;

    // Ensure that BT action server is active
    auto start_time = std::chrono::steady_clock::now();
    while (!action_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
        RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "mission server not available, waiting...");
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(timeout))
        {
            RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "Timeout waiting for mission server.");
            return;
        }
    }

    ActionT::Goal goal;
    goal.xml_text = xml_text;

    RCLCPP_INFO(rclcpp::get_logger("bt_client"), "Sending goal...");

    rclcpp_action::Client<ActionT>::SendGoalOptions options;
    options.goal_response_callback =
        std::bind(&BtClient::btGoalResponseCb, this, _1);
    options.feedback_callback =
        std::bind(&BtClient::btFeedbackCb, this, _1, _2);
    options.result_callback =
        std::bind(&BtClient::btResultCb, this, _1);

    action_client_->async_send_goal(goal, options);
}


// -----------------------------------
//      Action client callbacks
// -----------------------------------
void BtClient::btGoalResponseCb(
    const std::shared_ptr<GoalHandle> & goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "Goal was rejected by mission server.");
        return;
    }
    
    goal_handle_ = goal_handle;
    RCLCPP_INFO(
        rclcpp::get_logger("bt_client"),
        "Goal accepted by mission server, waiting for result..."
    );
}

void BtClient::btFeedbackCb(
    std::shared_ptr<GoalHandle>,
    const std::shared_ptr<const ActionT::Feedback> feedback)
{
    RCLCPP_INFO(
        rclcpp::get_logger("bt_client"),
        "Feedback received: %s", feedback->message.c_str()
    );
}

void BtClient::btResultCb(
    const GoalHandle::WrappedResult & result)
{
    switch (result.code)
    {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(rclcpp::get_logger("bt_client"), "-- Goal succeeded --");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "-- Goal aborted --");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "-- Goal canceled --");
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "-- Unknown result code --");
            break;
    }

    RCLCPP_INFO(
        rclcpp::get_logger("bt_client"),
        "[Result: %s]", result.result->success? "<SUCCESS>" : "<FAILURE>"
    );
    RCLCPP_INFO(
        rclcpp::get_logger("bt_client"),
        "[Result message: %s]", result.result->message.c_str()
    );
    goal_handle_ = nullptr;
}

void BtClient::request_cancel()
{
    if (goal_handle_ == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "No active mission to cancel.");
        return;
    }

    RCLCPP_WARN(rclcpp::get_logger("bt_client"), "Requesting to cancel the mission...");

    action_client_->async_cancel_goal(
        goal_handle_, 
        [this](std::shared_ptr<action_msgs::srv::CancelGoal_Response> response) {
            if (!response) {
                RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "Cancel response was null.");
                return;
            }

            if (!response->goals_canceling.empty()) {
                RCLCPP_INFO(rclcpp::get_logger("bt_client"), "Mission cancelled successfully.");
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "Failed to cancel the mission.");
            }
        }
    );

    goal_handle_ = nullptr;
}

// -----------------------------------
//      Service client callbacks
// -----------------------------------
void BtClient::srvResponseCb(
    rclcpp::Client<custom_msgs::srv::GetTargetList>::SharedFuture future)
{
    auto response = future.get();
    std::cout << "Registered targets: " << std::endl;
    for (const auto & id : response->target_ids)
    {
        std::cout << id << " ";
    }
    std::cout << std::endl;
}

// -----------------------------------
//                CLI
// -----------------------------------
std::string BtClient::readXmlFile(const std::string & filename, const std::string & prefix)
{
    std::cout << "Reading XML file: " << prefix + filename << std::endl;
    std::ifstream file(prefix + filename);
    if (!file) {
        RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "Failed to open file: %s", filename.c_str());
        return "";
    }
    return {
        std::istreambuf_iterator<char>(file),
        std::istreambuf_iterator<char>()
    };
}

void BtClient::printHelp()
{
    std::cout << "=======================================================" << std::endl;
    std::cout << "Mission Client\n" << std::endl;

    std::cout << "Control" << std::endl;
    std::cout << "\trun(r) <filename>: Run a new mission" << std::endl;
    std::cout << "\tcancel(c): Cancel the current mission" << std::endl;

    std::cout << "Info" << std::endl;
    std::cout << "\tid: Get the list of registered targets" << std::endl;

    std::cout << "Editing" << std::endl;
    std::cout << "\tedit <filename>: Edit the mission file" << std::endl;
    std::cout << "\tcd <path>: Change working directory" << std::endl;
    std::cout << "\tls: List files in the current directory" << std::endl;
    std::cout << "\tcp <src> <dest>: Copy a file" << std::endl;
    std::cout << "\trm <filename>: Remove a file" << std::endl;
    std::cout << "\tpwd: Print the current working directory" << std::endl;

    std::cout << "Other" << std::endl;
    std::cout << "\tgroot2: Visualize behavior trees" << std::endl;
    std::cout << "\tquit: Quit the program" << std::endl;
    std::cout << "=======================================================" << std::endl;
}

void BtClient::process_command()
{
    std::string input;
    printHelp();

    while (waiting_command_)
    {
        try {
            char* raw_input = readline("$ ");
            if (!raw_input)
                continue;

            std::string input(raw_input);
            free(raw_input);

            std::transform(input.begin(), input.end(), input.begin(), ::tolower);

            // Trim leading/trailing whitespace
            input.erase(0, input.find_first_not_of(" \t"));
            input.erase(input.find_last_not_of(" \t") + 1);

            if (input.empty())
                continue;

            add_history(input.c_str());

            if (input == "c" || input == "cancel")
            {
                request_cancel();
            }
            else if (input.rfind("r ", 0) == 0 || input.rfind("run ", 0) == 0)
            {
                std::istringstream iss(input);
                std::string command, filename;
                iss >> command >> filename;

                if (filename.size() < 4 || filename.substr(filename.size() - 4) != ".xml")
                    filename += ".xml";

                if (!filename.empty())
                {
                    std::string xml_text = readXmlFile(filename, working_directory_);

                    if (!xml_text.empty())
                        sendGoal(xml_text);
                }
            }
            else if (input == "id")
            {
                if (!srv_client_->wait_for_service(std::chrono::seconds(2))) {
                    RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "Get target list service not available, retrying...");
                }

                auto request = std::make_shared<custom_msgs::srv::GetTargetList::Request>();
                srv_client_->async_send_request(
                    request,
                    std::bind(&BtClient::srvResponseCb, this, std::placeholders::_1)
                );
            }
            else if (input.rfind("cd ", 0) == 0)
            {
                std::istringstream iss(input);
                std::string command, path;
                iss >> command >> path;
                if (!path.empty())
                {
                    if (path == "..")
                    {
                        fs::path current_path = fs::path(working_directory_);
                        fs::path parent_path = current_path.parent_path().parent_path(); // since we have a trailing slash
                        if (!parent_path.empty())
                        {
                            working_directory_ = parent_path.string() + "/";
                            if (working_directory_.back() != '/')
                                working_directory_ += '/';
                            std::cout << "Changed working directory to: " << working_directory_ << std::endl;
                        }
                        else
                        {
                            RCLCPP_WARN(rclcpp::get_logger("bt_client"), "Already at the root directory");
                        }
                    }
                    else
                    {
                        fs::path target_path = fs::path(path);
                        if (!target_path.is_absolute())
                        {
                            fs::path base_path = fs::path(working_directory_);
                            target_path = base_path / target_path;
                        }

                        if (fs::exists(target_path) && fs::is_directory(target_path))
                        {
                            working_directory_ = target_path.lexically_normal().string();
                            if (path.back() != '/')
                                path += '/';
                            std::cout << "Changed working directory to: " << working_directory_ << std::endl;
                        }
                        else
                        {
                            RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "Invalid path: %s", path.c_str());
                        }
                    }
                }
            }
            else if (input == "pwd")
            {
                std::cout << working_directory_ << std::endl;
            }
            else if (input == "help" || input == "h")
            {
                printHelp();
            }
            else if (input == "ls")
            {
                try {
                    int item_count = 1;
                    const int max_items_per_line = 7;
                    for (const auto & entry : fs::directory_iterator(working_directory_))
                    {
                        if (entry.is_regular_file())
                        {
                            if (item_count % max_items_per_line == 0)
                                std::cout << std::endl;
                            std::cout << entry.path().filename().string() << " ";
                            ++item_count;
                        }
                    }
                    std::cout << std::endl;
                }
                catch (const std::filesystem::filesystem_error& e) {
                    RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "Failed to list files: %s", e.what());
                }
            }
            else if (input.rfind("edit ", 0) == 0)
            {
                std::istringstream iss(input);
                std::string command, filename;
                iss >> command >> filename;

                if (!filename.empty())
                {
                    std::string full_path = working_directory_ + filename;
                    if (fs::exists(full_path))
                    {
                        std::thread([full_path]() {
                            std::string editor = "gedit";
                            std::string command = editor + " " + full_path;
                            system(command.c_str());
                        }).detach();
                    }
                    else
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "File not found: %s", full_path.c_str());
                    }
                }
            }
            else if (input.rfind("cp ", 0) == 0)
            {
                std::istringstream iss(input);
                std::string command, src, dest;
                iss >> command >> src >> dest;

                if (!src.empty() && !dest.empty())
                {
                    std::string src_path = working_directory_ + src;
                    std::string dest_path = working_directory_ + dest;
                    if (fs::exists(src_path))
                    {
                        fs::copy(src_path, dest_path);
                        std::cout << "Copied " << src << " to " << dest << std::endl;
                    }
                    else
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "Source file not found: %s", src_path.c_str());
                    }
                }
            }
            else if (input.rfind("rm ", 0) == 0)
            {
                std::istringstream iss(input);
                std::string command, filename;
                iss >> command >> filename;

                if (!filename.empty())
                {
                    std::string full_path = working_directory_ + filename;
                    if (fs::exists(full_path))
                    {
                        fs::remove(full_path);
                        std::cout << "Removed file: " << full_path << std::endl;
                    }
                    else
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "File not found: %s", full_path.c_str());
                    }
                }
            }
            else if (input == "groot" || input == "groot2")
            {
                std::thread([this]() {
                    system("groot2");
                }).detach();
            }
            else if (input == "quit")
            {
                waiting_command_ = false;
                RCLCPP_INFO(rclcpp::get_logger("bt_client"), "Exiting...");
                rclcpp::shutdown();
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("bt_client"), "Unknown command: %s", input.c_str());
            }
        }
        catch (const std::exception & e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "Exception in command processing: %s", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(rclcpp::get_logger("bt_client"), "Unknown exception in command processing");
        }
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto bt_client = std::make_shared<BtClient>();
    const char* home = std::getenv("HOME");
    std::string work_dir = home ? std::string(home) + "/data/bt/" : "/home/";
    bt_client->setWorkingDirectory(work_dir);
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(),
        2
    );
    RCLCPP_INFO(rclcpp::get_logger("bt_client"), "Mission client started.");
    executor.add_node(bt_client);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
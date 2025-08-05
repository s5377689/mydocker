#include "system_component/mission_server.hpp"
#include "system_component/helpers.hpp"
#include "bt_action/all.hpp"


namespace system_component
{

MissionServer::MissionServer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
):
    Node("mission_server_node", options),
    bt_id_(""),
    bt_status_(BT::NodeStatus::IDLE),
    status_xml_(""),
    result_msg_(std::make_shared<std::string>("")),
    goal_handle_(nullptr),
    eo_rtsp_url_("rtsp://192.168.144.25:8554/main.264"),
    // ir_rtsp_url_("rtsp://192.168.144.6:8554"),
    gimbal_control_ip_("192.168.144.25"),
    gimbal_control_port_(37260)
{
    using namespace std::placeholders;

    // Create the gimbal camera streamers
    gimbal_eo_streamer_ = std::make_shared<gimbal::usv::GimbalStreamer>(
        eo_rtsp_url_, "/camera/compressed/image_raw", 0.25f
    );

    // Create the gimbal controller
    gimbal_controller_ = std::make_shared<gimbal::usv::GimbalController>(
        gimbal_control_ip_, gimbal_control_port_
    );

    // Create the action server for behavior tree execution
    this->action_server_ = rclcpp_action::create_server<ActionT>(
        this,
        "execute_mission",
        std::bind(&MissionServer::handle_goal, this, _1, _2),
        std::bind(&MissionServer::handle_cancel, this, _1),
        std::bind(&MissionServer::handle_accepted, this, _1)
    );

    // Initialize the target registry
    registry_ = std::make_shared<TargetRegistry>();

    factory_.registerNodeType<bt_action::Arm>("Arm");
    factory_.registerNodeType<bt_action::Disarm>("Disarm");
    factory_.registerNodeType<bt_action::Guided>("Guided");
    factory_.registerNodeType<bt_action::Hold>("Hold");
    factory_.registerNodeType<bt_action::Navigate>("Navigate");
    factory_.registerNodeType<bt_action::RegisterTarget>("RegisterTarget");
    factory_.registerNodeType<bt_action::Search>("Search");
    factory_.registerNodeType<bt_action::NavigateWhileSearch>("NavigateWhileSearch");
    factory_.registerNodeType<bt_action::StopGimbalControl>("StopGimbalControl");
    factory_.registerNodeType<bt_action::ResumeGimbalControl>("ResumeGimbalControl");
    factory_.registerNodeType<bt_action::TakePhoto>("TakePhoto");
    factory_.registerNodeType<bt_action::Zigzag>("Zigzag");

    std::string xml_models = BT::writeTreeNodesModelXML(factory_);
    write_xml_file("/home/robuff/data/bt/tree_nodes_model.xml", xml_models);
    // write_xml_file("/home/bill/data/bt/tree_nodes_model.xml", xml_models);
    RCLCPP_INFO(rclcpp::get_logger("mission_server"), "Tree node model has been written to file");

    // Create services
    get_target_list_service_ = this->create_service<custom_msgs::srv::GetTargetList>(
        "/get_target_list",
        std::bind(&MissionServer::get_target_list, this, _1, _2)
    );

    // Create a publisher for the current BT status
    status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&MissionServer::status_timer_callback, this)
    );
    status_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/bt_status",
        rclcpp::QoS(10)
    );

    RCLCPP_INFO(rclcpp::get_logger("mission_server"), "BT server started");
}

MissionServer::~MissionServer()
{}

void MissionServer::get_target_list(
    const std::shared_ptr<custom_msgs::srv::GetTargetList::Request>,
    std::shared_ptr<custom_msgs::srv::GetTargetList::Response> response)
{
    response->target_ids = registry_->get_target_list();
}

// =======================================================
//               BT Action Server Callbacks 
// =======================================================
rclcpp_action::GoalResponse MissionServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ActionT::Goal> goal)
{
    RCLCPP_INFO(rclcpp::get_logger("mission_server"), "Received mission request");
    (void)uuid;
    (void)goal;

    if (goal_handle_ && goal_handle_->is_active())
    {
        RCLCPP_ERROR(rclcpp::get_logger("mission_server"), "There is an active mission, cancel it before running a new one.");
        return rclcpp_action::GoalResponse::REJECT; 
    }
    else
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MissionServer::handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(rclcpp::get_logger("mission_server"), "Receive request to cancel current mission.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MissionServer::handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    using namespace std::placeholders;

    // Accept and start a new mission
    RCLCPP_INFO(rclcpp::get_logger("mission_server"), "Mission accepted.");
    goal_handle_ = goal_handle;
    std::thread{std::bind(&MissionServer::execute, this, _1), goal_handle}.detach();  // Detach so that mission server doesn't need to know when to join it
}

// =======================================================
//                 Behavior Tree execution
// =======================================================
void MissionServer::execute(
    const std::shared_ptr<GoalHandle> goal_handle)
{
    // Get the bt tree without status attributes
    try {
        const auto goal = goal_handle->get_goal();
        bt_id_ = get_bt_id(goal->xml_text);
        status_xml_ = goal->xml_text;
        tree_ = std::make_shared<BT::Tree>(
            factory_.createTreeFromText(remove_status_attributes(status_xml_))
        );
        bt_status_ = BT::NodeStatus::RUNNING;

        RCLCPP_INFO(rclcpp::get_logger("mission_server"), "Execute the mission.");

        // -------------------------------------------
        //            BT Node Initialization 
        // -------------------------------------------
        // Base initialize the BtNode with the result message
        auto base_visitor = [this](BT::TreeNode* node)
        {
            if (auto bt_node = dynamic_cast<bt_action::BtNode*>(node))
            {
                bt_node->baseInitialize(result_msg_, goal_handle_);
            }
        };

        // Pass references to initialize data members for each type of nodes
        auto visitor = [this](BT::TreeNode* node)
        {
            if (auto register_target_node = dynamic_cast<bt_action::RegisterTarget*>(node))
            {
                register_target_node->initialize(registry_);
            }
            else if (auto search_node = dynamic_cast<bt_action::Search*>(node))
            {
                search_node->initialize(registry_, gimbal_eo_streamer_, gimbal_controller_);
            }
            else if (auto nav_search_node = dynamic_cast<bt_action::NavigateWhileSearch*>(node))
            {
                nav_search_node->initialize(registry_, gimbal_eo_streamer_, gimbal_controller_);
            }
            else if (auto take_photo_node = dynamic_cast<bt_action::TakePhoto*>(node))
            {
                take_photo_node->initialize(gimbal_eo_streamer_);
            }
            else if (auto stop_gimbal_node = dynamic_cast<bt_action::StopGimbalControl*>(node))
            {
                stop_gimbal_node->initialize(gimbal_controller_);
            }
            else if (auto resume_gimbal_node = dynamic_cast<bt_action::ResumeGimbalControl*>(node))
            {
                resume_gimbal_node->initialize(gimbal_controller_);
            }
            // else if (auto stop_gimbal_streamer_node = dynamic_cast<StopGimbalStreamer*>(node))
            // {
            //     stop_gimbal_streamer_node->initialize(gimbal_eo_streamer_);
            // }
            // else if (auto resume_gimbal_streamer_node = dynamic_cast<ResumeGimbalStreamer*>(node))
            // {
            //     resume_gimbal_streamer_node->initialize(gimbal_eo_streamer_);
            // }
        };
        tree_->applyVisitor(base_visitor);
        tree_->applyVisitor(visitor);
        *result_msg_ = "";

        // -------------------------------------------
        //    Resume previos BT status if provided 
        // -------------------------------------------
        // Apply the status from the XML to the BT tree
        auto status_doc = tinyxml2::XMLDocument();
        status_doc.Parse(status_xml_.c_str());
        auto tree_root = status_doc.FirstChildElement("root")->FirstChildElement("BehaviorTree");
        apply_status_from_xml_element(tree_->rootNode(), tree_root->FirstChildElement());

    }
    catch (const std::exception & e) {
        std::string result_msg = "Failed to create BT tree: " + std::string(e.what());
        RCLCPP_ERROR(rclcpp::get_logger("mission_server"), "%s", result_msg.c_str());
        auto result = std::make_shared<ActionT::Result>();
        result->success = false;
        result->message = result_msg;
        goal_handle_->abort(result);

        resetBt();
        return;
    }
    catch (...) {
        std::string result_msg = "Failed to create BT tree: Unknown error";
        RCLCPP_ERROR(rclcpp::get_logger("mission_server"), "%s", result_msg.c_str());
        auto result = std::make_shared<ActionT::Result>();
        result->success = false;
        result->message = result_msg;
        goal_handle_->abort(result);

        resetBt();
        return;
    }

    // -------------------------------------------
    //                  Main loop
    // -------------------------------------------
    while (rclcpp::ok() && bt_status_ == BT::NodeStatus::RUNNING)
    {
        try {
            bt_status_ = tree_->tickOnce();

            // If a cancel is requested, clear the resource and close the execution thread
            if (goal_handle->is_canceling())
            {
                tree_->haltTree();
                bt_status_ = BT::NodeStatus::FAILURE;

                auto result = std::make_shared<ActionT::Result>();
                result->success = false;
                goal_handle->canceled(result);

                RCLCPP_WARN(rclcpp::get_logger("mission_server"), "-- Mission cancelled --");
                resetBt();
                return;
            }

            rclcpp::sleep_for(std::chrono::milliseconds(10));
        }
        catch (const std::exception & e)
        {
            std::string result_msg = "Exception during BT execution: " + std::string(e.what());
            RCLCPP_ERROR(rclcpp::get_logger("mission_server"), "%s", result_msg.c_str());
            auto result = std::make_shared<ActionT::Result>();
            result->success = false;
            result->message = result_msg;
            goal_handle_->abort(result);

            resetBt();
            return;
        }
        catch (...)
        {
            std::string result_msg {"Unknown exception during BT execution"};
            RCLCPP_ERROR(rclcpp::get_logger("mission_server"), "%s", result_msg.c_str());
            auto result = std::make_shared<ActionT::Result>();
            result->success = false;
            result->message = result_msg;
            goal_handle_->abort(result);

            resetBt();
            return;
        }
    }

    // -------------------------------------------
    //        Post-process mission result
    // -------------------------------------------
    try {
        auto result = std::make_shared<ActionT::Result>();
        if (bt_status_ == BT::NodeStatus::SUCCESS)
        {
            RCLCPP_INFO(rclcpp::get_logger("mission_server"), "-- Mission succeeded --");
            result->success = true;
            result->message = *result_msg_;
            goal_handle->succeed(result);
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("mission_server"), "-- Mission failed --");
            result->success = false;
            result->message = *result_msg_;
            goal_handle->abort(result);
        }

        resetBt();
    }
    catch (const std::exception & e)
    {
        std::string result_msg = "Exception during result processing: " + std::string(e.what());
        RCLCPP_ERROR(rclcpp::get_logger("mission_server"), "%s", result_msg.c_str());
        auto result = std::make_shared<ActionT::Result>();
        result->success = false;
        result->message = result_msg;
        goal_handle_->abort(result);

        resetBt();
    }
    catch (...)
    {
        std::string result_msg {"Unknown exception during result processing"};
        RCLCPP_ERROR(rclcpp::get_logger("mission_server"), "%s", result_msg.c_str());
        auto result = std::make_shared<ActionT::Result>();
        result->success = false;
        result->message = result_msg;
        goal_handle_->abort(result);

        resetBt();
    }
}

void MissionServer::resetBt()
{
    // Publish the final BT status before resetting it
    if (goal_handle_ && tree_)
        update_and_publish_bt_status();

    goal_handle_.reset();
    tree_.reset();
}

// =======================================================
//                   BT Status Update 
// =======================================================
void MissionServer::status_timer_callback()
{
    // Update and publish the status XML every second
    if (!goal_handle_ || !tree_)
    {
        return;
    }

    if (tree_->rootNode()->status() != BT::NodeStatus::IDLE)
        update_and_publish_bt_status();
}

void MissionServer::update_and_publish_bt_status()
{
    // Update the current status XML of the BT tree
    tinyxml2::XMLDocument doc;
    auto decl = doc.NewDeclaration();
    doc.InsertFirstChild(decl);
    auto root = doc.NewElement("root");
    doc.InsertEndChild(root);
    auto bt = doc.NewElement("BehaviorTree");
    root->InsertEndChild(bt);

    // Append ID and status attributes to the BehaviorTree node
    bt->SetAttribute("ID", bt_id_.c_str());
    bt->SetAttribute("status", BT::toStr(bt_status_).c_str());

    auto first_node = tree_->rootNode();
    build_xml_with_status(first_node, doc, bt);

    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    status_xml_ = printer.CStr();
    write_xml_file("/home/robuff/data/bt/bt_status.xml", status_xml_);
    // write_xml_file("/home/bill/data/bt/bt_status.xml", status_xml_);

    // Publish the status XML to a topic
    auto msg = std_msgs::msg::String();
    msg.data = status_xml_;
    status_publisher_->publish(msg);
}

}  // namespace system_component

// =======================================================
// =======================================================
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto mission_server = std::make_shared<system_component::MissionServer>();
    RCLCPP_INFO(rclcpp::get_logger("mission_server"), "-- Behavior tree mission server started --");
    rclcpp::spin(mission_server);
    rclcpp::shutdown();
    return 0;
}
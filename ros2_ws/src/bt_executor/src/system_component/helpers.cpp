#include "system_component/helpers.hpp"
#include <fstream>
using namespace tinyxml2;
using namespace std;

namespace system_component
{

// =============================================
//             XML Read-Write Helpers
// =============================================
void write_xml_file(const std::string & filename, const std::string & text)
{
    std::ofstream file(filename);
    if (!file) {
        cerr << "Failed to open file for writing" << endl;
        return;
    }
    file << text;
    file.close();
}

string remove_status_attributes(const string & xml_text)
{
    XMLDocument doc;
    if (doc.Parse(xml_text.c_str()) != XML_SUCCESS)
    {
        cout << "Error loading file: " << doc.ErrorIDToName(doc.ErrorID()) << endl;
        return "";
    }
    auto root = doc.FirstChildElement("root");
    if (root == nullptr)
    {
        cout << "Error: No root element found" << endl;
        return "";
    }

    remove_children_status_attributes(root);

    XMLPrinter printer;
    doc.Print(&printer);
    string xml_out {printer.CStr()};

    return xml_out;
}

void remove_children_status_attributes(XMLElement* node)
{
    if (node == nullptr)
        return;

    if (node->Attribute("status"))
    {
        node->DeleteAttribute("status");
    }

    for (auto child = node->FirstChildElement(); child; child = child->NextSiblingElement())
        remove_children_status_attributes(child);
}

void apply_status_from_xml_element(
    BT::TreeNode* bt_node,
    tinyxml2::XMLElement* xml_node)
{
    string status_str;
    if (!xml_node->Attribute("status"))
        status_str = "IDLE";
    else
        status_str = xml_node->Attribute("status");

    // Default to IDLE if not explicitly set
    if (status_str.empty())
        status_str = "IDLE";

    BT::NodeStatus status;
    if (status_str == "IDLE")
        status = BT::NodeStatus::IDLE;
    else if (status_str == "RUNNING")
        // set to IDLE for previously RUNNING node
        status = BT::NodeStatus::IDLE;
    else if (status_str == "SUCCESS")
        status = BT::NodeStatus::SUCCESS;
    else if (status_str == "FAILURE")
        status = BT::NodeStatus::FAILURE;
    else
        throw std::runtime_error("Invalid status value: " + status_str);

    // Expose the protected setStatus method
    struct StatusSetter : public BT::TreeNode
    {
        using BT::TreeNode::setStatus;
        using BT::TreeNode::resetStatus;
    };
    // cout << "Setting status of node: " << bt_node->name() << " to " << status_str << endl;
    if (status == BT::NodeStatus::IDLE)
        static_cast<StatusSetter*>(bt_node)->resetStatus();
    else
        static_cast<StatusSetter*>(bt_node)->setStatus(status);

    // Apply status change to children recursively
    auto comp = dynamic_cast<BT::ControlNode*>(bt_node);
    if (!comp)
        return;
    XMLElement* xml_child = xml_node->FirstChildElement();
    for (unsigned i = 0; i < comp->childrenCount(); ++i)
    {
        // Apply status to each child pair
        apply_status_from_xml_element(
            const_cast<BT::TreeNode*>(comp->child(i)),
            xml_child
        );
        xml_child = xml_child->NextSiblingElement();
    }
}

string get_bt_id(
    const string & xml_text)
{
    XMLDocument doc;
    if (doc.Parse(xml_text.c_str()) != XML_SUCCESS)
    {
        cout << "Error loading file: " << doc.ErrorIDToName(doc.ErrorID()) << endl;
        return "";
    }
    auto root = doc.FirstChildElement("root");
    if (root == nullptr)
    {
        cout << "Error: No root element found" << endl;
        return "";
    }

    auto bt = root->FirstChildElement("BehaviorTree");
    if (bt == nullptr)
    {
        cout << "Error: No BehaviorTree element found" << endl;
        return "";
    }

    if (!bt->Attribute("ID"))
    {
        cout << "Error: No id attribute found in BehaviorTree element" << endl;
        return "";
    }
    string id = bt->Attribute("ID");
    return id; 
}

void build_xml_with_status(
    const BT::TreeNode* bt_node,
    tinyxml2::XMLDocument & out_doc,
    tinyxml2::XMLElement* parent_xml_node)
{
    if (!bt_node)
        return;

    auto xml_node = out_doc.NewElement(bt_node->name().c_str());
    xml_node->SetAttribute("status", BT::toStr(bt_node->status()).c_str());

    parent_xml_node->InsertEndChild(xml_node);

    // If the node is not a leaf node, iterate over its children
    auto ctrl_node = dynamic_cast<const BT::ControlNode*>(bt_node);
    if (ctrl_node)
    {
        for (unsigned i = 0; i < ctrl_node->childrenCount(); ++i)
        {
            build_xml_with_status(
                ctrl_node->child(i),
                out_doc,
                xml_node
            );
        }
    }
    else if (auto decorator_node = dynamic_cast<const BT::DecoratorNode*>(bt_node))
    {
        build_xml_with_status(
            decorator_node->child(),
            out_doc,
            xml_node
        );
    }
}

}  // namespace system_component
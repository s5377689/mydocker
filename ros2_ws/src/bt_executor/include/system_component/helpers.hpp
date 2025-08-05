#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <limits>
#include <utility>
#include <cmath>
#include <tinyxml2.h>
#include "behaviortree_cpp/bt_factory.h"

namespace system_component
{

void write_xml_file(
    const std::string & filename,
    const std::string & text
);
std::string remove_status_attributes(const std::string & xml_text);
void remove_children_status_attributes(tinyxml2::XMLElement* node);
void apply_status_from_xml_element(
    BT::TreeNode* bt_node,
    tinyxml2::XMLElement* xml_node
);
std::string get_bt_id(const std::string & xml_text);
void build_xml_with_status(
    const BT::TreeNode* bt_node,
    tinyxml2::XMLDocument & out_doc,
    tinyxml2::XMLElement* parent_xml_node
);

}  // namespace system_component

// BT XML Export Functions
function exportBT(btRoot) {
    console.log("Exporting BT XML...")
    if (btRoot) {
        let xml = '<root BTCPP_format="4">\n'
        xml += '\t<BehaviorTree ID="' + btRoot.name + '">\n'
        xml += exportNodeToXml(btRoot, 2)
        xml += '\t</BehaviorTree>\n'
        xml += '</root>\n'

        console.log("Completed BT XML export:")
        console.log(xml)
        return xml
    }
    return ""
}

function exportNodeToXml(node, indentLevel) {
    let indent = '\t'.repeat(indentLevel)
    let xml = ""

    // Skip <BehaviorTree> tag
    if (node.name !== btRoot.name) {
        xml += `${indent}<${node.name}`

        // Add attributes
        for (let key in node.attributes) {
            xml += ` ${key}="${node.attributes[key]}"`
        }

        if (node.children.length === 0) {
            xml += "/>\n"
        } else {
            xml += ">\n"
            for (let i = 0; i < node.children.length; ++i) {
                xml += exportNodeToXml(node.children[i], indentLevel + 1)
            }
            xml += `${indent}</${node.name}>\n`
        }
    } else {
        // For root node, just export its children
        for (let i = 0; i < node.children.length; ++i) {
            xml += exportNodeToXml(node.children[i], indentLevel)
        }
    }

    return xml
}

function sameTree(tree1, tree2) {
    if (!tree1 && !tree2)
        return true
    if (!tree1 || !tree2)
        return false

    return sameNode(tree1, tree2)
}

function sameNode(node1, node2) {
    // Compare node name
    if (node1.name !== node2.name)
        return false

    // Compare node attributes
    if (!sameAttributes(node1.attributes, node2.attributes))
        return false

    // Compare children count
    if (node1.children.length !== node2.children.length)
        return false

    // Recursively compare children
    for (let i = 0; i < node1.children.length; ++i) {
        if (!sameNode(node1.children[i], node2.children[i])) {
            return false
        }
    }

    return true
}

function sameAttributes(attrs1, attrs2) {
    if (!attrs1 && !attrs2)
        return true
    if (!attrs1 || !attrs2)
        return false

    const keys1 = Object.keys(attrs1)
    const keys2 = Object.keys(attrs2)

    if (keys1.length !== keys2.length)
        return false

    for (let key of keys1) {
        if (attrs1[key] !== attrs2[key])
            return false
    }

    return true
}

function getNodeTypeIcon(nodeType) {
    switch (nodeType) {
        case "Action":
            return "qrc:/icons/action_icon.png"
        case "Condition":
            return "qrc:/icons/condition_icon.png"
        case "Decorator":
            return "qrc:/icons/decorator_icon.png"
        case "Control":
            return "qrc:/icons/control_icon.png"
        case "Subtree":
            return "qrc:/icons/subtree_icon.png"
        case "Root":
            return "qrc:/icons/root_icon.png"
        default:
            return "qrc:/icons/default_icon.png"
    }
}
#include "bt_node.hpp"


void BtNode::rename(const QString & new_name)
{
    m_name = new_name;
}

QList<QObject*> BtNode::children() const
{
    QList<QObject*> list;

    for (auto child : m_children)
        list.append(child);

    return list;
}

void BtNode::addChild(BtNode* child)
{
    m_children.append(child);
    child->setParent(this);
}

void BtNode::setAttributes(const QVariantMap & attributes)
{
    if (m_attributes != attributes)
    {
        m_attributes = attributes;
        emit attributesChanged();
    }
}

BtNode* BtNode::clone(QObject* parent) const
{
    BtNode* cloned = new BtNode(m_name, parent);
    cloned->setStatus(m_status);
    cloned->setAttributes(m_attributes);

    // Clone children
    for (BtNode* child : m_children)
    {
        cloned->addChild(child->clone(cloned));
    }

    return cloned;
}

QString BtNode::nodeType() const
{
    if (m_isRoot)
        return "Root";

    if (m_name.isEmpty())
        return "Unknown";

    static const QSet<QString> controlNodes = {
        "Sequence", "Selector", "Parallel", "Fallback", "SubTree"
    };
    static const QSet<QString> conditionNodes = {
        "Condition", "CheckBlackboard", "CheckParameter"
    };

    if (m_name == "Root")
        return "Root";
    else if (controlNodes.contains(m_name))
        return "Control";
    else if (conditionNodes.contains(m_name))
        return "Condition";
    else
        return "Action";
}
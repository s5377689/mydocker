#include "btnode.h"


QString BtNode::name() const
{
    return m_name;
}

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
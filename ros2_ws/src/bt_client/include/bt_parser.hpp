#pragma once
#include <QObject>
#include <QDomDocument>
#include <QString>
#include "bt_node.hpp"


class BtParser : public QObject
{
    Q_OBJECT
public:
    Q_INVOKABLE BtNode* parseFromFile(const QString & filepath);
    Q_INVOKABLE BtNode* parseFromXml(const QString & xml_text);
    // Q_INVOKABLE QString getNodeTypeByName(const QString & nodeName);
    Q_INVOKABLE BtNode* createBtNode(const QString & name);

private:
    BtNode* parseElement(
        const QDomElement & element,
        const QString & new_name = ""
    );
};
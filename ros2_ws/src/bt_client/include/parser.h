#pragma once
#include <QObject>
#include <QDomDocument>
#include <QString>
#include "btnode.h"


class BtParser : public QObject
{
    Q_OBJECT
public:
    Q_INVOKABLE BtNode* parseFromFile(const QString & filepath);
    Q_INVOKABLE BtNode* parseFromXml(const QString & xml_text);

private:
    BtNode* parseElement(
        const QDomElement & element,
        const QString & new_name = ""
    );
};
#pragma once
#include <QObject>
#include <QDomDocument>
#include <QString>
#include <QList>
#include <QVariantMap>
#include "bt_node.hpp"


class TreeNodesModelParser : public QObject
{
    Q_OBJECT

public:
    explicit TreeNodesModelParser(QObject *parent = nullptr);
    ~TreeNodesModelParser();

    Q_INVOKABLE QList<BtNode*> parseFromFile(const QString & filePath);
    Q_INVOKABLE QList<BtNode*> parseFromXml(const QString & xmlText);
    
    Q_INVOKABLE QList<BtNode*> availableNodes() const { return m_availableNodes; }
    Q_INVOKABLE QList<BtNode*> getNodesByType(const QString & nodeType) const;
    Q_INVOKABLE BtNode* getNodeById(const QString & nodeId) const;

    void clearNodes();

signals:
    void parsingError(const QString & error);

private:
    QList<BtNode*> parseDomDocument(const QDomDocument & doc);
    // BtNode* parseNodeElement(const QDomElement & element, const QString & nodeType);
    BtNode* parseNodeElement(const QDomElement & element);
    QVariantMap parseInputPorts(const QDomElement & nodeElement);

    // All parsed nodes
    QList<BtNode*> m_availableNodes;
    QString m_lastError;
};
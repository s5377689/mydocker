#include "tree_nodes_model_parser.hpp"
#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QFileInfo>


TreeNodesModelParser::TreeNodesModelParser(QObject *parent)
    : QObject(parent)
{}

TreeNodesModelParser::~TreeNodesModelParser()
{
    clearNodes();
}

QList<BtNode*> TreeNodesModelParser::getNodesByType(const QString &nodeType) const
{
    QList<BtNode*> filteredNodes;
    for (const auto &node : m_availableNodes)
    {
        if (node->attributes().value("nodeType").toString() == nodeType)
        {
            filteredNodes.append(node);
        }
    }
    return filteredNodes;
}

BtNode* TreeNodesModelParser::getNodeById(const QString & nodeId) const
{
    for (const auto &node : m_availableNodes)
    {
        if (node->name() == nodeId)
        {
            return node;
        }
    }
    return nullptr; // Node not found
}

QList<BtNode*> TreeNodesModelParser::parseFromFile(const QString &filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        m_lastError = QString("Cannot open file: %1").arg(filePath);
        emit parsingError(m_lastError);
        return QList<BtNode*>();
    }

    QTextStream stream(&file);
    QString xmlText = stream.readAll();
    file.close();

    qDebug() << "Parsing tree nodes model from file: " << filePath;
    return parseFromXml(xmlText);
}

QList<BtNode*> TreeNodesModelParser::parseFromXml(const QString &xmlText)
{
    clearNodes();

    QDomDocument doc;
    QString errorMsg;
    int errorLine, errorColumn;

    if (!doc.setContent(xmlText, &errorMsg, &errorLine, &errorColumn))
    {
        m_lastError = QString("XML parsing error: %1 at line %2, column %3")
            .arg(errorMsg).arg(errorLine).arg(errorColumn);
        emit parsingError(m_lastError);
        return QList<BtNode*>();
    }

    try {
        m_availableNodes = parseDomDocument(doc);
        qDebug() << "Successfully parsed " << m_availableNodes.size() << " node templates.";
        return m_availableNodes;
    } catch (const std::exception &e) {
        m_lastError = QString("Exception during parsing: %1").arg(e.what());
        emit parsingError(m_lastError);
        return QList<BtNode*>();
    }
}

QList<BtNode*> TreeNodesModelParser::parseDomDocument(const QDomDocument &doc)
{
    QList<BtNode*> nodes;

    // Find root element
    QDomElement rootElement = doc.documentElement();
    if (rootElement.tagName() != "root")
        throw std::runtime_error("Root element must be <root>");

    // Find TreeNodesModel element
    QDomElement treeNodesModelElement = rootElement.firstChildElement("TreeNodesModel");
    if (treeNodesModelElement.isNull())
        throw std::runtime_error("No <TreeNodesModel> element found");

    // Parse each node type
    QStringList nodeTypes = {"Action", "Condition", "Control", "Decorator", "SubTree"};

    for (const QString &nodeType : nodeTypes)
    {
        QDomNodeList nodeList = treeNodesModelElement.elementsByTagName(nodeType);

        for (int i = 0; i < nodeList.count(); ++i)
        {
            QDomElement nodeElement = nodeList.at(i).toElement();
            if (nodeElement.isNull())
                continue;
            BtNode* node = parseNodeElement(nodeElement);
            if (node) {
                nodes.append(node);
                // qDebug() << "Parsed node:" << node->name() << "of type" << nodeType;
            }
        }
    }

    return nodes;
}

BtNode* TreeNodesModelParser::parseNodeElement(
    const QDomElement & element)
{
    QString nodeId = element.attribute("ID");
    if (nodeId.isEmpty())
    {
        m_lastError = QString("Node ID is missing");
        return nullptr;
    }

    BtNode* node = new BtNode(nodeId);
    QVariantMap attributes = parseInputPorts(element);
    node->setAttributes(attributes);

    return node;
}

QVariantMap TreeNodesModelParser::parseInputPorts(const QDomElement & nodeElement)
{
    QVariantMap attributes;
    QDomNodeList inputPortList = nodeElement.elementsByTagName("input_port");
    const char* defaultValue = "";

    for (int i = 0; i < inputPortList.count(); ++i)
    {
        QDomElement portElement = inputPortList.at(i).toElement();

        if (portElement.isNull())
            continue;

        QString portName = portElement.attribute("name");
        
        if (portName.isEmpty())
        {
            std::cerr << "Input port name is missing in <input_port> element" << std::endl;
            continue;
        }

        attributes[portName] = defaultValue;
    }

    return attributes;
}

void TreeNodesModelParser::clearNodes()
{
    for (auto node : m_availableNodes)
    {
        node->deleteLater();
    }
    m_availableNodes.clear();
}
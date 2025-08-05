#include "parser.h"
#include <QFile>


Q_INVOKABLE BtNode* BtParser::parseFromFile(
    const QString & filepath)
{
    QFile file(filepath);
    if (!file.open(QIODevice::ReadOnly))
    {
        qWarning("Failed to open file: %s", qUtf8Printable(filepath));
        return nullptr;
    }

    QDomDocument doc;
    if (!doc.setContent(&file))
    {
        qWarning("Failed to parse XML content");
        file.close();
        return nullptr;
    }

    auto rootElement = doc.documentElement().firstChildElement("BehaviorTree");

    if (rootElement.isNull())
    {
        qWarning("No element named 'BehaviorTree' found");
        file.close();
        return nullptr;
    }

    return parseElement(rootElement, rootElement.attribute("ID"));
}

Q_INVOKABLE BtNode* BtParser::parseFromXml(
    const QString & xml_text)
{
    QDomDocument doc;
    if (!doc.setContent(xml_text))
    {
        qWarning("Failed to parse XML content");
        return nullptr;
    }

    auto rootElement = doc.documentElement().firstChildElement("BehaviorTree");

    if (rootElement.isNull())
    {
        qWarning("No element named 'BehaviorTree' found");
        return nullptr;
    }

    return parseElement(rootElement, rootElement.attribute("ID"));
}

BtNode* BtParser::parseElement(
    const QDomElement & element,
    const QString & new_name)
{
    // Name the node with its tag name or the new name if provided
    BtNode* node = new BtNode(
        new_name.isEmpty() ? element.tagName() : new_name
    );

    // Set the status if it exists
    if (element.hasAttribute("status"))
    {
        node->setStatus(element.attribute("status"));
    }

    QDomNodeList children = element.childNodes();

    for (int i = 0; i < children.size(); ++i)
    {
        if (children.at(i).isElement())
            node->addChild(
                parseElement(children.at(i).toElement())
            );
    }

    return node;
}
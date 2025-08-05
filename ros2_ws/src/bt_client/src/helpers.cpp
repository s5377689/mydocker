// helpers.cpp
#include "helpers.hpp"
#include <QFile>
#include <QTextStream>
#include <QFileInfo>
#include <QDebug>

QString readXmlFile(const QString & filename, const QString & prefix)
{
    QString fullPath = prefix + filename;
    qDebug() << "Reading XML file:" << fullPath;

    QFileInfo fileInfo(fullPath);
    if (!fileInfo.exists()) {
        qWarning() << "File not found:" << fullPath;
        return QString();
    }
    if (!fileInfo.isFile()) {
        qWarning() << "Not a file:" << fullPath;
        return QString();
    }
    if (!fileInfo.isReadable()) {
        qWarning() << "File is not readable:" << fullPath;
        return QString();
    }
    
    QFile file(fullPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Failed to open file:" << filename;
        return QString();
    }
    
    QTextStream in(&file);
    QString content = in.readAll();

    if (in.status() != QTextStream::Ok) {
        qWarning() << "Failed to read file:" << fullPath;
        return QString();
    }

    file.close();
    
    return content;
}

QString buildSingleNodeTree(
    const QString & node_name)
{
    QString tree = QString("<root BTCPP_format=\"4\">\n");
    tree += QString("\t<BehaviorTree ID=\"%1 Action\">\n").arg(node_name);
    tree += QString("\t\t<%1/>\n").arg(node_name);
    tree += QString("\t</BehaviorTree>\n");
    tree += QString("</root>\n");
    return tree;
}
#include "file_manager.hpp"
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QVariantMap>
#include <QDebug>


FileManager::FileManager(QObject *parent)
    : QObject(parent)
{}

bool FileManager::saveTextFile(
    const QString & text,
    const QString & filePath,
    const QString & fileName)
{
    QDir dir;
    if (!dir.exists(filePath)) {
        dir.mkpath(filePath);
        qDebug() << "Directory created:" << filePath;
    }

    QString fullPath = filePath + fileName;
    QFile file(fullPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qWarning() << "Failed to open file for writing:" << fullPath;
        return false;
    }

    QTextStream out(&file);
    out << text;
    file.close();

    qDebug() << "File saved successfully: " << fullPath;
    return true;
}

QString FileManager::loadTextFile(
    const QString & filePath,
    const QString & fileName)
{
    QString fullPath = filePath + fileName;
    QFile file(fullPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Failed to open file for reading:" << fullPath;
        return QString();
    }

    QTextStream in(&file);
    QString text = in.readAll();
    file.close();

    qDebug() << "File loaded successfully: " << fullPath;
    return text;
}

QVariantList FileManager::loadWaypoints(
    const QString & fullFilePath)
{
    QVariantList waypoints;

    QFile file(fullFilePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Failed to open waypoints file:" << fullFilePath;
        return waypoints;
    }

    QTextStream in (&file);
    QString content = in.readAll();
    file.close();

    // Parse JSON
    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(content.toUtf8(), &error);
    
    if (error.error != QJsonParseError::NoError) {
        qWarning() << "JSON parse error:" << error.errorString();
        return waypoints;
    }

    QJsonObject planJson = doc.object();
    QJsonObject mission = planJson["mission"].toObject();
    QJsonArray items = mission["items"].toArray();

    // Extract waypoints from mission items
    for (const auto & itemValue : items) {
        QJsonObject item = itemValue.toObject();

        // Check if the item is a waypoint or ROI command
        if (item["command"].toInt() == 16 || item["command"].toInt() == 21) { // MAV_CMD_NAV_WAYPOINT or ROI
            QJsonArray params = item["params"].toArray();

            if (params.size() >= 7) {
                QVariantMap waypoint;
                waypoint["lat"] = params[4].toDouble();
                waypoint["lon"] = params[5].toDouble();
                waypoint["alt"] = params[6].toDouble();

                waypoints.append(waypoint);
            }
        }
    }

    return waypoints;
}

#pragma once
#include <QObject>
#include <QString>


class FileManager : public QObject
{
    Q_OBJECT

public:
    explicit FileManager(QObject *parent = nullptr);

public slots:
    Q_INVOKABLE bool saveTextFile(
        const QString & text,
        const QString & filePath,
        const QString & fileName
    );
    Q_INVOKABLE QString loadTextFile(
        const QString & filePath,
        const QString & fileName
    );
    Q_INVOKABLE QVariantList loadWaypoints(
        const QString & fullFilePath
    );
};
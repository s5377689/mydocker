#pragma once
#include <QObject>
#include <QList>

class BtNode : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString name READ name CONSTANT)
    Q_PROPERTY(QList<QObject*> children READ children CONSTANT)
    Q_PROPERTY(QString status READ getStatus CONSTANT)

public:
    explicit BtNode(
        const QString & name = "",
        QObject* parent = nullptr
    ):
        QObject(parent), m_name(name), m_status("IDLE")
    {}

    QString name() const;
    void rename(const QString & new_name);
    QList<QObject*> children() const;
    void addChild(BtNode* child);

    void setStatus(const QString & status) { m_status = status; }
    QString getStatus() const { return m_status; }

private:
    QString m_name;
    QString m_status;
    QList<BtNode*> m_children;
};
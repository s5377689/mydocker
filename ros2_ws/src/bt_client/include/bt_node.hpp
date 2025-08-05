#pragma once
#include <QObject>
#include <QList>
#include <QVariantMap>

class BtNode : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString name READ name CONSTANT)
    Q_PROPERTY(int id READ id WRITE setId NOTIFY idChanged)
    Q_PROPERTY(QList<QObject*> children READ children CONSTANT)
    Q_PROPERTY(QString status READ status WRITE setStatus)
    Q_PROPERTY(QVariantMap attributes READ attributes WRITE setAttributes NOTIFY attributesChanged)
    Q_PROPERTY(bool isRoot READ isRoot WRITE setIsRoot)
    Q_PROPERTY(QString nodeType READ nodeType CONSTANT)

public:
    explicit BtNode(
        const QString & name = "",
        QObject* parent = nullptr
    ):
        QObject(parent),
        m_name(name),
        m_status("IDLE")
    {}

    QString name() const { return m_name; }
    void rename(const QString & new_name);
    int id() const { return m_id; }
    void setId(int id) { m_id = id; emit idChanged(); }
    QList<QObject*> children() const;
    Q_INVOKABLE void addChild(BtNode* child);

    QString status() const { return m_status; }
    void setStatus(const QString & status) { m_status = status; }

    QVariantMap attributes() const { return m_attributes; }
    void setAttributes(const QVariantMap & attributes);

    Q_INVOKABLE BtNode* clone(QObject* parent = nullptr) const;
    bool isRoot() const { return m_isRoot; }
    void setIsRoot(bool isRoot) { m_isRoot = isRoot; }
    QString nodeType() const;

signals:
    void attributesChanged();
    void idChanged();

private:
    QString m_name;     // Descriptive name of the node (maybe not unique)
    int m_id;           // Unique identifier for the node after creation by BtVisualizer
    QString m_status;
    QList<BtNode*> m_children;
    QVariantMap m_attributes;
    bool m_isRoot = false;  // Flag to indicate if this is the root node
};
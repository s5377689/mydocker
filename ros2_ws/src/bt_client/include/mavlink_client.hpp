#pragma once

#include <mavlink/common/mavlink.h>
#include <QObject>
#include <QTcpSocket>
#include <QTimer>
#include <QMap>


class MAVLinkClient : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool connected READ isConnected NOTIFY connectionChanged)
    Q_PROPERTY(QVariantMap parameters READ getParameters NOTIFY parametersChanged)

    // Status properties
    // Q_PROPERTY(bool armed READ armed NOTIFY armedChanged)
    // Q_PROPERTY(QString flightMode READ flightMode NOTIFY flightModeChanged)

public:
    explicit MAVLinkClient(QObject *parent = nullptr);

    // Connection management
    Q_INVOKABLE void connectToSITL();
    Q_INVOKABLE void disconnect();
    bool isConnected() const { return socket_.state() == QTcpSocket::ConnectedState; }

    // Parameter management
    QVariantMap getParameters() const;
    Q_INVOKABLE QStringList getParameterNames() const;
    Q_INVOKABLE QVariant getParameter(const QString & name) const;
    Q_INVOKABLE int getParameterCount() const;
    Q_INVOKABLE bool setParameter(
        const QString & name,
        const QVariant & value
    );
    Q_INVOKABLE void refreshParameter(const QString & name);
    Q_INVOKABLE void resetAllParametersToDefault();
    Q_INVOKABLE void resetConfigParametersToDefault();
    Q_INVOKABLE void resetSensorParametersToDefault(); 
    Q_INVOKABLE void rebootArduPilot();

public slots:
    Q_INVOKABLE void requestParameters();

signals:
    void parametersChanged();
    void connectionChanged();
    void parameterUpdateResult(
        const QString & paramName,
        bool success,
        const QString & message
    );

private slots:
    void onConnected();
    void onDisconnected();
    void onReadyRead();
    void sendHeartbeat();
    void onParametersReceived();
    void onParametersUpdateTimeout();

private:
    void handleMAVLinkMessage(const mavlink_message_t & msg);
    void sendParameterSet(
        const QString & paramName,
        float value
    );
    void sendPreflightStorageCommand(int parameterAction);

    QTcpSocket socket_;
    QTimer heartbeatTimer_;
    QVariantMap paramMap_;

    quint8 systemId_ = 255;
    quint8 componentId_ = 190;
    quint8 targetSystemId_ = 1;
    quint8 targetComponentId_ = 1;

    // Tracking whether parameters are being received
    QTimer paramReceivingTimer_;
    bool paramReceiving_ = false;

    // Tracking for parameter updates
    QMap<QString, float> pendingParameterUpdates_;
    QTimer paramUpdateTimeoutTimer_;

    // Store original parameter values for reset functionality
    QMap<QString, float> originalParameterValues_;
};
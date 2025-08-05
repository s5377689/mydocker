#include "mavlink_client.hpp"

MAVLinkClient::MAVLinkClient(QObject *parent)
    : QObject(parent)
{
    connect(&socket_, &QTcpSocket::connected, this, &MAVLinkClient::onConnected);
    connect(&socket_, &QTcpSocket::disconnected, this, &MAVLinkClient::onDisconnected);
    connect(&socket_, &QTcpSocket::readyRead, this, &MAVLinkClient::onReadyRead);
    connect(&heartbeatTimer_, &QTimer::timeout, this, &MAVLinkClient::sendHeartbeat);
    heartbeatTimer_.setInterval(1000);  // 1Hz

    // Set up parameter batching timer
    paramReceivingTimer_.setSingleShot(true);
    paramReceivingTimer_.setInterval(100);  // 100ms delay
    connect(&paramReceivingTimer_, &QTimer::timeout, this, &MAVLinkClient::onParametersReceived);

    // Set up parameter update timer
    paramUpdateTimeoutTimer_.setSingleShot(true);
    paramUpdateTimeoutTimer_.setInterval(1000);  // 1 seconds delay
    connect(&paramUpdateTimeoutTimer_, &QTimer::timeout, this, &MAVLinkClient::onParametersUpdateTimeout);
}

void MAVLinkClient::rebootArduPilot()
{
    if (!isConnected()) {
        return;
    }

    qDebug() << "Rebooting ArduPilot...";

    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        systemId_,
        componentId_,
        &msg,
        targetSystemId_,
        targetComponentId_,
        MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,  // Confirmation
        1,  // param1: 1 = reboot autopilot, 0 = shutdown
        0,  // param2: 0 = shutdown companion computer
        0,  // param3: Reserved
        0,  // param4: Reserved
        0,  // param5: Reserved
        0,  // param6: Reserved
        0   // param7: Reserved
    );

    QByteArray buffer;
    buffer.resize(MAVLINK_MAX_PACKET_LEN);
    int len = mavlink_msg_to_send_buffer(reinterpret_cast<uint8_t*>(buffer.data()), &msg);
    socket_.write(buffer.left(len));

    qDebug() << "Sent reboot command to ArduPilot";
}

QVariantMap MAVLinkClient::getParameters() const
{
    return paramMap_;
}

QStringList MAVLinkClient::getParameterNames() const
{
    return paramMap_.keys();
}

QVariant MAVLinkClient::getParameter(const QString &name) const
{
    return paramMap_.value(name);
}

int MAVLinkClient::getParameterCount() const
{
    return paramMap_.size();
}

bool MAVLinkClient::setParameter(
    const QString & name,
    const QVariant & value)
{
    if (!isConnected()) {
        emit parameterUpdateResult(
            name,
            false,
            "Not connected to ArduPilot."
        );
        return false;
    }

    if (name.isEmpty()) {
        emit parameterUpdateResult(
            name,
            false,
            "Parameter name cannot be empty."
        );
        return false;
    }

    // Convert QVariant to float
    bool ok;
    float floatValue = value.toFloat(&ok);
    if (!ok) {
        emit parameterUpdateResult(
            name,
            false,
            "Invalid parameter value."
        );
        return false;
    }

    // Check if parameter exists
    if (!paramMap_.contains(name)) {
        emit parameterUpdateResult(
            name,
            false,
            "Parameter does not exist."
        );
        return false;
    }

    // Check if value is different from current value
    float currentValue = paramMap_[name].toFloat();
    if (qAbs(currentValue - floatValue) < 0.0001f) {
        emit parameterUpdateResult(
            name,
            true,
            "Parameter value unchanged."
        );
        return true;
    }

    qDebug() << "Setting parameter" << name << "from" << currentValue << "to" << floatValue;

    // Track the pending update
    pendingParameterUpdates_[name] = floatValue;
    paramUpdateTimeoutTimer_.start();

    // Send the parameter set message
    sendParameterSet(name, floatValue);

    return true;
}

void MAVLinkClient::connectToSITL()
{
    if (socket_.state() == QTcpSocket::ConnectedState) {
        qWarning() << "Already connected to SITL.";
        return;
    }
    socket_.connectToHost("localhost", 5760);
}

void MAVLinkClient::disconnect()
{
    if (socket_.state() == QTcpSocket::UnconnectedState) {
        qWarning() << "Not connected to SITL.";
        return;
    }
    socket_.disconnectFromHost();
    paramMap_.clear();
    heartbeatTimer_.stop();
}

void MAVLinkClient::onParametersReceived()
{
    paramReceiving_ = false;
    emit parametersChanged();
    qDebug() << "Parameters updated, total count:" << paramMap_.size();
}

void MAVLinkClient::onParametersUpdateTimeout()
{
    for (auto it = pendingParameterUpdates_.constBegin(); it != pendingParameterUpdates_.constEnd(); ++it) {
        emit parameterUpdateResult(
            it.key(),
            false,
            "Timeout waiting for parameter update"
        );
    }
    pendingParameterUpdates_.clear();
}

void MAVLinkClient::onConnected()
{
    emit connectionChanged();
    heartbeatTimer_.start();
    qDebug() << "Connected to SITL.";

    QTimer::singleShot(500, this, &MAVLinkClient::requestParameters);
}

void MAVLinkClient::onDisconnected()
{
    paramReceiving_ = false;
    paramMap_.clear();
    pendingParameterUpdates_.clear();
    heartbeatTimer_.stop();
    emit connectionChanged();
    emit parametersChanged();
    qDebug() << "Disconnected from SITL.";
}

void MAVLinkClient::sendHeartbeat()
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        systemId_,
        componentId_,
        &msg,
        MAV_TYPE_GCS,
        MAV_AUTOPILOT_INVALID,
        MAV_MODE_MANUAL_ARMED,
        0,
        MAV_STATE_ACTIVE
    );

    QByteArray buffer;
    buffer.resize(MAVLINK_MAX_PACKET_LEN);
    int len = mavlink_msg_to_send_buffer(reinterpret_cast<uint8_t*>(buffer.data()), &msg);
    socket_.write(buffer.left(len));
}

void MAVLinkClient::sendParameterSet(
    const QString & paramName,
    float value)
{
    mavlink_message_t msg;

    // Convert QString to C-style string
    QByteArray paramBytes = paramName.toUtf8();
    char paramId[17] = {0};  // MAVLink param ID is 16 bytes + null terminator
    strncpy(paramId, paramBytes.constData(), 16);

    mavlink_msg_param_set_pack(
        systemId_,
        componentId_,
        &msg,
        targetSystemId_,
        targetComponentId_,
        paramId,
        value,
        MAV_PARAM_TYPE_REAL32
    );

    // Write the message to the socket
    QByteArray buffer;
    buffer.resize(MAVLINK_MAX_PACKET_LEN);
    int len = mavlink_msg_to_send_buffer(reinterpret_cast<uint8_t*>(buffer.data()), &msg);
    socket_.write(buffer.left(len));

    qDebug() << "Sent parameter set for" << paramName << "=" << value;
}

void MAVLinkClient::refreshParameter(const QString & name)
{
    if (!isConnected())
        return;

    mavlink_message_t msg;

    // Convert QString to C-style string
    QByteArray paramBytes = name.toUtf8();
    char paramId[17] = {0};
    strncpy(paramId, paramBytes.constData(), 16);

    mavlink_msg_param_request_read_pack(
        systemId_,
        componentId_,
        &msg,
        targetSystemId_,
        targetComponentId_,
        paramId,
        -1  // Request by name
    );

    QByteArray buffer;
    buffer.resize(MAVLINK_MAX_PACKET_LEN);
    int len = mavlink_msg_to_send_buffer(reinterpret_cast<uint8_t*>(buffer.data()), &msg);
    socket_.write(buffer.left(len));

    qDebug() << "Requested parameter refresh for: " << name;
}

void MAVLinkClient::requestParameters()
{
    mavlink_message_t msg;
    mavlink_msg_param_request_list_pack(
        systemId_,
        componentId_,
        &msg,
        targetSystemId_,
        targetComponentId_
    );

    QByteArray buffer;
    buffer.resize(MAVLINK_MAX_PACKET_LEN);
    int len = mavlink_msg_to_send_buffer(reinterpret_cast<uint8_t*>(buffer.data()), &msg);
    socket_.write(buffer.left(len));
}

void MAVLinkClient::resetAllParametersToDefault()
{
    if (!isConnected()) {
        emit parameterUpdateResult(
            "ALL",
            false,
            "Not connected to ArduPilot."
        );
        return;
    }

    qDebug() << "Resetting all parameters to default values.";

    // PARAM_RESET_ALL_DEFAULT = 4 (resets everything including operation counters)
    sendPreflightStorageCommand(4);

    emit parameterUpdateResult(
        "ALL",
        true,
        "Reset all parameters command sent. ArduPilot will reboot."
    );
}

void MAVLinkClient::resetConfigParametersToDefault()
{
    if (!isConnected()) {
        emit parameterUpdateResult(
            "CONFIG",
            false,
            "Not connected to ArduPilot."
        );
        return;
    }

    qDebug() << "Resetting user configurable parameters to default values";
    
    // PARAM_RESET_CONFIG_DEFAULT = 2 (resets user config, keeps operation counters)
    sendPreflightStorageCommand(2);
    
    emit parameterUpdateResult(
        "CONFIG",
        true,
        "Reset config parameters command sent. ArduPilot will reboot."
    );
}

void MAVLinkClient::resetSensorParametersToDefault()
{
    if (!isConnected()) {
        emit parameterUpdateResult(
            "SENSORS",
            false,
            "Not connected to ArduPilot."
        );
        return;
    }

    qDebug() << "Resetting sensor calibration parameters to default values";
    
    // PARAM_RESET_SENSOR_DEFAULT = 3 (resets only sensor calibration)
    sendPreflightStorageCommand(3);
    
    emit parameterUpdateResult(
        "SENSORS",
        true,
        "Reset sensor parameters command sent."
    );
}

void MAVLinkClient::sendPreflightStorageCommand(int parameterAction)
{
    mavlink_message_t msg;

    mavlink_msg_command_long_pack(
        systemId_,
        componentId_,
        &msg,
        targetSystemId_,
        targetComponentId_,
        MAV_CMD_PREFLIGHT_STORAGE,  // Command: 245
        0,  // confirmation (0 = first transmission)
        parameterAction,  // param1: Parameter Storage Action
        0,  // param2: Mission Storage (0 = ignore)
        0,  // param3: Logging Rate (0 = ignore)
        0,  // param4: Reserved
        0,  // param5: Empty
        0,  // param6: Empty
        0   // param7: Empty
    );

    QByteArray buffer;
    buffer.resize(MAVLINK_MAX_PACKET_LEN);
    int len = mavlink_msg_to_send_buffer(reinterpret_cast<uint8_t*>(buffer.data()), &msg);
    socket_.write(buffer.left(len));

    qDebug() << "Sent MAV_CMD_PREFLIGHT_STORAGE command with action: " << parameterAction;
}

void MAVLinkClient::onReadyRead()
{
    static mavlink_message_t msg;
    static mavlink_status_t status;

    while (socket_.bytesAvailable())
    {
        uint8_t byte;
        socket_.read(reinterpret_cast<char*>(&byte), 1);
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            handleMAVLinkMessage(msg);
        }
    }
}

void MAVLinkClient::handleMAVLinkMessage(const mavlink_message_t & msg)
{
    switch (msg.msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
            // qDebug() << "Received heartbeat from system" << msg.sysid;
            break;

        case MAVLINK_MSG_ID_COMMAND_ACK:
        {
            mavlink_command_ack_t ack;
            mavlink_msg_command_ack_decode(&msg, &ack);

            if (ack.command == MAV_CMD_PREFLIGHT_STORAGE) {
                if (ack.result == MAV_RESULT_ACCEPTED) {
                    qDebug() << "Parameter reset command accepted.";
                    emit parameterUpdateResult(
                        "RESET",
                        true,
                        "Parameter reset command accepted. ArduPilot will reboot."
                    );
                } else {
                    qDebug() << "Parameter reset command failed with result:" << ack.result;
                    emit parameterUpdateResult(
                        "RESET",
                        false,
                        QString("Parameter reset command failed with result: %1").arg(ack.result)
                    );
                }
            }

            break;
        }

        case MAVLINK_MSG_ID_PARAM_VALUE:
        {
            mavlink_param_value_t param;    
            mavlink_msg_param_value_decode(&msg, &param);

            QString name = QString::fromUtf8(
                param.param_id, strnlen(param.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN)
            );
            float value = param.param_value;

            // Check if this is a response to a PARAM_SET request
            if (pendingParameterUpdates_.contains(name)) {
                float expectedValue = pendingParameterUpdates_[name];
                pendingParameterUpdates_.remove(name);

                if (pendingParameterUpdates_.isEmpty())
                    paramUpdateTimeoutTimer_.stop();

                if (qAbs(value - expectedValue) < 0.0001f) {
                    emit parameterUpdateResult(
                        name,
                        true,
                        "Parameter updated successfully"
                    );
                }
                else {
                    emit parameterUpdateResult(
                        name,
                        false,
                        QString("Parameter update failed: expected %1, got %2").arg(expectedValue).arg(value)
                    );
                }
            }

            // Update our parameter map
            paramMap_[name] = QVariant(param.param_value);

            // Instead of emitting parametersChanged() for every parameter,
            // we batch updates to reduce signal emissions
            if (!paramReceiving_) {
                paramReceiving_ = true;
                qDebug() << "Started receiving parameters...";
            }

            paramReceivingTimer_.start(); // Delay the update if parameters are coming shortly after each other
            break;
        }
    }
}

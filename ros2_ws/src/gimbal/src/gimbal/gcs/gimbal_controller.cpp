#include <fcntl.h>
#include <cstring>
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "gimbal/gcs/gimbal_controller.hpp"

namespace gimbal::gcs
{

GimbalController::GimbalController(
    QObject *parent,
    const std::string & ip,
    uint16_t port
):
    QObject(parent),
    ip_(ip),
    port_(port),
    running_(false),
    connected_(false)
{
   std::cout << "[GimbalController] Initialized with IP: " << ip_ << " and port: " << port_ << std::endl;
}

GimbalController::~GimbalController() {
    stop();
}

void GimbalController::start() {
    if (running_)
        return;

    connected_ = connectToGimbal();
    if (!connected_) {
        std::cerr << "[GimbalController] Failed to start gimbal client." << std::endl;
        return;
    }

    running_ = true;
    heartbeat_thread_ = std::thread(&GimbalController::heartbeatLoop, this, 1);  // 1 hertz
    request_status_thread_ = std::thread(&GimbalController::requestStatusLoop, this, 10);  // 10 hertz
    receive_thread_ = std::thread(&GimbalController::receiveLoop, this);
    send_thread_ = std::thread(&GimbalController::sendLoop, this);
}

void GimbalController::stop() {
    running_ = false;
    send_cv_.notify_all();      // which makes send_thread to exit

    if (heartbeat_thread_.joinable())
        heartbeat_thread_.join();
    if (request_status_thread_.joinable())
        request_status_thread_.join();
    if (receive_thread_.joinable())
        receive_thread_.join();
    if (send_thread_.joinable())
        send_thread_.join();
    // if (action_thread_.joinable())
    //     action_thread_.join();

    if (connected_) {
        close(sockfd_);
        connected_ = false;
        std::cout << "[GimbalController] Connection to gimbal has been closed." << std::endl;
    }

    std::cout << "[GimbalController] All threads have exited." << std::endl;
}

void GimbalController::requestStatusLoop(uint8_t hertz)
{
    std::cout << "[GimbalController] Request status thread started." << std::endl;
    int interval_ms = 1000 / hertz;  // Convert hertz to milliseconds
    auto cmd_pose = buildRequestPoseCommand();
    auto cmd_zoom = buildRequestZoomLevelCommand();
    while (running_) {
        enqueueCommand(cmd_pose);
        enqueueCommand(cmd_zoom);
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }    
    std::cout << "[GimbalController] Request status thread exited." << std::endl;
}

void GimbalController::receiveLoop()
{
    std::cout << "[GimbalController] Receive thread started." << std::endl;

    while (running_) {
        if (!connected_) {
            bool success = reconnect(3);
            if (!success)    
            {
                std::cerr << "[GimbalController] Failed to reconnect to the gimbal." << std::endl;
                break;
            }
            else
                connected_ = true;
        }

        // Receive status
        uint8_t buffer[1024];
        ssize_t received = recv(sockfd_, buffer, sizeof(buffer), MSG_DONTWAIT);

        if (received > 0) {
            stream_buffer_.insert(stream_buffer_.end(), buffer, buffer + received);
            // printf("stream_buffer_ data: ");
            // for (int i = 0; i < stream_buffer_.size(); ++i)
            // {
            //     printf("%02x ", stream_buffer_[i]);
            // }
            // printf("\n");
            parseStreamBuffer();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "[GimbalController] Receive thread exited." << std::endl;
}

void GimbalController::sendLoop()
{
    std::cout << "[GimbalController] Send thread started." << std::endl;
    std::unique_lock<std::mutex> lock(send_queue_mutex_);

    while (running_)
    {
        send_cv_.wait(lock,
            [this] () {
                return !send_queue_.empty() || !running_;
            }
        );  // Wait until there is a command to send or the client is requested to stop

        if (!running_) {
            break;  // Exit if not running anymore
        }

        while (!send_queue_.empty() && connected_) {
            auto cmd = send_queue_.front();
            send_queue_.pop();
            lock.unlock();

            ssize_t sent = send(sockfd_, cmd.data(), cmd.size(), 0);
            if (sent < 0) {
                std::cerr << "[GimbalController] Failed to send command: " << strerror(errno) << std::endl;
                connected_ = false;
                reconnect(3);
            }

            lock.lock();  // Lock again for the next iteration
        }
    }

    std::cout << "[GimbalController] Send thread exited." << std::endl;
}

void GimbalController::heartbeatLoop(uint8_t hertz)
{
    std::cout << "[GimbalController] Heartbeat thread started." << std::endl;
    int interval_ms = 1000 / hertz;  // Convert hertz to milliseconds
    auto cmd = buildHeartbeatCommand();
    while (running_) {
        enqueueCommand(cmd);
        // std::cout << "[GimbalController] Heartbeat command sent." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }
    std::cout << "[GimbalController] Heartbeat thread exited." << std::endl;
}

bool GimbalController::reconnect(int n_retry)
{
    if (connected_) {
        return true;
    }

    std::cout << "[GimbalController] Attempting to reconnect..." << std::endl;

    int retry = 0;
    while (running_ && retry < n_retry) {
        bool connected = connectToGimbal();
        if (connected) {
            std::cout << "[GimbalController] Successfully reconnected to the gimbal." << std::endl;
            return true;
        } else {
            std::cerr << "[GimbalController] Reconnection failed, retrying..." << std::endl;
        }
        ++retry;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return false;  // not running anymore
}

bool GimbalController::connectToGimbal()
{
    sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_ < 0) {
        std::cerr << "[GimbalController] Failed to create socket\n";
        return false;
    }

    // Set to non-blocking mode
    int flags = fcntl(sockfd_, F_GETFL, 0);
    fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK);

    sockaddr_in serv_addr;
    std::memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port_);
    inet_pton(AF_INET, ip_.c_str(), &serv_addr.sin_addr);

    int result = ::connect(sockfd_, (sockaddr*)&serv_addr, sizeof(serv_addr)); 
    if (result < 0 && errno != EINPROGRESS) {
        std::cerr << "[GimbalController] Connection failed immedidately\n";
        return false;
    }

    // Wait for socket to become writable
    fd_set wait_set;
    FD_ZERO(&wait_set);
    FD_SET(sockfd_, &wait_set);
    struct timeval timeout;
    timeout.tv_sec = 3;
    timeout.tv_usec = 0;

    if (select(sockfd_ + 1, nullptr, &wait_set, nullptr, &timeout) <= 0) {
        std::cerr << "[GimbalController] Connection timed out or failed\n";
        close(sockfd_);
        return false;
    }

    // Check for socket errors
    int so_error;
    socklen_t len = sizeof(so_error);
    getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &so_error, &len);
    if (so_error != 0) {
        std::cerr << "[GimbalController] Socket error: " << strerror(so_error) << "\n";
        close(sockfd_);
        return false;
    }

    // Restore to blocking mode if successful
    fcntl(sockfd_, F_SETFL, flags);
    std::cout << "[GimbalController] Successfully connected to the gimbal\n";

    return true;
}

void GimbalController::parseStreamBuffer()
{
    // Check if we have enough data for a complete packet
    while (stream_buffer_.size() >= 10)
    {
        // Find the header
        const std::vector<uint8_t> header = {0x55, 0x66};  // Header bytes
        auto it = std::search(
            stream_buffer_.begin(),
            stream_buffer_.end(),
            header.begin(),
            header.end()
        );

        // If the header is not found (only contains invalid data), clear and return
        if (it == stream_buffer_.end()) {

            // Keep the last byte in case it's part of the two-bytes header {0x55, 0x66}
            if (!stream_buffer_.empty()) {
                stream_buffer_ = {stream_buffer_.back()};
            }

            return;
        }

        size_t header_pos = std::distance(stream_buffer_.begin(), it);

        // Remove any data before the header
        if (header_pos > 0)
        {
            stream_buffer_.erase(stream_buffer_.begin(), it);
        }

        // If the packet is not complete, return
        if (stream_buffer_.size() < 10) {
            // std::cerr << "[GimbalController] Incomplete packet, waiting for more data..." << std::endl;
            return;
        }

        uint8_t cmd_id = stream_buffer_[7];
        
        // Only handle packets with type 0x0d (Gimbal pose) or 0x18 (Gimbal zoom level)
        if (cmd_id != 0x0d && cmd_id != 0x18) {
            // Remove the packets that are not of type 0x0d or 0x18
            stream_buffer_.erase(stream_buffer_.begin(), stream_buffer_.begin() + 10);
            // std::cerr << "[GimbalController] Ignoring packet with CMD_ID: " << std::hex << (int)cmd_id << std::endl;
            continue;
        }

        size_t packet_size;

        switch (cmd_id)
        {
            case 0x0d:
                packet_size = 22;
                break;
            case 0x18:
                packet_size = 12;
                break;
            default:
                std::cerr << "[GimbalController] Unknown command ID: " << std::hex << (int)cmd_id << std::endl;
                stream_buffer_.erase(stream_buffer_.begin(), stream_buffer_.begin() + 10);  // Remove header
                continue;
        }                

        // If the packet is not complete, return
        if (stream_buffer_.size() < packet_size)
            return;

        // Extract the packet
        std::vector<uint8_t> packet(stream_buffer_.begin(), stream_buffer_.begin() + packet_size);

        switch (cmd_id)
        {
            case 0x0d:  // Gimbal pose
                parseAndStorePose(packet);
                break;
            case 0x18:  // Gimbal zoom level
                parseAndStoreZoomLevel(packet);
                break;
            default:
                std::cerr << "[GimbalController] Unknown command ID: " << std::hex << (int)cmd_id << std::endl;
                stream_buffer_.erase(stream_buffer_.begin(), stream_buffer_.begin() + 10);  // Remove header
                continue;
        }
        
        // Remove the processed packet from the stream buffer
        stream_buffer_.erase(stream_buffer_.begin(), stream_buffer_.begin() + packet_size);
    }
}

void GimbalController::enqueueCommand(const std::vector<uint8_t> &cmd)
{
    {
        std::lock_guard<std::mutex> lock(send_queue_mutex_);
        send_queue_.push(cmd);
    }
    send_cv_.notify_all();  // Notify the send thread to send the command
}

void GimbalController::parseAndStorePose(const std::vector<uint8_t> & packet)
{
    gimbal::GimbalStatus gimbal_status;
    // auto now = std::chrono::system_clock::now();
    // std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    // std::tm tm = *std::localtime(&now_c);
    // std::ostringstream oss;
    // oss << std::put_time(&tm, "%F %T");
    // std::cout << "[GimbalController] Received Gimbal status packet. Parsing..."
    //     << oss.str() << std::endl;

    // Start unpacking using little-endian format
    auto get_int16 = [&](int idx) {
        return static_cast<int16_t>(packet[idx] | (packet[idx+1] << 8));
    };

    // packet layout: STX(2), CTRL(1), data_len(2), SEQ(2), CMD_ID(1), DATA(12), CRC16(2)
    // DATA layout: yaw(2), pitch(2), roll(2), 
    //              yaw_velocity(2), pitch_velocity(2), roll_velocity(2)
    gimbal_status.yaw              = get_int16(8) * 0.1f;  // Starts from the 8-th byte
    gimbal_status.pitch            = get_int16(10) * 0.1f;
    gimbal_status.roll             = get_int16(12) * 0.1f;
    gimbal_status.yaw_velocity     = get_int16(14) * 0.1f;  // deg/s
    gimbal_status.pitch_velocity   = get_int16(16) * 0.1f;
    gimbal_status.roll_velocity    = get_int16(18) * 0.1f;  // deg/s

    {
        std::lock_guard<std::mutex> lock(status_mutex_);    
        gimbal_status_.yaw = gimbal_status.yaw;
        gimbal_status_.pitch = gimbal_status.pitch;
        gimbal_status_.roll = gimbal_status.roll;
        gimbal_status_.yaw_velocity = gimbal_status.yaw_velocity;
        gimbal_status_.pitch_velocity = gimbal_status.pitch_velocity;
        gimbal_status_.roll_velocity = gimbal_status.roll_velocity;
    }

    // std::cout << "gimbal_status_.yaw: " << gimbal_status_.yaw << "  "
    //           << "gimbal_status_.pitch: " << gimbal_status_.pitch << "  " << std::endl;
    emit gimbalStatusChanged();
}

void GimbalController::parseAndStoreZoomLevel(const std::vector<uint8_t> & packet)
{
    // auto now = std::chrono::system_clock::now();
    // std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    // std::tm tm = *std::localtime(&now_c);
    // std::ostringstream oss;
    // oss << std::put_time(&tm, "%F %T");
    // std::cout << "[GimbalController] Received Gimbal status packet. Parsing..."
    //     << oss.str() << std::endl;

    // packet layout: STX(2), CTRL(1), data_len(2), SEQ(2), CMD_ID(1), DATA(2), CRC16(2)
    // DATA layout:  zoom_int(1), zoom_float(1)
    float zoom = packet[8] + packet[9] * 0.1f;
    {
        std::lock_guard<std::mutex> lock(status_mutex_);    
        gimbal_status_.zoom = zoom;
    }

    // std::cout << "gimbal_status_.zoom: " << gimbal_status_.zoom << std::endl;
    emit gimbalStatusChanged();
}

void GimbalController::updateBBox(cv::Rect bbox)
{
    std::lock_guard<std::mutex> lock(current_bbox_mutex_);
    current_bbox_ = bbox;
}

Q_INVOKABLE void GimbalController::onImageLeftClicked(int x, int y, int img_width, int img_height)
{
    int dx = x - img_width / 2;
    // int dy = img_height / 2 - y;

    // ====== TEMP ======
    // Current gimbal is setup inversely, so we need to temporarily flip the y-axis
    int dy = y - img_height / 2;

    auto gimbal_status = getGimbalStatus();

    auto [target_yaw, target_pitch] = offsetToAbsoluteAngles(
        dx, dy, gimbal_status.zoom, gimbal_status.yaw, gimbal_status.pitch,
        img_width, img_height, GIMBAL_FOV_X, GIMBAL_FOV_Y
    );

    // ====== TEMP ======
    target_pitch -= 180.0f;
    while (target_pitch < -180.0f) {
        target_pitch += 360.0f;
    }
    // target_pitch = -target_pitch;

    // std::cout << "[GimbalController] Target angles: yaw = " << target_yaw
    //           << ", pitch = " << target_pitch << std::endl;

    // Enqueue the command to rotate the gimbal to the target angles
    rotateTo(target_yaw, target_pitch);
}

}  // namespace gimbal::gcs
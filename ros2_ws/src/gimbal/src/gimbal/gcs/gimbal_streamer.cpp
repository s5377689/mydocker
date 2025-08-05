#include "gimbal/gcs/gimbal_streamer.hpp"

namespace gimbal::gcs
{

GimbalStreamer::GimbalStreamer(
    QObject * parent,
    const std::string & rtsp_url,
    const std::string & topic_name,
    float resize_factor
):
    QObject(parent),
    rtsp_url_(rtsp_url),
    resize_factor_(resize_factor),
    running_(false),
    connected_(false)
{
    std::cout << "[GimbalStreamer] Initialized camera with RTSP URL: " << rtsp_url_ << std::endl;
    
    img_pub_ = rclcpp::create_publisher<sensor_msgs::msg::CompressedImage>(
        rclcpp::Node::make_shared("gimbal_streamer"),
        topic_name,
        10
    );
}

GimbalStreamer::~GimbalStreamer()
{
    stop();
}

void GimbalStreamer::setImageProvider(
    GimbalImageProvider * provider)
{
    img_provider_ = provider;
}

void GimbalStreamer::start()
{
    if (running_)
        return;

    running_ = true;
    capture_thread_ = std::thread(&GimbalStreamer::captureLoop, this);
    std::cout << "[GimbalStreamer] Thread started." << std::endl;
}

void GimbalStreamer::stop()
{
    running_ = false;
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    std::cout << "[GimbalStreamer] All threads have exited." << std::endl;
}

bool GimbalStreamer::getLatestFrame(
    cv::Mat & out_frame)
{
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_frame_.empty()) {
        return false;
    }
    out_frame = latest_frame_.clone();
    return true;
}

void GimbalStreamer::captureLoop()
{
    AVFormatContext * format_ctx = nullptr;
    AVCodecContext * codec_ctx = nullptr;
    AVCodec * codec = nullptr;
    AVFrame * frame = nullptr;
    AVPacket * packet = nullptr;
    struct SwsContext * sws_ctx = nullptr;

    if (avformat_open_input(&format_ctx, rtsp_url_.c_str(), nullptr, nullptr) < 0) {
        std::cerr << "[GimbalStreamer] Failed to open RTSP stream.\n";
        return;
    }

    if (avformat_find_stream_info(format_ctx, nullptr) < 0) {
        std::cerr << "[GimbalStreamer] Failed to find stream info.\n";
        avformat_close_input(&format_ctx);
        return;
    }

    int video_stream_index = -1;
    for (unsigned int i = 0; i < format_ctx->nb_streams; ++i) {
        if (format_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
            video_stream_index = i;
            break;
        }
    }

    if (video_stream_index == -1) {
        std::cerr << "[GimbalStreamer] No video stream found.\n";
        avformat_close_input(&format_ctx);
        return;
    }

    AVCodecParameters * codec_params = format_ctx->streams[video_stream_index]->codecpar;
    codec = avcodec_find_decoder(codec_params->codec_id);
    if (!codec) {
        std::cerr << "[GimbalStreamer] Unsupported codec.\n";
        avformat_close_input(&format_ctx);
        return;
    }

    codec_ctx = avcodec_alloc_context3(codec);
    if (!codec_ctx) {
        std::cerr << "[GimbalStreamer] Could not allocate codec context.\n";
        avformat_close_input(&format_ctx);
        return;
    }

    if (avcodec_parameters_to_context(codec_ctx, codec_params) < 0) {
        std::cerr << "[GimbalStreamer] Could not copy codec parameters to context.\n";
        avcodec_free_context(&codec_ctx);
        avformat_close_input(&format_ctx);
        return;
    }

    if (avcodec_open2(codec_ctx, codec, nullptr) < 0) {
        std::cerr << "[GimbalStreamer] Could not open codec.\n";
        avcodec_free_context(&codec_ctx);
        avformat_close_input(&format_ctx);
        return;
    }

    frame = av_frame_alloc();
    packet = av_packet_alloc();

    int width = codec_ctx->width;
    int height = codec_ctx->height;

    sws_ctx = sws_getContext(
        codec_ctx->width,
        codec_ctx->height,
        codec_ctx->pix_fmt,
        width,
        height,
        AV_PIX_FMT_BGR24,
        SWS_BICUBIC,
        nullptr,
        nullptr,
        nullptr
    );

    uint8_t * dst_data[4];
    int dst_linesize[4];
    av_image_alloc(dst_data, dst_linesize, width, height, AV_PIX_FMT_BGR24, 1);

    connected_ = true;
    emit connectionChanged();
    std::cout << "[GimbalStreamer] Successfully connected to the camera.\n";

    int publish_interval = 1000 / publish_rate_;
    auto last_publish_time = std::chrono::steady_clock::now();

    while (running_)
    {
        if (av_read_frame(format_ctx, packet) < 0) {
            std::cerr << "[GimbalStreamer] Failed to read frame.\n";
            connected_ = false;
            emit connectionChanged();
            break;
        }

        if (packet->stream_index == video_stream_index) {
            if (avcodec_send_packet(codec_ctx, packet) == 0 &&
                avcodec_receive_frame(codec_ctx, frame) == 0) {

                sws_scale(
                    sws_ctx,
                    frame->data,
                    frame->linesize,
                    0,
                    codec_ctx->height,
                    dst_data,
                    dst_linesize
                );

                cv::Mat bgr(height, width, CV_8UC3, dst_data[0], dst_linesize[0]);
                {
                    std::lock_guard<std::mutex> lock(frame_mutex_);
                    latest_frame_ = bgr.clone();

                    // Update the image provider for display in GUI
                    if (img_provider_) {
                        QImage qimg = mat_to_qimage(bgr);
                        img_provider_->update_image(qimg);
                    }
                }

                // Publish the frame for remote viewing
                if (publish_frame_) {
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_publish_time).count();
                    if (elapsed_time >= publish_interval) {
                        // By default,
                        // resize EO image from 1920x1080 to 480x270, jpg quality 75
                        // resize IR image from 640x480 to 320x240, jpg quality 75
                        resize_compress_then_publish(img_pub_, bgr, resize_factor_);
                    }
                }
            }
        }

        av_packet_unref(packet);
    }

    av_freep(&dst_data[0]);
    av_frame_free(&frame);
    av_packet_free(&packet);
    avcodec_free_context(&codec_ctx);
    avformat_close_input(&format_ctx);
    sws_freeContext(sws_ctx);
    connected_ = false;
    emit connectionChanged();
    std::cout << "[GimbalStreamer] Capture thread exited.\n";
}

QImage GimbalStreamer::mat_to_qimage(
    const cv::Mat & mat)
{
    if (mat.type() == CV_8UC3)
        return QImage(
            mat.data,
            mat.cols,
            mat.rows,
            mat.step,
            QImage::Format_BGR888).copy();
    else if (mat.type() == CV_8UC1)
            return QImage(
                mat.data,
                mat.cols,
                mat.rows,
                mat.step,
                QImage::Format_Grayscale8).copy();
    else
        throw std::runtime_error("Unsupported cv::Mat type for conversion to QImage");
}


}  // namespace gimbal::gcs
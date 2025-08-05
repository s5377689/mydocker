#include "ros2_image_provider.hpp"


Ros2ImageProvider::Ros2ImageProvider(Ros2TopicSubscriber* subscriber)
    : QQuickImageProvider(QQuickImageProvider::Image), subscriber_(subscriber)
{}

QImage Ros2ImageProvider::requestImage(const QString &, QSize *size, const QSize &requestedSize)
{
    QImage img = subscriber_->getLatestImage();
    if (img.isNull()) {
        QImage blank(320, 240, QImage::Format_RGB888);
        blank.fill(Qt::black);
        if (size) *size = blank.size();
        return blank;
    }
    QImage out = img;
    if (requestedSize.isValid())
        out = out.scaled(requestedSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    if (size) *size = out.size();
    return out;
}
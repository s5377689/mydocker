#pragma once
#include <QQuickImageProvider>
#include <QImage>
#include <QObject>
#include "ros2_topic_subscriber.hpp"

class Ros2TopicSubscriber;

class Ros2ImageProvider : public QQuickImageProvider
{
public:
    Ros2ImageProvider(Ros2TopicSubscriber* subscriber);

    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;

private:
    Ros2TopicSubscriber* subscriber_;
};
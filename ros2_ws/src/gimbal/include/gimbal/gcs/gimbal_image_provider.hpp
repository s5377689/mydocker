#pragma once

#include <QMutex>
#include <QQuickImageProvider>

namespace gimbal
{

class GimbalImageProvider : public QQuickImageProvider
{
public:
    GimbalImageProvider();
    void update_image(const QImage & img);
    QImage requestImage(
        const QString & id,
        QSize * size,
        const QSize & requested_size
    ) override;
private:
    QImage img_;
    QMutex img_mutex_;
};

}  // namespace gimbal
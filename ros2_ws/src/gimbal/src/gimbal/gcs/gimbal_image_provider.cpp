#include "gimbal/gcs/gimbal_image_provider.hpp"
#include <iostream>

namespace gimbal
{

GimbalImageProvider::GimbalImageProvider()
    : QQuickImageProvider(QQuickImageProvider::Image)
{}

void GimbalImageProvider::update_image(
    const QImage & img)
{
    QMutexLocker locker(&img_mutex_);
    img_ = img;
}

QImage GimbalImageProvider::requestImage(
    const QString & id,
    QSize * size,
    const QSize & requested_size)
{
    Q_UNUSED(id);
    QMutexLocker locker(&img_mutex_);

    if (img_.isNull()) {
        // qDebug() << "Image is null, returning empty image.";
        return QImage();
    }

    if (size) {
        *size = img_.size();
    }

    if (requested_size.isValid() && !img_.isNull()) {
        return img_.scaled(requested_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    }

    return img_;
}

}  // namespace gimbal
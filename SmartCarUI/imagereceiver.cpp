#include "imagereceiver.h"
#include <QDebug>

ImageReceiver::ImageReceiver(QTcpSocket *socket, QObject *parent)
    : QObject(parent)
    , m_socket(socket)
{
    // socket 作为子对象，随 moveToThread 一起迁移
    if (m_socket) {
        m_socket->setParent(this);
        connect(m_socket, &QTcpSocket::readyRead, this, &ImageReceiver::onReadyRead);
        connect(m_socket, &QTcpSocket::disconnected, this, &ImageReceiver::onDisconnected);
    }
}

ImageReceiver::~ImageReceiver()
{
    if (m_socket) {
        m_socket->disconnect();
        if (m_socket->state() != QAbstractSocket::UnconnectedState) {
            m_socket->abort();  // 强制关闭
        }
        m_socket->deleteLater();
        m_socket = nullptr;
    }
}

void ImageReceiver::onReadyRead()
{
    if (!m_socket) return;

    // 把新数据追加到缓冲区
    m_buffer.append(m_socket->readAll());

    // 循环拆帧：可能一次收到多帧
    while (true) {
        if (m_readingHeader) {
            // 需要至少 4 字节才能读长度头
            if (m_buffer.size() < 4) break;

            // 大端 4 字节 → quint32
            m_expectedPayload = static_cast<quint32>(
                (static_cast<unsigned char>(m_buffer[0]) << 24) |
                (static_cast<unsigned char>(m_buffer[1]) << 16) |
                (static_cast<unsigned char>(m_buffer[2]) << 8)  |
                (static_cast<unsigned char>(m_buffer[3]))
            );

            // 移除已读的 4 字节头
            m_buffer.remove(0, 4);
            m_readingHeader = false;

            // 防止恶意超大帧
            if (m_expectedPayload > 5 * 1024 * 1024) {  // 5MB 上限
                qDebug() << "ImageReceiver: frame too large, discarding:" << m_expectedPayload;
                m_expectedPayload = 0;
                m_readingHeader = true;
                m_buffer.clear();
                break;
            }
        }

        // 读 payload 阶段
        if (m_buffer.size() < static_cast<int>(m_expectedPayload)) break;

        // 完整一帧到齐
        QImage img;
        if (img.loadFromData(
                reinterpret_cast<const uchar*>(m_buffer.constData()),
                static_cast<int>(m_expectedPayload),
                "JPEG"))
        {
            emit imageReceived(img);
        } else {
            qDebug() << "ImageReceiver: failed to decode JPEG frame, size =" << m_expectedPayload;
        }

        // 移除已读 payload
        m_buffer.remove(0, static_cast<int>(m_expectedPayload));
        m_expectedPayload = 0;
        m_readingHeader = true;
    }
}

void ImageReceiver::onDisconnected()
{
    qDebug() << "ImageReceiver: client disconnected";
    m_buffer.clear();
    m_expectedPayload = 0;
    m_readingHeader = true;
    emit clientDisconnected();
}

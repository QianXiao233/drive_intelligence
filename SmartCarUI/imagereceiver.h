#ifndef IMAGERECEIVER_H
#define IMAGERECEIVER_H

#include <QObject>
#include <QImage>
#include <QTcpSocket>
#include <QByteArray>

class ImageReceiver : public QObject
{
    Q_OBJECT

public:
    explicit ImageReceiver(QTcpSocket *socket, QObject *parent = nullptr);
    ~ImageReceiver() override;

signals:
    void imageReceived(const QImage &image);
    void clientDisconnected();

private slots:
    void onReadyRead();
    void onDisconnected();

private:
    QTcpSocket *m_socket = nullptr;
    QByteArray m_buffer;        // 累积接收缓冲区
    quint32 m_expectedPayload = 0;  // 当前帧预期 payload 长度
    bool m_readingHeader = true;    // true=正在读4字节头, false=正在读payload
};

#endif // IMAGERECEIVER_H

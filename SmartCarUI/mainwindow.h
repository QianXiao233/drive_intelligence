#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QString>
#include <QProcess>
#include <QQueue>
#include <QByteArray>
#include <QTcpServer>
#include <QTcpSocket>
#include <opencv2/opencv.hpp>
#include <vector>

#include "behaviordialog.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void updateCameraFrame();
    void readMPU6050Data();
    void updateStatus();
    void on_btnVoice_clicked();
    void on_btnAiSet_clicked();

    // ---- Socket: JSON 结果接收 (Python→Qt:9998) ----
    void onJsonNewConnection();
    void onJsonReadyRead();
    void onJsonClientDisconnected();

    // ---- Socket: 抓拍图像接收 (Python→Qt:9997) ----
    void onCaptureNewConnection();
    void onCaptureReadyRead();
    void onCaptureDisconnected();

    // ---- Socket: 帧发送 (Qt→Python:9996) ----
    void connectToPythonFrameServer();

private:
    struct SensorData {
        double ax = 0.0;     // g
        double ay = 0.0;     // g
        double az = 0.0;     // g
        double gx = 0.0;     // deg/s
        double gy = 0.0;     // deg/s
        double gz = 0.0;     // deg/s
        double pitch = 0.0;  // deg
        double roll = 0.0;   // deg
        bool valid = false;
    };

    struct RoadResult {
        QString carStatus;
        QString roadMain;
        QString shakeLevel;
        QString slopeStatus;
        QString roadRisk;
        RiskLevel riskLevel = RiskLevel::Normal;
    };

    void initUiState();
    void initSpeaker();
    void initCamera();
    void reopenCamera();
    void loadFaceCascade();
    bool readMPU6050(SensorData &out);
    RoadResult analyzeRoad(const SensorData &data);
    void updateHardwareStatusBar(const QString &prefix = QString());
    void setStatusText(const QString &text);
    bool checkSpeakerStatus() const;
    bool commandExists(const QString &command) const;
    void speak(const QString &text, bool force = false);
    void playNextSpeech();
    void clearSpeechQueue(bool stopCurrent);
    void speakAndStatus(const QString &speechText, const QString &statusText, bool force = false);
    void updateDriverPresence(bool faceDetected);
    void updateDriverAnalysis(bool faceDetected, const cv::Mat &frame, const std::vector<cv::Rect> &faces);
    void captureAndShow(const cv::Mat &frame, const QString &reason);
    void maybeVoiceAlert(RiskLevel riskLevel, const QString &message);
    QString formatDuration(qint64 seconds) const;
    QString riskName(RiskLevel riskLevel) const;
    void applyDriverRiskStyle(RiskLevel riskLevel);
    void applyRoadRiskStyle(RiskLevel riskLevel);
    void initSocketServers();
    void sendFrameToPython(const cv::Mat &frame);

private:
    Ui::MainWindow *ui;

    QTimer *camTimer = nullptr;
    QTimer *sensorTimer = nullptr;
    QTimer *statTimer = nullptr;

    QProcess *speechProcess = nullptr;
    QQueue<QString> speechQueue;

    cv::VideoCapture cap;
    cv::CascadeClassifier faceCascade;

    int cameraFrameCounter = 0;
    int cameraErrorCounter = 0;
    bool cachedFaceDetected = false;
    std::vector<cv::Rect> cachedFaces;

    bool cameraOk = false;
    bool sensorOk = false;
    bool speakerOk = false;
    bool voiceBusy = false;
    bool speechPlaying = false;
    bool faceCascadeLoaded = false;

    qint64 appStartMs = 0;
    qint64 driverStartMs = 0;
    qint64 lastDriverSeenMs = 0;
    qint64 noDriverStartMs = 0;
    qint64 lastCaptureMs = 0;
    qint64 lastDriverAlertMs = 0;
    qint64 lastRoadAlertMs = 0;

    RiskLevel currentDriverRisk = RiskLevel::Normal;
    RiskLevel currentRoadRisk = RiskLevel::Normal;
    RiskLevel lastDriverAlertRisk = RiskLevel::Normal;
    int voiceSequenceId = 0;

    double lastAccMagnitude = -1.0;

    // --- Socket 通信 ---

    // 通道①: 帧发送 (Qt→Python:9996)
    QTcpSocket *m_frameSocket = nullptr;
    int m_frameSendCounter = 0;        // 每 N 帧发送一次
    bool m_frameSocketConnected = false;

    // 通道②: JSON 结果 (Python→Qt:9998)
    QTcpServer *m_jsonServer = nullptr;
    QByteArray m_jsonBuffer;

    // 通道③: 抓拍图像 (Python→Qt:9997)
    QTcpServer *m_captureServer = nullptr;
    QTcpSocket *m_captureSocket = nullptr;
    QByteArray m_captureBuffer;
    bool m_captureReadingHeader = true;
    quint32 m_captureExpectedPayload = 0;

    // 最近行为
    QString m_lastSocketBehavior;
};

#endif // MAINWINDOW_H

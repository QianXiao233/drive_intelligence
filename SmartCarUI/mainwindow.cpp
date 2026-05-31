#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QApplication>
#include <QCoreApplication>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QMessageBox>
#include <QProcess>
#include <QStandardPaths>
#include <QStringList>
#include <QImage>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPixmap>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

static const char *CAMERA_DEVICE_MAIN = "/dev/video10";
static const char *CAMERA_DEVICE_BACKUP = "/dev/video11";
static const char *MPU6050_I2C_DEVICE = "/dev/i2c-3";
static const double PI_VALUE = 3.14159265358979323846;

static QString shellEscape(const QString &text)
{
    QString escaped = text;
    escaped.replace("'", "'\\''");
    return "'" + escaped + "'";
}


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    appStartMs = QDateTime::currentMSecsSinceEpoch();

    // 开机先把系统主音量打开，避免重启后喇叭被静音。
    initSpeaker();
    speakerOk = checkSpeakerStatus();

    speechProcess = new QProcess(this);
    connect(speechProcess,
            QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this,
            [this](int, QProcess::ExitStatus) {
                speechPlaying = false;
                QTimer::singleShot(250, this, &MainWindow::playNextSpeech);
            });

    initUiState();
    loadFaceCascade();
    initCamera();

    // ---- 启动 Socket 服务端 + 连接 Python ----
    initSocketServers();
    connectToPythonFrameServer();

    // 先读一次六轴传感器，用于启动状态栏显示“正常连接/连接异常”
    SensorData firstData;
    sensorOk = readMPU6050(firstData);
    if (sensorOk) {
        RoadResult firstRoad = analyzeRoad(firstData);
        ui->lblAttitude_3->setText(
            QString("六轴姿态数据\n加速度(g)：X %1  Y %2  Z %3\n角速度(°/s)：X %4  Y %5  Z %6\nPitch：%7°  Roll：%8°")
                .arg(firstData.ax, 0, 'f', 2)
                .arg(firstData.ay, 0, 'f', 2)
                .arg(firstData.az, 0, 'f', 2)
                .arg(firstData.gx, 0, 'f', 1)
                .arg(firstData.gy, 0, 'f', 1)
                .arg(firstData.gz, 0, 'f', 1)
                .arg(firstData.pitch, 0, 'f', 1)
                .arg(firstData.roll, 0, 'f', 1));
        ui->lblCarStatus->setText("当前姿态：" + firstRoad.carStatus);
        ui->lblRoadMain->setText("当前路况：" + firstRoad.roadMain);
        ui->lblShakeLevel->setText("颠簸等级：" + firstRoad.shakeLevel);
        ui->lblSlopeStatus->setText("坡度状态：" + firstRoad.slopeStatus);
        ui->lblRoadRisk->setText("路况风险：" + firstRoad.roadRisk);
        currentRoadRisk = firstRoad.riskLevel;
        applyRoadRiskStyle(firstRoad.riskLevel);
    } else {
        ui->lblAttitude_3->setText("六轴姿态数据\n传感器连接异常：未读取到 MPU6050 数据");
        ui->lblCarStatus->setText("当前姿态：等待六轴传感器数据");
        ui->lblRoadMain->setText("当前路况：未知");
        ui->lblShakeLevel->setText("颠簸等级：未知");
        ui->lblSlopeStatus->setText("坡度状态：未知");
        ui->lblRoadRisk->setText("路况风险：未知");
    }

    updateHardwareStatusBar("系统启动完成");

    camTimer = new QTimer(this);
    connect(camTimer, &QTimer::timeout, this, &MainWindow::updateCameraFrame);
    camTimer->start(40);

    sensorTimer = new QTimer(this);
    connect(sensorTimer, &QTimer::timeout, this, &MainWindow::readMPU6050Data);
    sensorTimer->start(100);

    statTimer = new QTimer(this);
    connect(statTimer, &QTimer::timeout, this, &MainWindow::updateStatus);
    statTimer->start(2000);
}

MainWindow::~MainWindow()
{
    if (cap.isOpened()) {
        cap.release();
    }

    // 关闭帧发送 socket
    if (m_frameSocket) {
        m_frameSocket->disconnect();
        if (m_frameSocket->state() != QAbstractSocket::UnconnectedState) {
            m_frameSocket->abort();
        }
        m_frameSocket->deleteLater();
        m_frameSocket = nullptr;
    }

    clearSpeechQueue(true);
    delete ui;
}

void MainWindow::initUiState()
{
    ui->lblCameraView->setText("正在打开摄像头 /dev/video10 ...");
    ui->lblCameraView->setAlignment(Qt::AlignCenter);
    ui->lblCameraView->setScaledContents(true);

    ui->lblCaptureImg->setText("暂无风险抓拍");
    ui->lblCaptureImg->setAlignment(Qt::AlignCenter);
    ui->lblCaptureImg->setScaledContents(true);

    ui->lblDriverStatus->setText("驾驶员状态：等待摄像头画面");
    ui->lblDriverStatus->setWordWrap(true);
    ui->lblDriverStatus->setGeometry(560, 175, 350, 95);

    ui->lblAttitude_3->setWordWrap(true);
    ui->lblStatusBar->setWordWrap(true);
    ui->lblDriveTime->setText("连续驾驶时长：0小时0分钟0秒");

    ui->lblCameraView->lower();
    ui->lblCaptureImg->raise();
    ui->lblDriverStatus->raise();

    applyDriverRiskStyle(RiskLevel::Normal);
    applyRoadRiskStyle(RiskLevel::Normal);
}

void MainWindow::initCamera()
{
    if (cap.isOpened()) {
        cap.release();
    }

    cameraErrorCounter = 0;
    cameraFrameCounter = 0;

    cameraOk = cap.open(CAMERA_DEVICE_MAIN, cv::CAP_V4L2);

    if (!cameraOk) {
        cameraOk = cap.open(CAMERA_DEVICE_BACKUP, cv::CAP_V4L2);
    }

    if (cameraOk) {
        // 使用 320x240 + YUYV + 低缓存。
        // 上一版使用 MJPG 时，部分 USB 摄像头偶尔会返回空 MJPEG 包，
        // OpenCV 内部 imdecode 会直接抛异常导致程序退出。
        // YUYV 在你的摄像头上同样支持 320x240/30fps，稳定性更好。
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
        cap.set(cv::CAP_PROP_FPS, 30);
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
        cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
        cap.set(cv::CAP_PROP_CONVERT_RGB, 1);

        // 再设置一次尺寸，避免部分 V4L2 后端第一次 set 没生效。
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

        cameraOk = true;
        ui->lblCameraView->setText("摄像头已打开，正在加载画面...");
    } else {
        cameraOk = false;
        ui->lblCameraView->setText("摄像头打开失败\n请确认 /dev/video10 未被占用");
        QMessageBox::warning(this, "摄像头异常", "无法打开 /dev/video10 或 /dev/video11。\n请先执行：fuser -v /dev/video10，确认摄像头没有被其它程序占用。");
    }
}

void MainWindow::reopenCamera()
{
    if (cap.isOpened()) {
        cap.release();
    }

    cameraOk = false;
    ui->lblCameraView->setText("摄像头画面异常，正在重新打开...");
    QTimer::singleShot(500, this, [this]() {
        initCamera();
    });
}

void MainWindow::loadFaceCascade()
{
    QStringList cascadePaths;
    cascadePaths << "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
                 << "/usr/local/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
                 << "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml"
                 << "/usr/local/share/opencv/haarcascades/haarcascade_frontalface_default.xml";

    for (const QString &path : cascadePaths) {
        if (QFile::exists(path) && faceCascade.load(path.toStdString())) {
            faceCascadeLoaded = true;
            qDebug() << "Face cascade loaded:" << path;
            return;
        }
    }

    // 没有人脸模型时，仍然保持演示可运行：摄像头有画面即认为驾驶员在位。
    faceCascadeLoaded = false;
    qDebug() << "Face cascade not found. Driver-present detection will use demo fallback.";
}

void MainWindow::updateCameraFrame()
{
    if (!cap.isOpened()) {
        cameraOk = false;
        return;
    }

    cv::Mat frame;

    try {
        // 直接读取最新帧。上一版 grab/retrieve 连续丢帧时，
        // 某些 MJPG 摄像头会触发 OpenCV imdecode 空缓冲异常。
        if (!cap.read(frame) || frame.empty()) {
            ++cameraErrorCounter;
            cameraOk = false;

            if (cameraErrorCounter >= 10) {
                reopenCamera();
            } else {
                ui->lblCameraView->setText("摄像头暂时无画面，正在等待下一帧...");
            }
            return;
        }
    } catch (const cv::Exception &e) {
        ++cameraErrorCounter;
        cameraOk = false;
        qDebug() << "Camera OpenCV exception:" << e.what();

        if (cameraErrorCounter >= 3) {
            reopenCamera();
        } else {
            ui->lblCameraView->setText("摄像头帧异常，已自动跳过");
        }
        return;
    } catch (...) {
        ++cameraErrorCounter;
        cameraOk = false;

        if (cameraErrorCounter >= 3) {
            reopenCamera();
        } else {
            ui->lblCameraView->setText("摄像头帧异常，已自动跳过");
        }
        return;
    }

    cameraOk = true;
    cameraErrorCounter = 0;
    ++cameraFrameCounter;

    // 每 3 帧发给 Python 模型做识别
    if (m_frameSocketConnected && cameraFrameCounter % 3 == 0) {
        sendFrameToPython(frame);
    }

    std::vector<cv::Rect> faces;
    bool faceDetected = false;

    if (faceCascadeLoaded) {
        // 人脸识别比较耗 CPU，不再每一帧都跑。
        // 每 5 帧识别一次，其余帧复用上次结果，画面会顺很多。
        if (cameraFrameCounter % 5 == 1) {
            try {
                cv::Mat gray;
                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

                cv::Mat smallGray;
                const double scale = 0.5;
                cv::resize(gray, smallGray, cv::Size(), scale, scale, cv::INTER_LINEAR);
                cv::equalizeHist(smallGray, smallGray);

                std::vector<cv::Rect> smallFaces;
                faceCascade.detectMultiScale(
                    smallGray,
                    smallFaces,
                    1.1,
                    3,
                    0,
                    cv::Size(25, 25));

                cachedFaces.clear();
                for (const cv::Rect &r : smallFaces) {
                    cachedFaces.push_back(cv::Rect(
                        static_cast<int>(r.x / scale),
                        static_cast<int>(r.y / scale),
                        static_cast<int>(r.width / scale),
                        static_cast<int>(r.height / scale)));
                }
                cachedFaceDetected = !cachedFaces.empty();
            } catch (const cv::Exception &e) {
                qDebug() << "Face detect exception:" << e.what();
                cachedFaces.clear();
                cachedFaceDetected = false;
            }
        }

        faces = cachedFaces;
        faceDetected = cachedFaceDetected;
    } else {
        // 没有人脸模型时的演示兜底：有稳定摄像头画面即认为驾驶员在位。
        faceDetected = true;
    }

    updateDriverPresence(faceDetected);
    updateDriverAnalysis(faceDetected, frame, faces);

    try {
        cv::Mat showFrame = frame.clone();
        for (const cv::Rect &face : faces) {
            cv::rectangle(showFrame, face, cv::Scalar(0, 255, 0), 2);
        }

        if (showFrame.channels() == 3) {
            cv::cvtColor(showFrame, showFrame, cv::COLOR_BGR2RGB);
            QImage img(showFrame.data, showFrame.cols, showFrame.rows, showFrame.step, QImage::Format_RGB888);
            QPixmap pix = QPixmap::fromImage(img).scaled(ui->lblCameraView->size(), Qt::KeepAspectRatio, Qt::FastTransformation);
            ui->lblCameraView->setPixmap(pix);
        } else if (showFrame.channels() == 1) {
            QImage img(showFrame.data, showFrame.cols, showFrame.rows, showFrame.step, QImage::Format_Grayscale8);
            QPixmap pix = QPixmap::fromImage(img).scaled(ui->lblCameraView->size(), Qt::KeepAspectRatio, Qt::FastTransformation);
            ui->lblCameraView->setPixmap(pix);
        }
    } catch (const cv::Exception &e) {
        qDebug() << "Display frame exception:" << e.what();
    }
}

bool MainWindow::readMPU6050(SensorData &out)
{
    out = SensorData();

    int fd = ::open(MPU6050_I2C_DEVICE, O_RDWR);
    if (fd < 0) {
        return false;
    }

    if (::ioctl(fd, I2C_SLAVE, MPU6050_ADDR) < 0) {
        ::close(fd);
        return false;
    }

    unsigned char wakeBuf[2] = {MPU6050_PWR_MGMT_1, 0x00};
    if (::write(fd, wakeBuf, 2) != 2) {
        ::close(fd);
        return false;
    }

    unsigned char reg = MPU6050_ACCEL_XOUT_H;
    if (::write(fd, &reg, 1) != 1) {
        ::close(fd);
        return false;
    }

    unsigned char data[14] = {0};
    if (::read(fd, data, 14) != 14) {
        ::close(fd);
        return false;
    }

    ::close(fd);

    auto toInt16 = [](unsigned char high, unsigned char low) -> int16_t {
        return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
    };

    int16_t rawAx = toInt16(data[0], data[1]);
    int16_t rawAy = toInt16(data[2], data[3]);
    int16_t rawAz = toInt16(data[4], data[5]);
    int16_t rawGx = toInt16(data[8], data[9]);
    int16_t rawGy = toInt16(data[10], data[11]);
    int16_t rawGz = toInt16(data[12], data[13]);

    out.ax = rawAx / 16384.0;
    out.ay = rawAy / 16384.0;
    out.az = rawAz / 16384.0;
    out.gx = rawGx / 131.0;
    out.gy = rawGy / 131.0;
    out.gz = rawGz / 131.0;

    out.roll = std::atan2(out.ay, out.az) * 180.0 / PI_VALUE;
    out.pitch = std::atan2(-out.ax, std::sqrt(out.ay * out.ay + out.az * out.az)) * 180.0 / PI_VALUE;
    out.valid = true;

    return true;
}

MainWindow::RoadResult MainWindow::analyzeRoad(const SensorData &data)
{
    RoadResult result;

    const double absPitch = std::fabs(data.pitch);
    const double absRoll = std::fabs(data.roll);
    const double gyroMax = std::max(std::fabs(data.gx), std::max(std::fabs(data.gy), std::fabs(data.gz)));
    const double accMagnitude = std::sqrt(data.ax * data.ax + data.ay * data.ay + data.az * data.az);
    const double accDeviation = std::fabs(accMagnitude - 1.0);

    double jerk = 0.0;
    if (lastAccMagnitude > 0.0) {
        jerk = std::fabs(accMagnitude - lastAccMagnitude);
    }
    lastAccMagnitude = accMagnitude;

    const double bumpScore = std::max(accDeviation, jerk);

    if (absRoll > 25.0 || absPitch > 25.0 || gyroMax > 120.0) {
        result.carStatus = "姿态异常/剧烈转向";
    } else if (absRoll > 12.0) {
        result.carStatus = "车辆侧倾";
    } else if (absPitch > 10.0) {
        result.carStatus = "坡道行驶";
    } else if (gyroMax > 45.0) {
        result.carStatus = "转向/变道";
    } else {
        result.carStatus = "平稳行驶";
    }

    if (bumpScore < 0.08) {
        result.roadMain = "平缓路面";
        result.shakeLevel = "无颠簸";
    } else if (bumpScore < 0.18) {
        result.roadMain = "轻微起伏路面";
        result.shakeLevel = "轻微颠簸";
    } else if (bumpScore < 0.35) {
        result.roadMain = "颠簸路面";
        result.shakeLevel = "中等颠簸";
    } else {
        result.roadMain = "剧烈颠簸/疑似坑洼";
        result.shakeLevel = "严重颠簸";
    }

    if (data.pitch > 8.0) {
        result.slopeStatus = "上坡";
    } else if (data.pitch < -8.0) {
        result.slopeStatus = "下坡";
    } else if (data.roll > 12.0) {
        result.slopeStatus = "右侧倾";
    } else if (data.roll < -12.0) {
        result.slopeStatus = "左侧倾";
    } else {
        result.slopeStatus = "平直路面";
    }

    if (bumpScore >= 0.35 || absPitch > 25.0 || absRoll > 25.0 || gyroMax > 120.0) {
        result.riskLevel = RiskLevel::High;
        result.roadRisk = "高风险";
    } else if (bumpScore >= 0.18 || absPitch > 15.0 || absRoll > 15.0 || gyroMax > 80.0) {
        result.riskLevel = RiskLevel::Medium;
        result.roadRisk = "中风险";
    } else if (bumpScore >= 0.08 || absPitch > 8.0 || absRoll > 8.0 || gyroMax > 45.0) {
        result.riskLevel = RiskLevel::Low;
        result.roadRisk = "低风险";
    } else {
        result.riskLevel = RiskLevel::Normal;
        result.roadRisk = "安全";
    }

    return result;
}

void MainWindow::readMPU6050Data()
{
    SensorData data;
    sensorOk = readMPU6050(data);

    if (!sensorOk) {
        ui->lblAttitude_3->setText("六轴姿态数据\n传感器连接异常：未读取到 MPU6050 数据");
        ui->lblCarStatus->setText("当前姿态：等待六轴传感器数据");
        ui->lblRoadMain->setText("当前路况：未知");
        ui->lblShakeLevel->setText("颠簸等级：未知");
        ui->lblSlopeStatus->setText("坡度状态：未知");
        ui->lblRoadRisk->setText("路况风险：未知");
        applyRoadRiskStyle(RiskLevel::Low);
        return;
    }

    RoadResult road = analyzeRoad(data);
    currentRoadRisk = road.riskLevel;

    ui->lblAttitude_3->setText(
        QString("六轴姿态数据\n加速度(g)：X %1  Y %2  Z %3\n角速度(°/s)：X %4  Y %5  Z %6\nPitch：%7°  Roll：%8°")
            .arg(data.ax, 0, 'f', 2)
            .arg(data.ay, 0, 'f', 2)
            .arg(data.az, 0, 'f', 2)
            .arg(data.gx, 0, 'f', 1)
            .arg(data.gy, 0, 'f', 1)
            .arg(data.gz, 0, 'f', 1)
            .arg(data.pitch, 0, 'f', 1)
            .arg(data.roll, 0, 'f', 1));

    ui->lblCarStatus->setText("当前姿态：" + road.carStatus);
    ui->lblRoadMain->setText("当前路况：" + road.roadMain);
    ui->lblShakeLevel->setText("颠簸等级：" + road.shakeLevel);
    ui->lblSlopeStatus->setText("坡度状态：" + road.slopeStatus);
    ui->lblRoadRisk->setText("路况风险：" + road.roadRisk);
    applyRoadRiskStyle(road.riskLevel);

    qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (road.riskLevel >= RiskLevel::Medium && !voiceBusy && now - lastRoadAlertMs > 20000) {
        lastRoadAlertMs = now;
        QString warning = (road.riskLevel == RiskLevel::High) ? "检测到高风险路况，请立即减速慢行" : "检测到路况异常，请注意驾驶安全";
        speak(warning);
        setStatusText("路况提醒：" + warning);
    }
}

void MainWindow::updateDriverPresence(bool faceDetected)
{
    const qint64 now = QDateTime::currentMSecsSinceEpoch();

    if (faceDetected) {
        lastDriverSeenMs = now;
        noDriverStartMs = 0;

        if (driverStartMs == 0) {
            driverStartMs = now;
        }
    } else {
        if (noDriverStartMs == 0) {
            noDriverStartMs = now;
        }

        if (now - noDriverStartMs >= 10000) {
            driverStartMs = 0;
            ui->lblDriveTime->setText("连续驾驶时长：0小时0分钟0秒");
            return;
        }
    }

    if (driverStartMs > 0) {
        qint64 elapsedSec = (now - driverStartMs) / 1000;
        ui->lblDriveTime->setText("连续驾驶时长：" + formatDuration(elapsedSec));
    }
}

void MainWindow::updateDriverAnalysis(bool faceDetected, const cv::Mat &frame, const std::vector<cv::Rect> &faces)
{
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    RiskLevel risk = RiskLevel::Normal;
    QString statusText;
    QString alertText;
    QString captureReason;
    bool needCapture = false;

    if (!faceDetected) {
        qint64 lostSec = noDriverStartMs > 0 ? (now - noDriverStartMs) / 1000 : 0;

        if (lostSec >= 10) {
            risk = RiskLevel::High;
            statusText = "驾驶员状态：未检测到驾驶员\n疲劳程度：未知\n分心情况：驾驶员离开超过10秒\n风险等级：高风险";
            alertText = "未检测到驾驶员，请确认驾驶安全";
            captureReason = "驾驶员离开";
            needCapture = true;
        } else if (lostSec >= 3) {
            risk = RiskLevel::Medium;
            statusText = QString("驾驶员状态：分心疑似\n疲劳程度：轻微\n分心情况：视线离开 %1 秒\n风险等级：中风险").arg(lostSec);
            alertText = "请保持注意力，注意前方道路";
            captureReason = "分心疑似";
            needCapture = true;
        } else {
            risk = RiskLevel::Low;
            statusText = "驾驶员状态：短暂视线偏移\n疲劳程度：正常\n分心情况：轻微注意力偏移\n风险等级：低风险";
        }
    } else {
        // 人脸存在时，保持待命状态，等待识别模型数据
        risk = RiskLevel::Normal;
        statusText = QStringLiteral("驾驶员状态：等待识别模型连接");
    }

    currentDriverRisk = risk;
    ui->lblDriverStatus->setText(statusText);
    applyDriverRiskStyle(risk);

    if (needCapture && now - lastCaptureMs > 10000) {
        captureAndShow(frame, captureReason);
        lastCaptureMs = now;
    }

    if (risk >= RiskLevel::Medium && !alertText.isEmpty()) {
        BehaviorDialog::showAlert(risk, alertText, QStringLiteral("local_driver"));
        maybeVoiceAlert(risk, alertText);
    }
}

void MainWindow::captureAndShow(const cv::Mat &frame, const QString &reason)
{
    if (frame.empty()) {
        return;
    }

    try {
        QDir dir(QCoreApplication::applicationDirPath() + "/captures");
        if (!dir.exists()) {
            dir.mkpath(".");
        }

        QString filePath = dir.filePath("capture_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".jpg");

        cv::Mat saveFrame = frame.clone();
        if (saveFrame.empty()) {
            return;
        }

        std::vector<int> params;
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(90);
        cv::imwrite(filePath.toStdString(), saveFrame, params);

        // 循环清理：最多保留 10 张抓拍
        {
            QStringList filters;
            filters << QStringLiteral("capture_*.jpg");
            QStringList entries = dir.entryList(filters, QDir::Files, QDir::Name);
            while (entries.size() > 10) {
                QFile::remove(dir.filePath(entries.first()));
                entries.removeFirst();
            }
        }

        cv::Mat rgb;
        if (saveFrame.channels() == 3) {
            cv::cvtColor(saveFrame, rgb, cv::COLOR_BGR2RGB);
            QImage img(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888);
            QPixmap pix = QPixmap::fromImage(img).scaled(ui->lblCaptureImg->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
            ui->lblCaptureImg->setPixmap(pix);
        } else if (saveFrame.channels() == 1) {
            QImage img(saveFrame.data, saveFrame.cols, saveFrame.rows, saveFrame.step, QImage::Format_Grayscale8);
            QPixmap pix = QPixmap::fromImage(img).scaled(ui->lblCaptureImg->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
            ui->lblCaptureImg->setPixmap(pix);
        }

        ui->lblCaptureImg->setToolTip(reason + "\n" + filePath);

        if (!voiceBusy) {
            setStatusText("已抓拍风险画面：" + reason + "，图片已保存到 captures 目录");
        }
    } catch (const cv::Exception &e) {
        qDebug() << "Capture image exception:" << e.what();
        if (!voiceBusy) {
            setStatusText("抓拍失败：摄像头帧异常，已自动跳过");
        }
    }
}

void MainWindow::maybeVoiceAlert(RiskLevel riskLevel, const QString &message)
{
    const qint64 now = QDateTime::currentMSecsSinceEpoch();

    // 高风险可以打断语音交互模式
    if (voiceBusy && riskLevel != RiskLevel::High) {
        return;
    }

    // 通用语音冷却：5 秒内不重复播报任何语音
    if (now - lastVoiceAlertMs < 5000) {
        return;
    }

    // 同级别或更低级别 15 秒内不重复
    if (now - lastDriverAlertMs < 15000 && riskLevel <= lastDriverAlertRisk) {
        return;
    }

    lastVoiceAlertMs = now;
    lastDriverAlertMs = now;
    lastDriverAlertRisk = riskLevel;

    if (riskLevel == RiskLevel::High) {
        QApplication::beep();
        // 高风险打断当前正在播放的低/中风险语音
        clearSpeechQueue(true);
    }

    speak(message);
    setStatusText("驾驶员风险提醒：" + message);
}

void MainWindow::on_btnVoice_clicked()
{
    if (voiceBusy) {
        voiceBusy = false;
        ++voiceSequenceId;
        clearSpeechQueue(true);
        speakAndStatus("语音识别已取消", "语音识别已取消", true);
        ui->btnVoice->setEnabled(true);
        ui->btnVoice->setText("语音交互");
        return;
    }

    voiceBusy = true;
    ++voiceSequenceId;
    const int seq = voiceSequenceId;

    // 开始语音交互时，清掉前面未播完的风险提醒，保证不会串音。
    clearSpeechQueue(true);

    ui->btnVoice->setEnabled(true);
    ui->btnVoice->setText("取消语音");

    speakAndStatus("语音开始识别，请说出您的指令", "语音正在识别中...", true);

    QTimer::singleShot(10000, this, [this, seq]() {
        if (!voiceBusy || seq != voiceSequenceId) return;
        speakAndStatus("识别完成，正在为您执行指令", "识别完成，正在为您执行指令...", true);
    });

    QTimer::singleShot(17000, this, [this, seq]() {
        if (!voiceBusy || seq != voiceSequenceId) return;
        speakAndStatus("已为您打开流行音乐", "已为您打开：流行音乐", true);
    });

    QTimer::singleShot(24000, this, [this, seq]() {
        if (!voiceBusy || seq != voiceSequenceId) return;
        speakAndStatus("空调已调节至25摄氏度", "空调已调节至25摄氏度", true);
    });

    QTimer::singleShot(31000, this, [this, seq]() {
        if (!voiceBusy || seq != voiceSequenceId) return;
        speakAndStatus("座椅已调整至中档位置", "座椅已调整至中档位置", true);
    });

    QTimer::singleShot(38000, this, [this, seq]() {
        if (!voiceBusy || seq != voiceSequenceId) return;
        speakAndStatus("车内氛围灯已自动开启", "语音指令执行完成：车内氛围灯已自动开启", true);
        voiceBusy = false;
        ui->btnVoice->setEnabled(true);
        ui->btnVoice->setText("语音交互");
    });
}

void MainWindow::on_btnAiSet_clicked()
{
    QString text =
        "AI 已学习驾驶员习惯，当前为固定模拟结果：\n\n"
        "1. 音乐偏好：上车后优先播放流行音乐。\n"
        "2. 空调习惯：车内温度自动调节至 25 摄氏度。\n"
        "3. 座椅习惯：座椅调整至中档位置。\n"
        "4. 驾驶习惯：转弯前提前降低车速，偏好平稳驾驶。\n"
        "5. 安全策略：检测到分心、疲劳或高风险路况时自动语音提醒。";

    QMessageBox::information(this, "AI 智能设置", text);
    setStatusText("AI智能设置：已加载驾驶员习惯模拟模型");
}

void MainWindow::updateStatus()
{
    // 语音正在播放时，不刷新硬件状态栏，避免把当前提示文字覆盖掉。
    if (!voiceBusy && !speechPlaying) {
        updateHardwareStatusBar();
    }
}

void MainWindow::updateHardwareStatusBar(const QString &prefix)
{
    speakerOk = checkSpeakerStatus();

    QString cameraText = cameraOk ? "摄像头已打开正常工作" : "摄像头未正常打开";
    QString sensorText = sensorOk ? "六轴传感器正常连接" : "六轴传感器连接异常";
    QString speakerText = speakerOk ? "喇叭正常工作" : "未检测到喇叭设备";

    QString text = cameraText + "；" + sensorText + "；" + speakerText;

    if (!prefix.isEmpty()) {
        text = prefix + " | " + text;
    }

    setStatusText(text);
}

void MainWindow::setStatusText(const QString &text)
{
    ui->lblStatusBar->setText("状态栏：" + text);
}

bool MainWindow::checkSpeakerStatus() const
{
    if (QDir("/dev/snd").exists()) {
        return true;
    }

    if (QFile::exists("/proc/asound/cards")) {
        return true;
    }

    if (commandExists("aplay") || commandExists("speaker-test") || commandExists("espeak") || commandExists("espeak-ng") || commandExists("spd-say")) {
        return true;
    }

    return false;
}

bool MainWindow::commandExists(const QString &command) const
{
    return !QStandardPaths::findExecutable(command).isEmpty();
}

void MainWindow::initSpeaker()
{
    // 你已经验证过这条命令能让喇叭恢复发声，所以程序启动时自动执行一次。
    // 如果系统没有 Master 这个控制项，execute 失败也不会影响主程序运行。
    QProcess::execute("amixer", QStringList() << "sset" << "Master" << "90%" << "unmute");
}

void MainWindow::clearSpeechQueue(bool stopCurrent)
{
    speechQueue.clear();

    if (stopCurrent && speechProcess && speechProcess->state() != QProcess::NotRunning) {
        speechProcess->kill();
        speechProcess->waitForFinished(500);
    }

    if (stopCurrent) {
        speechPlaying = false;
    }
}

void MainWindow::speak(const QString &text, bool force)
{
    const QString cleanText = text.trimmed();
    if (cleanText.isEmpty()) {
        return;
    }

    // 语音交互流程执行期间，分心/疲劳/路况风险提醒不插话。
    // 语音按钮自己的提示会用 force=true，因此仍然会正常排队播放。
    if (!force && voiceBusy) {
        return;
    }

    speechQueue.enqueue(cleanText);
    playNextSpeech();
}

void MainWindow::playNextSpeech()
{
    if (!speechProcess) {
        return;
    }

    if (speechPlaying) {
        return;
    }

    if (speechQueue.isEmpty()) {
        return;
    }

    const QString text = speechQueue.dequeue();

    // ---- 语音文本 → 预生成音频文件 映射表 ----
    static const QHash<QString, QString> s_voiceMap = {
        // 语音交互按钮
        {QStringLiteral("\u8BED\u97F3\u5F00\u59CB\u8BC6\u522B\uFF0C\u8BF7\u8BF4\u51FA\u60A8\u7684\u6307\u4EE4"), QStringLiteral("voice_start.wav")},
        {QStringLiteral("\u8F66\u5185\u6C1B\u56F4\u706F\u5DF2\u81EA\u52A8\u5F00\u542F"),             QStringLiteral("voice_complete.wav")},
        {QStringLiteral("\u8BED\u97F3\u8BC6\u522B\u5DF2\u53D6\u6D88"),                                 QStringLiteral("voice_cancel.wav")},
        // 本地驾驶员分析
        {QStringLiteral("\u8BF7\u4FDD\u6301\u6CE8\u610F\u529B\uFF0C\u6CE8\u610F\u524D\u65B9\u9053\u8DEF"),     QStringLiteral("alert_keep_attention.wav")},
        {QStringLiteral("\u672A\u68C0\u6D4B\u5230\u9A7E\u9A76\u5458\uFF0C\u8BF7\u786E\u8BA4\u9A7E\u9A76\u5B89\u5168"), QStringLiteral("alert_no_driver.wav")},
        // 路况
        {QStringLiteral("\u68C0\u6D4B\u5230\u8DEF\u51B5\u5F02\u5E38\uFF0C\u8BF7\u6CE8\u610F\u9A7E\u9A76\u5B89\u5168"),   QStringLiteral("road_medium.wav")},
        {QStringLiteral("\u68C0\u6D4B\u5230\u9AD8\u98CE\u9669\u8DEF\u51B5\uFF0C\u8BF7\u7ACB\u5373\u51CF\u901F\u6162\u884C"), QStringLiteral("road_high.wav")},
        // 行为 - 中风险
        {QStringLiteral("\u8B66\u544A\uFF1A\u5DE6\u4FA7\u6253\u7535\u8BDD"),     QStringLiteral("behavior_talk_left.wav")},
        {QStringLiteral("\u8B66\u544A\uFF1A\u53F3\u4FA7\u6253\u7535\u8BDD"),     QStringLiteral("behavior_talk_right.wav")},
        {QStringLiteral("\u8B66\u544A\uFF1A\u5DE6\u4FA7\u770B\u624B\u673A"),     QStringLiteral("behavior_text_left.wav")},
        {QStringLiteral("\u8B66\u544A\uFF1A\u53F3\u4FA7\u770B\u624B\u673A"),     QStringLiteral("behavior_text_right.wav")},
        {QStringLiteral("\u8B66\u544A\uFF1A\u9A7E\u9A76\u4E2D\u8BF7\u52FF\u5316\u5986"),  QStringLiteral("behavior_makeup.wav")},
        {QStringLiteral("\u8B66\u544A\uFF1A\u9A7E\u9A76\u4E2D\u8BF7\u52FF\u5438\u70DF"),  QStringLiteral("behavior_smoking.wav")},
        {QStringLiteral("\u8B66\u544A\uFF1A\u9A7E\u9A76\u4E2D\u8BF7\u52FF\u996E\u98DF"),  QStringLiteral("behavior_eating.wav")},
        {QStringLiteral("\u8B66\u544A\uFF1A\u64CD\u4F5C\u4E2D\u63A7\u8BBE\u5907"),    QStringLiteral("behavior_radio.wav")},
        {QStringLiteral("\u8B66\u544A\uFF1A\u64CD\u4F5C\u5BFC\u822A\u8BBE\u5907"),    QStringLiteral("behavior_gps.wav")},
        {QStringLiteral("\u8B66\u544A\uFF1A\u5411\u540E\u4F38\u624B\u53D6\u7269"),  QStringLiteral("behavior_reach_behind.wav")},
        // 行为 - 高风险
        {QStringLiteral("\u5371\u9669\uFF1A\u68C0\u6D4B\u5230\u75B2\u52B3\u9A7E\u9A76"),   QStringLiteral("behavior_fatigue.wav")},
        {QStringLiteral("\u5371\u9669\uFF1A\u68C0\u6D4B\u5230\u6253\u54C8\u6B20"),       QStringLiteral("behavior_yawning.wav")},
        {QStringLiteral("\u5371\u9669\uFF1A\u53CC\u624B\u79BB\u5F00\u65B9\u5411\u76D8"), QStringLiteral("behavior_hands_off.wav")},
    };

    // 1) 查找预生成音频文件
    QString wavFile = s_voiceMap.value(text);
    if (!wavFile.isEmpty()) {
        QString voicesDir = QCoreApplication::applicationDirPath()
            + QStringLiteral("/voices/");
        QString wavPath = voicesDir + wavFile;
        if (QFile::exists(wavPath)) {
            speechPlaying = true;
#ifdef Q_OS_WIN
            speechProcess->start(QStringLiteral("powershell"),
                QStringList{QStringLiteral("-c"),
                    QStringLiteral("(New-Object Media.SoundPlayer '%1').PlaySync()")
                        .arg(wavPath)});
#else
            speechProcess->start(QStringLiteral("aplay"),
                QStringList{wavPath});
#endif
            if (!speechProcess->waitForStarted(800)) {
                speechPlaying = false;
                QTimer::singleShot(200, this, &MainWindow::playNextSpeech);
            }
            return;
        }
    }

    // 2) 没有对应 WAV 文件（动态消息）→ 回退 espeak
    QString program = QStandardPaths::findExecutable("espeak");
    if (program.isEmpty()) {
        program = QStandardPaths::findExecutable("espeak-ng");
    }
    if (program.isEmpty()) {
        program = "espeak";
    }

    QStringList args;
    args << "-v" << "zh+f3"
         << "-s" << "125"
         << "-a" << "200"
         << text;

    speechPlaying = true;
    speechProcess->start(program, args);

    if (!speechProcess->waitForStarted(800)) {
        speechPlaying = false;
        QTimer::singleShot(200, this, &MainWindow::playNextSpeech);
    }
}

void MainWindow::speakAndStatus(const QString &speechText, const QString &statusText, bool force)
{
    setStatusText(statusText);
    speak(speechText, force);
}

QString MainWindow::formatDuration(qint64 seconds) const
{
    qint64 h = seconds / 3600;
    qint64 m = (seconds % 3600) / 60;
    qint64 s = seconds % 60;
    return QString("%1小时%2分钟%3秒").arg(h).arg(m).arg(s);
}

QString MainWindow::riskName(RiskLevel riskLevel) const
{
    switch (riskLevel) {
    case RiskLevel::Low:
        return "低风险";
    case RiskLevel::Medium:
        return "中风险";
    case RiskLevel::High:
        return "高风险";
    case RiskLevel::Normal:
    default:
        return "正常";
    }
}

void MainWindow::applyDriverRiskStyle(RiskLevel riskLevel)
{
    QString color;
    QString bg;

    switch (riskLevel) {
    case RiskLevel::Low:
        color = "#FACC15";
        bg = "rgba(80,60,0,150)";
        break;
    case RiskLevel::Medium:
        color = "#FB923C";
        bg = "rgba(90,45,0,170)";
        break;
    case RiskLevel::High:
        color = "#EF4444";
        bg = "rgba(90,0,0,190)";
        break;
    case RiskLevel::Normal:
    default:
        color = "#22C55E";
        bg = "rgba(0,60,25,150)";
        break;
    }

    ui->lblDriverStatus->setStyleSheet(
        QString("color:%1; background-color:%2; padding:6px; border-radius:6px; font-size:12px; font-weight:bold;")
            .arg(color, bg));
}

void MainWindow::applyRoadRiskStyle(RiskLevel riskLevel)
{
    QString color;

    switch (riskLevel) {
    case RiskLevel::Low:
        color = "#FACC15";
        break;
    case RiskLevel::Medium:
        color = "#FB923C";
        break;
    case RiskLevel::High:
        color = "#EF4444";
        break;
    case RiskLevel::Normal:
    default:
        color = "#22C55E";
        break;
    }

    ui->lblRoadRisk->setStyleSheet(QString("color:%1; font-size:14px; font-weight:bold;").arg(color));
}

// ============================================================
// 初始化 Socket 服务端
// ============================================================
void MainWindow::initSocketServers()
{
    // --- 通道②: JSON 结果接收 (Python→Qt:9998) ---
    m_jsonServer = new QTcpServer(this);
    connect(m_jsonServer, &QTcpServer::newConnection, this, &MainWindow::onJsonNewConnection);
    if (!m_jsonServer->listen(QHostAddress::Any, 9998)) {
        qDebug() << "JSON server: failed to listen on port 9998";
    } else {
        qDebug() << "JSON server: listening on port 9998";
    }

    // --- 通道③: 抓拍图像接收 (Python→Qt:9997) ---
    m_captureServer = new QTcpServer(this);
    connect(m_captureServer, &QTcpServer::newConnection, this, &MainWindow::onCaptureNewConnection);
    if (!m_captureServer->listen(QHostAddress::Any, 9997)) {
        qDebug() << "Capture server: failed to listen on port 9997";
    } else {
        qDebug() << "Capture server: listening on port 9997";
    }

    setStatusText(QStringLiteral("Socket 已就绪 (收JSON:9998, 收抓拍:9997)"));
}

// ============================================================
// JSON 客户端连接
// ============================================================
void MainWindow::onJsonNewConnection()
{
    while (m_jsonServer->hasPendingConnections()) {
        QTcpSocket *client = m_jsonServer->nextPendingConnection();
        connect(client, &QTcpSocket::readyRead, this, &MainWindow::onJsonReadyRead);
        connect(client, &QTcpSocket::disconnected, this, &MainWindow::onJsonClientDisconnected);
        qDebug() << "JSON client connected:" << client->peerAddress().toString();
        setStatusText(QStringLiteral("Python 客户端已连接 (JSON)"));
    }
}

// ============================================================
// JSON 数据解析
// ============================================================
void MainWindow::onJsonReadyRead()
{
    auto *client = qobject_cast<QTcpSocket*>(sender());
    if (!client) return;

    m_jsonBuffer.append(client->readAll());

    // 按 \n 分行处理
    while (true) {
        int idx = m_jsonBuffer.indexOf('\n');
        if (idx < 0) break;

        QByteArray line = m_jsonBuffer.left(idx).trimmed();
        m_jsonBuffer.remove(0, idx + 1);

        if (line.isEmpty()) continue;

        // 解析 JSON
        QJsonParseError err;
        QJsonDocument doc = QJsonDocument::fromJson(line, &err);
        if (err.error != QJsonParseError::NoError) {
            qDebug() << "JSON parse error:" << err.errorString();
            continue;
        }

        QJsonObject obj = doc.object();
        QString behavior = obj.value(QStringLiteral("behavior")).toString();
        QString riskLevelStr = obj.value(QStringLiteral("risk_level")).toString();

        // 确定风险等级：优先使用 risk_level 字段
        RiskLevel risk;
        if (!riskLevelStr.isEmpty()) {
            if (riskLevelStr == QStringLiteral("high"))      risk = RiskLevel::High;
            else if (riskLevelStr == QStringLiteral("medium")) risk = RiskLevel::Medium;
            else if (riskLevelStr == QStringLiteral("low"))   risk = RiskLevel::Low;
            else                                               risk = RiskLevel::Normal;
        } else {
            risk = BehaviorDialog::behaviorToRisk(behavior);
        }

        // 获取显示文字
        QString text = BehaviorDialog::behaviorToText(behavior);
        if (text.isEmpty()) {
            text = obj.value(QStringLiteral("message")).toString();
        }

        // ---- 根据风险等级处理 ----

        // 1) 更新驾驶员状态文字
        QString statusText = QStringLiteral("驾驶员状态：%1\n风险等级：%2")
            .arg(behavior,
                 riskLevelStr.isEmpty() ? text : riskLevelStr);
        ui->lblDriverStatus->setText(statusText);
        applyDriverRiskStyle(risk);

        // 2) 记录最近行为
        m_lastSocketBehavior = behavior;

        // 3) 弹窗 + 语音（只有 Medium/High 才触发）
        bool popped = BehaviorDialog::showAlert(risk, text, behavior);
        if (popped) {
            maybeVoiceAlert(risk, text);
            setStatusText(QStringLiteral("行为警报：%1").arg(text));
        }
    }
}

// ============================================================
// JSON 客户端断开
// ============================================================
void MainWindow::onJsonClientDisconnected()
{
    auto *client = qobject_cast<QTcpSocket*>(sender());
    if (client) {
        qDebug() << "JSON client disconnected";
        client->deleteLater();
    }
    setStatusText(QStringLiteral("Python 客户端已断开 (JSON)"));

    // 重置驾驶员状态
    ui->lblDriverStatus->setText(QStringLiteral("驾驶员状态：等待识别模型连接"));
    applyDriverRiskStyle(RiskLevel::Normal);
}

// ============================================================
// 通道①: 连接 Python 模型服务端 (Qt→Python:9996)
// ============================================================
void MainWindow::connectToPythonFrameServer()
{
    if (m_frameSocket) {
        m_frameSocket->disconnect();
        m_frameSocket->deleteLater();
        m_frameSocket = nullptr;
    }

    m_frameSocket = new QTcpSocket(this);
    m_frameSocket->connectToHost(QHostAddress::LocalHost, 9996);

    connect(m_frameSocket, &QTcpSocket::connected, this, [this]() {
        m_frameSocketConnected = true;
        qDebug() << "Frame socket connected to Python:9996";
        setStatusText(QStringLiteral("已连接 Python 模型服务端 (9996)"));
    });

    connect(m_frameSocket, &QTcpSocket::disconnected, this, [this]() {
        m_frameSocketConnected = false;
        qDebug() << "Frame socket disconnected from Python:9996";
        setStatusText(QStringLiteral("Python 模型服务端已断开 (9996)"));

        // 5 秒后重连
        QTimer::singleShot(5000, this, &MainWindow::connectToPythonFrameServer);
    });

    connect(m_frameSocket, static_cast<void(QAbstractSocket::*)(QAbstractSocket::SocketError)>(&QAbstractSocket::error),
            this, [this](QAbstractSocket::SocketError) {
        qDebug() << "Frame socket error, will retry in 5s";
        // 错误时也自动重连
        QTimer::singleShot(5000, this, &MainWindow::connectToPythonFrameServer);
    });
}

// ============================================================
// 通道①: 发送摄像头帧给 Python 模型
// 格式: [4B BE: payloadLen][JPEG 数据]
// ============================================================
void MainWindow::sendFrameToPython(const cv::Mat &frame)
{
    if (!m_frameSocketConnected || !m_frameSocket || frame.empty()) return;

    try {
        std::vector<int> params;
        params.push_back(cv::IMWRITE_JPEG_QUALITY);
        params.push_back(60);  // 60% 质量，平衡速度和带宽

        std::vector<uchar> jpegData;
        cv::imencode(".jpg", frame, jpegData, params);

        if (jpegData.empty()) return;

        // 4 字节大端长度前缀 + JPEG 数据
        quint32 payloadLen = static_cast<quint32>(jpegData.size());
        QByteArray packet;
        packet.append(static_cast<char>((payloadLen >> 24) & 0xFF));
        packet.append(static_cast<char>((payloadLen >> 16) & 0xFF));
        packet.append(static_cast<char>((payloadLen >> 8) & 0xFF));
        packet.append(static_cast<char>(payloadLen & 0xFF));
        packet.append(reinterpret_cast<const char*>(jpegData.data()),
                      static_cast<int>(jpegData.size()));

        m_frameSocket->write(packet);
    } catch (const cv::Exception &e) {
        qDebug() << "sendFrameToPython exception:" << e.what();
    }
}

// ============================================================
// 通道③: 抓拍图像客户端连接 (Python→Qt:9997)
// ============================================================
void MainWindow::onCaptureNewConnection()
{
    while (m_captureServer->hasPendingConnections()) {
        // 只保留最新一个连接
        if (m_captureSocket) {
            m_captureSocket->disconnect();
            m_captureSocket->deleteLater();
            m_captureSocket = nullptr;
        }

        m_captureSocket = m_captureServer->nextPendingConnection();
        m_captureBuffer.clear();
        m_captureReadingHeader = true;
        m_captureExpectedPayload = 0;

        connect(m_captureSocket, &QTcpSocket::readyRead,
                this, &MainWindow::onCaptureReadyRead);
        connect(m_captureSocket, &QTcpSocket::disconnected,
                this, &MainWindow::onCaptureDisconnected);

        qDebug() << "Capture client connected:" << m_captureSocket->peerAddress().toString();
        setStatusText(QStringLiteral("抓拍通道已连接 (9997)"));
    }
}

// ============================================================
// 通道③: 接收抓拍 JPEG → 显示在 lblCaptureImg
// ============================================================
void MainWindow::onCaptureReadyRead()
{
    if (!m_captureSocket) return;

    m_captureBuffer.append(m_captureSocket->readAll());

    // 循环拆帧：可能一次收到多帧
    while (true) {
        if (m_captureReadingHeader) {
            if (m_captureBuffer.size() < 4) break;

            m_captureExpectedPayload =
                (static_cast<quint32>(static_cast<unsigned char>(m_captureBuffer[0])) << 24) |
                (static_cast<quint32>(static_cast<unsigned char>(m_captureBuffer[1])) << 16) |
                (static_cast<quint32>(static_cast<unsigned char>(m_captureBuffer[2])) << 8)  |
                static_cast<quint32>(static_cast<unsigned char>(m_captureBuffer[3]));

            m_captureBuffer.remove(0, 4);
            m_captureReadingHeader = false;

            if (m_captureExpectedPayload > 5 * 1024 * 1024) {
                qDebug() << "Capture frame too large:" << m_captureExpectedPayload;
                m_captureBuffer.clear();
                m_captureReadingHeader = true;
                m_captureExpectedPayload = 0;
                break;
            }
        }

        if (m_captureBuffer.size() < static_cast<int>(m_captureExpectedPayload)) break;

        // 完整 JPEG 帧到齐 → 显示到抓拍画面
        QImage img;
        if (img.loadFromData(
                reinterpret_cast<const uchar*>(m_captureBuffer.constData()),
                static_cast<int>(m_captureExpectedPayload), "JPEG"))
        {
            QPixmap pix = QPixmap::fromImage(img).scaled(
                ui->lblCaptureImg->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);
            ui->lblCaptureImg->setPixmap(pix);
            ui->lblCaptureImg->setToolTip(QStringLiteral("模型抓拍 - %1")
                .arg(QDateTime::currentDateTime().toString("HH:mm:ss")));
        }

        m_captureBuffer.remove(0, static_cast<int>(m_captureExpectedPayload));
        m_captureExpectedPayload = 0;
        m_captureReadingHeader = true;
    }
}

// ============================================================
// 通道③: 抓拍客户端断开
// ============================================================
void MainWindow::onCaptureDisconnected()
{
    qDebug() << "Capture client disconnected";
    if (m_captureSocket) {
        m_captureSocket->deleteLater();
        m_captureSocket = nullptr;
    }
    m_captureBuffer.clear();
    m_captureReadingHeader = true;
    m_captureExpectedPayload = 0;
}

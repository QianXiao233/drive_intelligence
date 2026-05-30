#ifndef BEHAVIORDIALOG_H
#define BEHAVIORDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QString>

enum class RiskLevel {
    Normal = 0,
    Low = 1,
    Medium = 2,
    High = 3
};

// 7 类车辆姿态事件
enum class PostureEvent {
    Stable = 0,          // 姿态平稳
    SlightBump = 1,      // 轻微颠簸
    SuddenBrake = 2,     // 急刹车
    SharpTurn = 3,       // 急转弯
    SevereBump = 4,      // 剧烈颠簸
    AbnormalTilt = 5,    // 异常侧倾
    SevereImpact = 6     // 剧烈冲击
};

class BehaviorDialog : public QDialog
{
    Q_OBJECT

public:
    explicit BehaviorDialog(RiskLevel riskLevel, const QString &message, QWidget *parent = nullptr);
    ~BehaviorDialog() override = default;

    // 统一入口：Normal/Low 不弹窗返回 false；Medium/High 弹窗返回 true
    static bool showAlert(RiskLevel riskLevel, const QString &message,
                          const QString &behaviorKey = QString());

    // 行为映射工具
    static RiskLevel behaviorToRisk(const QString &behavior);
    static QString behaviorToText(const QString &behavior);

    // 姿态事件映射工具
    static RiskLevel postureToRisk(PostureEvent event);
    static QString postureToText(PostureEvent event);
    static QString postureToVoice(PostureEvent event);

    // 联合风险判定：行为 + 姿态，同时异常时升级为 High
    static RiskLevel combinedRisk(RiskLevel driverRisk, RiskLevel postureRisk);

private:
    static bool checkCooldown(const QString &behaviorKey);
    static void speak(const QString &text);
    static void beep();

    QLabel *m_label = nullptr;
};

#endif // BEHAVIORDIALOG_H

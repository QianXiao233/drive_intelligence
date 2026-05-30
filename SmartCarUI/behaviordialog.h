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

private:
    static bool checkCooldown(const QString &behaviorKey);
    static void speak(const QString &text);
    static void beep();

    QLabel *m_label = nullptr;
};

#endif // BEHAVIORDIALOG_H

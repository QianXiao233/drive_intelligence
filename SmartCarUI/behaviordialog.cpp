#include "behaviordialog.h"

#include <QApplication>
#include <QDateTime>
#include <QDebug>
#include <QMap>
#include <QProcess>
#include <QScreen>
#include <QTimer>
#include <QVBoxLayout>

#include <cstdio>

// ============================================================
// behavior → RiskLevel 映射表（22 类识别码）
// 同时支持 C1~C22 编码、英文名、短名，全部转小写匹配
// ============================================================
static const QMap<QString, RiskLevel> BEHAVIOR_RISK_MAP = {
    // --- 正常 ---
    {QStringLiteral("c1"),              RiskLevel::Normal},
    {QStringLiteral("drive_safe"),      RiskLevel::Normal},
    {QStringLiteral("safe_driving"),    RiskLevel::Normal},
    // --- 低风险（仅改文字，不弹窗）---
    {QStringLiteral("c8"),              RiskLevel::Medium},
    {QStringLiteral("make_up"),         RiskLevel::Medium},
    {QStringLiteral("c9"),              RiskLevel::Low},
    {QStringLiteral("look_left"),       RiskLevel::Low},
    {QStringLiteral("c10"),             RiskLevel::Low},
    {QStringLiteral("look_right"),      RiskLevel::Low},
    {QStringLiteral("c11"),             RiskLevel::Low},
    {QStringLiteral("look_up"),         RiskLevel::Low},
    {QStringLiteral("c12"),             RiskLevel::Low},
    {QStringLiteral("look_down"),       RiskLevel::Low},
    {QStringLiteral("c13"),             RiskLevel::Medium},
    {QStringLiteral("smoke_left"),      RiskLevel::Medium},
    {QStringLiteral("c14"),             RiskLevel::Medium},
    {QStringLiteral("smoke_right"),     RiskLevel::Medium},
    {QStringLiteral("c15"),             RiskLevel::Low},
    {QStringLiteral("smoke_mouth"),     RiskLevel::Low},
    {QStringLiteral("c22"),             RiskLevel::Low},
    {QStringLiteral("talk_to_passenger"), RiskLevel::Low},
    {QStringLiteral("distracted"),      RiskLevel::Low},
    {QStringLiteral("looking_away"),    RiskLevel::Low},
    // --- 中风险（弹窗 + 语音）---
    {QStringLiteral("c4"),              RiskLevel::Medium},
    {QStringLiteral("talk_left"),       RiskLevel::Medium},
    {QStringLiteral("c5"),              RiskLevel::Medium},
    {QStringLiteral("talk_right"),      RiskLevel::Medium},
    {QStringLiteral("c6"),              RiskLevel::Medium},
    {QStringLiteral("text_left"),       RiskLevel::Medium},
    {QStringLiteral("c7"),              RiskLevel::Medium},
    {QStringLiteral("text_right"),      RiskLevel::Medium},
    {QStringLiteral("c16"),             RiskLevel::Medium},
    {QStringLiteral("eat_left"),        RiskLevel::Medium},
    {QStringLiteral("c17"),             RiskLevel::Medium},
    {QStringLiteral("eat_right"),       RiskLevel::Medium},
    {QStringLiteral("c18"),             RiskLevel::Medium},
    {QStringLiteral("operate_radio"),   RiskLevel::Medium},
    {QStringLiteral("c19"),             RiskLevel::Medium},
    {QStringLiteral("operate_gps"),     RiskLevel::Medium},
    {QStringLiteral("c20"),             RiskLevel::Medium},
    {QStringLiteral("reach_behind"),    RiskLevel::Medium},
    {QStringLiteral("phone_using"),     RiskLevel::Medium},
    {QStringLiteral("phone_call"),      RiskLevel::Medium},
    {QStringLiteral("drinking"),        RiskLevel::Medium},
    {QStringLiteral("eating"),          RiskLevel::Medium},
    // --- 高风险（弹窗 + 语音 + 蜂鸣）---
    {QStringLiteral("c2"),              RiskLevel::High},
    {QStringLiteral("sleep"),           RiskLevel::High},
    {QStringLiteral("c3"),              RiskLevel::High},
    {QStringLiteral("yawning"),         RiskLevel::High},
    {QStringLiteral("c21"),             RiskLevel::High},
    {QStringLiteral("leave_steering_wheel"), RiskLevel::High},
    {QStringLiteral("eyes_closed"),     RiskLevel::High},
    {QStringLiteral("no_driver"),       RiskLevel::High},
    {QStringLiteral("no_face"),         RiskLevel::High},
    {QStringLiteral("tired"),           RiskLevel::High},
};

// ============================================================
// behavior → 弹窗/状态文字 映射表
// ============================================================
static const QMap<QString, QString> BEHAVIOR_TEXT_MAP = {
    // --- C1 正常 ---
    {QStringLiteral("c1"),              QStringLiteral("驾驶状态：正常")},
    {QStringLiteral("drive_safe"),      QStringLiteral("驾驶状态：正常")},
    {QStringLiteral("safe_driving"),    QStringLiteral("驾驶状态：正常")},
    // --- C2-C3 疲劳 ---
    {QStringLiteral("c2"),              QStringLiteral("危险：检测到疲劳驾驶")},
    {QStringLiteral("sleep"),           QStringLiteral("危险：检测到疲劳驾驶")},
    {QStringLiteral("c3"),              QStringLiteral("危险：检测到打哈欠")},
    {QStringLiteral("yawning"),         QStringLiteral("危险：检测到打哈欠")},
    // --- C4-C7 手机 ---
    {QStringLiteral("c4"),              QStringLiteral("警告：左侧打电话")},
    {QStringLiteral("talk_left"),       QStringLiteral("警告：左侧打电话")},
    {QStringLiteral("c5"),              QStringLiteral("警告：右侧打电话")},
    {QStringLiteral("talk_right"),      QStringLiteral("警告：右侧打电话")},
    {QStringLiteral("c6"),              QStringLiteral("警告：左侧看手机")},
    {QStringLiteral("text_left"),       QStringLiteral("警告：左侧看手机")},
    {QStringLiteral("c7"),              QStringLiteral("警告：右侧看手机")},
    {QStringLiteral("text_right"),      QStringLiteral("警告：右侧看手机")},
    // --- C8 化妆（中风险）---
    {QStringLiteral("c8"),              QStringLiteral("警告：驾驶中请勿化妆")},
    {QStringLiteral("make_up"),         QStringLiteral("警告：驾驶中请勿化妆")},
    // --- C9-C12 视线偏移 ---
    {QStringLiteral("c9"),              QStringLiteral("注意：视线向左偏移")},
    {QStringLiteral("look_left"),       QStringLiteral("注意：视线向左偏移")},
    {QStringLiteral("c10"),             QStringLiteral("注意：视线向右偏移")},
    {QStringLiteral("look_right"),      QStringLiteral("注意：视线向右偏移")},
    {QStringLiteral("c11"),             QStringLiteral("注意：视线向上偏移")},
    {QStringLiteral("look_up"),         QStringLiteral("注意：视线向上偏移")},
    {QStringLiteral("c12"),             QStringLiteral("注意：视线向下偏移")},
    {QStringLiteral("look_down"),       QStringLiteral("注意：视线向下偏移")},
    // --- C13-C14 吸烟（中风险）---
    {QStringLiteral("c13"),             QStringLiteral("警告：驾驶中请勿吸烟")},
    {QStringLiteral("smoke_left"),      QStringLiteral("警告：驾驶中请勿吸烟")},
    {QStringLiteral("c14"),             QStringLiteral("警告：驾驶中请勿吸烟")},
    {QStringLiteral("smoke_right"),     QStringLiteral("警告：驾驶中请勿吸烟")},
    // --- C15 嘴叼香烟（低风险）---
    {QStringLiteral("c15"),             QStringLiteral("提示：驾驶中请勿吸烟")},
    {QStringLiteral("smoke_mouth"),     QStringLiteral("提示：驾驶中请勿吸烟")},
    // --- C16-C17 饮食 ---
    {QStringLiteral("c16"),             QStringLiteral("警告：驾驶中请勿饮食")},
    {QStringLiteral("eat_left"),        QStringLiteral("警告：驾驶中请勿饮食")},
    {QStringLiteral("c17"),             QStringLiteral("警告：驾驶中请勿饮食")},
    {QStringLiteral("eat_right"),       QStringLiteral("警告：驾驶中请勿饮食")},
    // --- C18-C19 操作设备 ---
    {QStringLiteral("c18"),             QStringLiteral("警告：操作中控设备")},
    {QStringLiteral("operate_radio"),   QStringLiteral("警告：操作中控设备")},
    {QStringLiteral("c19"),             QStringLiteral("警告：操作导航设备")},
    {QStringLiteral("operate_gps"),     QStringLiteral("警告：操作导航设备")},
    // --- C20 伸手 ---
    {QStringLiteral("c20"),             QStringLiteral("警告：向后伸手取物")},
    {QStringLiteral("reach_behind"),    QStringLiteral("警告：向后伸手取物")},
    // --- C21 离方向盘 ---
    {QStringLiteral("c21"),             QStringLiteral("危险：双手离开方向盘")},
    {QStringLiteral("leave_steering_wheel"), QStringLiteral("危险：双手离开方向盘")},
    // --- C22 交谈 ---
    {QStringLiteral("c22"),             QStringLiteral("提示：请勿与乘客交谈")},
    {QStringLiteral("talk_to_passenger"), QStringLiteral("提示：请勿与乘客交谈")},
};


// ============================================================
// BehaviorDialog 构造
// ============================================================
BehaviorDialog::BehaviorDialog(RiskLevel riskLevel, const QString &message, QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle(QStringLiteral("\u884C\u4E3A\u8B66\u62A5"));
    setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
    setAttribute(Qt::WA_DeleteOnClose);
    setAttribute(Qt::WA_ShowWithoutActivating);
    setModal(false);
    setFixedSize(380, 160);

    QString bgColor, borderColor, textColor;
    switch (riskLevel) {
    case RiskLevel::High:
        bgColor  = QStringLiteral("#1A0A0A");
        borderColor = QStringLiteral("#f44336");
        textColor   = QStringLiteral("#f44336");
        break;
    case RiskLevel::Medium:
    default:
        bgColor  = QStringLiteral("#1A1A0A");
        borderColor = QStringLiteral("#ffc107");
        textColor   = QStringLiteral("#ffc107");
        break;
    }

    setStyleSheet(QStringLiteral(
        "BehaviorDialog {"
        "  background-color: %1;"
        "  border: 3px solid %2;"
        "  border-radius: 12px;"
        "}"
    ).arg(bgColor, borderColor));

    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(20, 20, 20, 20);

    m_label = new QLabel(message, this);
    m_label->setStyleSheet(QStringLiteral(
        "color: %1; font-size: 20px; font-weight: bold; padding: 10px;"
    ).arg(textColor));
    m_label->setAlignment(Qt::AlignCenter);
    m_label->setWordWrap(true);
    layout->addWidget(m_label);

    // 4 秒后自动关闭
    QTimer::singleShot(4000, this, &QDialog::close);
}

// ============================================================
// showAlert — 统一入口（仅弹窗，不处理语音）
// Normal / Low  → 不弹窗，返回 false（调用方自行改文字颜色）
// Medium / High → 弹非模态弹窗，返回 true
// 语音由调用方（MainWindow::maybeVoiceAlert）统一管理
// ============================================================
bool BehaviorDialog::showAlert(RiskLevel riskLevel, const QString &message,
                                const QString &behaviorKey)
{
    // Normal / Low → 不弹窗
    if (riskLevel <= RiskLevel::Low) {
        return false;
    }

    // 冷却检查（同一 behavior 5 秒内不重复）
    if (!behaviorKey.isEmpty() && !checkCooldown(behaviorKey)) {
        return false;
    }

    // 创建非模态弹窗
    auto *dlg = new BehaviorDialog(riskLevel, message, nullptr);
    dlg->show();

    // 屏幕居中
    if (auto *screen = QApplication::primaryScreen()) {
        dlg->move(screen->geometry().center() - dlg->rect().center());
    }

    return true;
}

// ============================================================
// behavior → RiskLevel
// ============================================================
RiskLevel BehaviorDialog::behaviorToRisk(const QString &behavior)
{
    return BEHAVIOR_RISK_MAP.value(behavior.toLower(), RiskLevel::Normal);
}

// ============================================================
// behavior → 显示文字
// ============================================================
QString BehaviorDialog::behaviorToText(const QString &behavior)
{
    return BEHAVIOR_TEXT_MAP.value(behavior.toLower(),
        QStringLiteral("\u672A\u77E5\u884C\u4E3A"));
}

// ============================================================
// 冷却检查（静态，全局共享）
// ============================================================
bool BehaviorDialog::checkCooldown(const QString &behaviorKey)
{
    static QMap<QString, qint64> cooldownMap;
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    const qint64 last = cooldownMap.value(behaviorKey, 0);

    if (now - last < 5000) {
        return false;   // 5 秒内不重复
    }

    cooldownMap[behaviorKey] = now;
    return true;
}

// ============================================================
// espeak 语音（Linux 专用）
// ============================================================
void BehaviorDialog::speak(const QString &text)
{
#ifdef Q_OS_LINUX
    QString program = QStringLiteral("espeak");
    if (QProcess::execute(program, {QStringLiteral("--version")}) != 0) {
        program = QStringLiteral("espeak-ng");
    }
    QProcess::startDetached(program, {
        QStringLiteral("-v"), QStringLiteral("zh+f3"),
        QStringLiteral("-s"), QStringLiteral("150"),
        text
    });
#else
    Q_UNUSED(text);
#endif
}

// ============================================================
// 蜂鸣
// ============================================================
void BehaviorDialog::beep()
{
    std::fputc('\a', stderr);
}

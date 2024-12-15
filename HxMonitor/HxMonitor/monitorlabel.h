#ifndef MONITORLABEL_H
#define MONITORLABEL_H

#include <QWidget>
#include <QLabel>
#include <QPainter>
#include <QString>

struct MonitorLableNode{
    QString title;
    QStringList infolist;
};

class MonitorLable : public QWidget
{
    Q_OBJECT
private:
    QLabel *titlelabel = nullptr;//组件的标题
    QVector <QLabel *>infolabelArr;//组件的信息内容
public:
    explicit MonitorLable(QWidget *parent = nullptr);
    explicit MonitorLable(const QStringList &infolist,QWidget *parent = nullptr);
    explicit MonitorLable(const QString title,const QStringList &infolist,QWidget *parent = nullptr);
    explicit MonitorLable(const MonitorLableNode &node,QWidget *parent = nullptr);
    MonitorLable& SetTitle(const QString& title);
    MonitorLable& RefreshMonitorLable(const QStringList &infolist);
    MonitorLable& RefreshMonitorLable(const MonitorLableNode &node);
    ~MonitorLable();
protected:
    // 重写绘制事件来绘制外边框
    void paintEvent(QPaintEvent *event) override {
        // 计算外边框区域，只包含 QLabel 区域
        QRect borderRect;
        for (QLabel *label : infolabelArr) {
            if (borderRect.isNull()) {
                borderRect = label->geometry();
            } else {
                borderRect = borderRect.united(label->geometry());
            }
        }

        // 绘制边框
        QPainter painter(this);
        QPen pen(Qt::black);
        pen.setWidth(2);
        painter.setPen(pen);
        painter.drawRect(borderRect.adjusted(-5, -5, 5, 5)); // 边框扩展一点点

        QWidget::paintEvent(event);
    }
signals:
};

#endif // MONITORLABEL_H

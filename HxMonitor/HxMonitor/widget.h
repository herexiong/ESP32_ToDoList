#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QProcess>
#include <QPushButton>
#include <QComboBox>
#include "monitorlabel.h"

#include <QSerialPortInfo>
#include <QSerialPort>

#include <QSystemTrayIcon>
#include <QMenu>
#include <QCloseEvent>
#include <QHideEvent>

QT_BEGIN_NAMESPACE
namespace Ui {
class Widget;
}
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();
    void closeEvent(QCloseEvent *event);
    void hideEvent(QHideEvent *event);

private:
    Ui::Widget *ui;
    QProcess *process;        //执行命令行
    QVector<QString>* recivedata;  //收到的数据
    bool widgetisinit = false;
    QVector<MonitorLable *>* widgetobj; //用于存储对象指针
    QPushButton* logbtn;//抓日志按钮对象指针
    QComboBox *combox;//串口选择框
    QLabel *comlabel;
    QPushButton* connectbtn;//连接按钮对象指针

    QVector<QString>ports;
    QSerialPort *serialPort;
    void USART(QString port);

    void startCmdMonitorProcess(const QString& cmd);
    void runAsAdmin(const QString &program, const QStringList &arguments);

    QSystemTrayIcon *trayIcon;
    QMenu *trayIconMenu;
    //窗口管理动作
    QAction *minimizeAction;
    QAction *maximizeAction;
    QAction *restoreAction;
    QAction *quitAction;
private slots:
    void readProcessData(void);
    void resolvedata();
    void RefreshPort(void);

    void iconActivated(QSystemTrayIcon::ActivationReason reason);
signals:
    void recivedone(void);
};
#endif // WIDGET_H

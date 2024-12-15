#include "widget.h"
#include "ui_widget.h"
#include <QVBoxLayout>
#include <QMessageBox>

#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>

#define FRAME_BEGIN QString::fromLocal8Bit("BEGIN")
#define FRAME_END QString::fromLocal8Bit("END")
#define TARGET_APP_POS QCoreApplication::applicationDirPath()+QString("/CmdMonitor/publish/CmdMonitor.exe")

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
    , recivedata(new QVector<QString>)
    ,widgetobj(new QVector<MonitorLable *>)
    ,serialPort(new QSerialPort(parent))
{
    ui->setupUi(this);

    this->logbtn = new QPushButton("CatLog",this);
    this->logbtn->resize(80,25);
    this->logbtn->show();
    connect(logbtn,&QPushButton::clicked,this,[](){
        QProcess::startDetached(TARGET_APP_POS, QStringList() << "LogMode");
    });

    this->combox = new QComboBox(this);
    this->combox->resize(180,25);
    this->combox->move(100,0);
    this->combox->addItem("Com1");
    this->combox->show();

    this->comlabel = new QLabel("串口号",this);
    this->comlabel->resize(80,25);
    this->comlabel->move(285,0);
    this->comlabel->show();

    this->connectbtn = new QPushButton("连接",this);
    this->connectbtn->resize(80,25);
    this->connectbtn->show();
    this->connectbtn->move(340,0);
    connect(connectbtn,&QPushButton::clicked,[&](){
        QString port = QString(this->combox->currentText()).split("-")[0];//获取串口号
        USART(port);
    });

    //执行命令获取
    process = new QProcess(this);
    this->startCmdMonitorProcess("");

    //有可读数据读取
    connect(process, SIGNAL(readyReadStandardOutput()), this, SLOT(readProcessData()));
    connect(this,SIGNAL(recivedone()),this,SLOT(resolvedata()));

    //托盘初始化
    QIcon icon = QIcon(QCoreApplication::applicationDirPath() +"/icon.png");
    trayIcon = new QSystemTrayIcon(this);
    trayIcon->setIcon(icon);
    trayIcon->setToolTip("a trayicon example");
    trayIcon->show(); //必须调用，否则托盘图标不显示

    //创建菜单项动作(以下动作只对windows有效)
    quitAction = new QAction("退出~", this);
    connect(quitAction, SIGNAL(triggered()), qApp, SLOT(quit())); //关闭应用，qApp对应的是程序全局唯一指针

    //创建托盘菜单(必须先创建动作，后添加菜单项，还可以加入菜单项图标美化)
    trayIconMenu = new QMenu(this);
    trayIconMenu->addSeparator();
    trayIconMenu->addAction(quitAction);
    trayIcon->setContextMenu(trayIconMenu);

    connect(trayIcon,SIGNAL(activated(QSystemTrayIcon::ActivationReason)),
            this,SLOT(iconActivated(QSystemTrayIcon::ActivationReason)));
}

void Widget::startCmdMonitorProcess(const QString& cmd)
{
    if (process->state() == QProcess::NotRunning) {
        // qDebug()<<"strat process";
        process->start(TARGET_APP_POS, QStringList()<<cmd);
    }
    if (!process->waitForStarted()) {
        qDebug() << "Error:" << process->errorString();
    }
}

void Widget::readProcessData(void)
{
    // 读取子进程标准输出
    while (process->canReadLine()) {
        QString data = QString::fromLocal8Bit(process->readLine());

        data.replace("\r", "");
        data.replace("\n", "");
        data.replace("\"", "");

        if(data.compare(FRAME_BEGIN) == 0){
            this->recivedata->clear();//清空容器准备接收
        }else if(data.compare(FRAME_END) == 0){
            emit recivedone();//触发信号，解析内容
        }else{
            this->recivedata->append(data);
        }
    }
}

void Widget::resolvedata(void)
{
    QVector<MonitorLableNode> result;
    MonitorLableNode currentNode;

    for (const QString& line : *(this->recivedata)) {
        if (line.startsWith("->")) {
            // 如果当前节点有数据，将其加入结果
            if (!currentNode.title.isEmpty() || !currentNode.infolist.isEmpty()) {
                result.append(currentNode);
                currentNode = MonitorLableNode(); // 重置当前节点
            }
            // 设置新节点的标题
            currentNode.title = line.mid(2).trimmed();
        } else {
            // 按空格分割并添加到当前节点的 infolist
            QStringList parts = line.split(" ", Qt::SkipEmptyParts);
            currentNode.infolist.append(parts);
        }
    }
    // 添加最后一个节点（如果有）
    if (!currentNode.title.isEmpty() || !currentNode.infolist.isEmpty()) {
        result.append(currentNode);
    }

    if(!widgetisinit){
        // 创建一个垂直布局
        QVBoxLayout *layout = new QVBoxLayout(this);
        layout->setSpacing(10); // 设置组件间的间距
        layout->setContentsMargins(0,  50, 0, 0); // 设置布局的边距

        for(int i = 0;i<result.size();i++)
        {
            widgetobj->append(new MonitorLable(result[i],this));
            layout->addWidget((*widgetobj)[i]); // 将组件添加到布局中
        }
        widgetisinit = true;
    }else{
        for(int i = 0;i<result.size() && i<widgetobj->size();i++)
        {
            (*widgetobj)[i] ->RefreshMonitorLable(result[i]);
        }
        RefreshPort();
    }

    // 构建 JSON 对象
    QJsonObject json;
    for(MonitorLableNode node: result)
    {
        QJsonObject temp_json = QJsonObject();
        int type = -1;//0->CPU, 1->GPU, 2->RAM, 3->Net
        for(int i = 0;i<node.infolist.size();i++)
        {
            if(node.infolist[0].startsWith("CPU")) type = 0;
            else if(node.infolist[0].startsWith("GPU")) type = 1;
            else if(node.infolist[0].startsWith("内存")) type = 2;
            else if(node.infolist[0].startsWith("网络")) type = 3;
            if(type != -1)
            {
                switch (type) {
                case 0://CPU
                    switch (i) {
                    case 0:
                        temp_json.insert("usage",node.infolist[i].split(':',Qt::SkipEmptyParts)[1]);
                        break;
                    case 1:
                        temp_json.insert("power",node.infolist[i].split(':',Qt::SkipEmptyParts)[1]);
                        break;
                    case 2:
                        temp_json.insert("temp",node.infolist[i].split(':',Qt::SkipEmptyParts)[1]);
                        break;
                    }
                    break;
                case 1://GPU
                    switch (i) {
                    case 0:
                        temp_json.insert("usage",node.infolist[i].split(':',Qt::SkipEmptyParts)[1]);
                        break;
                    case 1:
                        temp_json.insert("power",node.infolist[i].split(':',Qt::SkipEmptyParts)[1]);
                        break;
                    case 2:
                        temp_json.insert("temp",node.infolist[i].split(':',Qt::SkipEmptyParts)[1]);
                        break;
                    case 3:
                        temp_json.insert("usedRAM",node.infolist[i].split(':',Qt::SkipEmptyParts)[1].split('/')[0]);
                        temp_json.insert("totalRAM",node.infolist[i].split(':',Qt::SkipEmptyParts)[1].split('/')[1]);
                        break;
                    }
                    break;
                case 2://RAM
                    switch (i) {
                    case 0:
                        // temp_json.insert("us",node.infolist[i].split(':',Qt::SkipEmptyParts)[1]);
                        temp_json.insert("usedRAM",node.infolist[i].split(':',Qt::SkipEmptyParts)[1].split('/')[0]);
                        temp_json.insert("totalRAM",node.infolist[i].split(':',Qt::SkipEmptyParts)[1].split('/')[1]);
                        break;
                    case 1:
                        temp_json.insert("usage",node.infolist[i].split(':',Qt::SkipEmptyParts)[1]);
                        break;
                    }
                    break;
                case 3://NetWork
                    switch (i) {
                    case 0:
                        temp_json.insert("upload",node.infolist[i].split(':',Qt::SkipEmptyParts)[1]);
                        break;
                    case 1:
                        temp_json.insert("download",node.infolist[i].split(':',Qt::SkipEmptyParts)[1]);
                        break;
                    }
                    break;
                }
            }
        }
        switch (type) {
        case 0:
            temp_json.insert("title",node.title.split(':',Qt::SkipEmptyParts)[1]);
            json.insert("CPU", QJsonValue(temp_json));
            break;
        case 1:
            temp_json.insert("title",node.title.split(':',Qt::SkipEmptyParts)[1]);
            json.insert("GPU", QJsonValue(temp_json));
            break;
        case 2:
            temp_json.insert("title",node.title);
            json.insert("RAM", QJsonValue(temp_json));
            break;
        case 3:
            temp_json.insert("title",node.title.split(':',Qt::SkipEmptyParts)[1]);
            json.insert("NET", QJsonValue(temp_json));
            break;
        }
    }
    // qDebug()<<json;//验证json数据
    // //串口发送数据
    if(serialPort) serialPort->write(QJsonDocument(json).toJson().data());
    // qDebug()<<QJsonDocument(json).toJson().data();//验证json数据
}

//刷新可用串口
void Widget::RefreshPort(void) {
    QVector<QString>temp;
    //获取当前可用串口号
    for (const QSerialPortInfo& info : QSerialPortInfo::availablePorts()) {
        temp.push_back(info.portName()+"--"+info.description());
    }
    //排序现有的串口号,用于比较和原有的差距
    std::sort(temp.begin(), temp.end(), [](const auto &a, const auto &b) {
        return a < b; // 升序
    });
    if (temp != this->ports) {  //如果可用串口号有变化
        this->combox->clear();  //清除原有列表
        this->ports = temp;         //更新串口列表
        for (auto& a : ports) {     //更新新串口
            this->combox->addItem(a);
        }
    }
}

//串口通信核心
void Widget::USART(QString port) {
    static bool connect_status = false;
    QSerialPort::BaudRate Baud = QSerialPort::Baud115200;//波特率
    QSerialPort::DataBits Data = QSerialPort::Data8;     //数据位
    QSerialPort::StopBits Stop = QSerialPort::OneStop;     //停止位
    QSerialPort::Parity Check = QSerialPort::NoParity;      //校验位

    // serialPort = new QSerialPort(this);
    //为串口设置配置
    serialPort->setBaudRate(Baud);
    serialPort->setPortName(port);
    serialPort->setDataBits(Data);
    serialPort->setParity(Check);
    serialPort->setStopBits(Stop);
    if(!connect_status){
        //打开串口
        if (serialPort->open(QSerialPort::ReadWrite)) {
            qDebug()<<"串口打开";
            connectbtn->setText("关闭连接");
            connect_status = true;
        }else {
            QMessageBox::critical(this, "串口打开失败","请确认串口是否正确连接");
        }
    }else{
        serialPort->close();
        qDebug()<<"串口关闭";
        connectbtn->setText("连接");
        connect_status = false;
    }
}

void Widget::iconActivated(QSystemTrayIcon::ActivationReason reason)
{
    switch (reason)
    {
    case QSystemTrayIcon::Trigger:
        // trayIcon->showMessage("title","你单击了"); //后面两个默认参数
        this->showNormal();
        break;
    case QSystemTrayIcon::DoubleClick:
        // trayIcon->showMessage("title","你双击了");
        break;
    case QSystemTrayIcon::MiddleClick:
        // trayIcon->showMessage("title","你中键了");
        break;
    default:
        break;
    }

}

void Widget::closeEvent(QCloseEvent *event)
{
    if(trayIcon->isVisible())
    {
        hide(); //隐藏窗口
        event->ignore(); //忽略事件
    }
}

void Widget::hideEvent(QHideEvent *event)
{
    if(trayIcon->isVisible())
    {
        hide(); //隐藏窗口
        // trayIcon->showMessage("title","隐藏到托盘图标了"); //提示用户隐藏到了托盘
        event->ignore(); //忽略事件
    }
}


Widget::~Widget()
{
    delete this->ui;
    delete this->recivedata;
}

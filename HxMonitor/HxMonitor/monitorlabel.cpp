#include "monitorlabel.h"
#include <QDebug>
#include <QVBoxLayout>

MonitorLable::MonitorLable(QWidget *parent)
    : QWidget{parent}
{
}

MonitorLable::MonitorLable(const QStringList &infolist,QWidget *parent)
    :MonitorLable{"",infolist,parent}
{
    // setStyleSheet("background-color: white;"); // 背景颜色
}

MonitorLable& MonitorLable::SetTitle(const QString& title){
    if(titlelabel == nullptr){
        this->titlelabel = new QLabel(title,this);
        titlelabel->setStyleSheet("font-weight: bold; font-size: 16px;");
    }else{
        this->titlelabel->setText(title);
    }
    return *this;
}

MonitorLable::MonitorLable(const QString title , const QStringList &infolist,  QWidget *parent)
    :MonitorLable(parent)
{
    for(int i=0 ; i<infolist.size();i++){
        infolabelArr.append(new QLabel(infolist[i],this));
    }
    this->SetTitle(title);
    // 布局
    // 创建 6 个 QLabel，并添加到网格布局
    QGridLayout *gridLayout = new QGridLayout;
    gridLayout->setSpacing(10); // 网格间距
    gridLayout->setContentsMargins(0, 0, 0, 0); // 去掉网格布局边距

    for (int i = 0; i < infolabelArr.size(); i++) {
        infolabelArr[i]->setAlignment(Qt::AlignCenter);
        infolabelArr[i]->setStyleSheet("border: 1px solid gray; padding: 5px;");
        gridLayout->addWidget(infolabelArr[i], i / 3, i % 3); // 按行列布局
    }

    // 主布局，包含标题和网格布局
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->addWidget(titlelabel);
    mainLayout->addLayout(gridLayout);
    mainLayout->setSpacing(10); // 标题和网格的间距
    mainLayout->setContentsMargins(10, 10, 10, 10); // 主布局边距

    setLayout(mainLayout);
}

MonitorLable::MonitorLable(const MonitorLableNode &node,QWidget *parent)
    :MonitorLable(node.title,node.infolist,parent)
{}

MonitorLable& MonitorLable::RefreshMonitorLable(const QStringList &infolist){
    for(int i = 0;i<infolabelArr.size()&&i<infolist.size();i++){
        infolabelArr[i]->setText(infolist[i]);
    }
    return *this;
}

MonitorLable& MonitorLable::RefreshMonitorLable(const MonitorLableNode &node){
    return RefreshMonitorLable(node.infolist);
}


MonitorLable::~MonitorLable()
{
    qDebug()<<"MonitorLable delete";
}

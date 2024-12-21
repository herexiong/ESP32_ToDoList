# 桌面便签
功能
+ 带NTP校准的时钟日期显示
+ 获取本地实时天气信息
+ DLNA局域网音乐投放
+ 温湿度，二氧化碳传感器监测
+ 倒计时功能
+ Windows性能监视器
+ 同步ToDoist代办事项（开发中）
## 性能监视器
性能监视器只支持Windows，基于[LibreHardwareMonitor](https://github.com/LibreHardwareMonitor/LibreHardwareMonitor)动态库实现的二次开发的CmdMonitor软件实现将对应信息打印到控制台上，再由QT开发的上位机软件捕捉控制台输出，将其通过串口发送给下位机，虽然看着十分繁琐，但好在CPU占用在0.1%左右，内存占用10M以内。LibreHardwareMonitor是由C#编写的，我尝试在QT中调用它的动态库，但是没成功，只能绕个圈圈用C#二次开发的CmdMonitor。  

在AMD更新24.12.1版驱动后，AMD显卡的很多信息无法被LibreHardwareMonitor所输出，即使LibreHardwareMonitor软件也是一样，之前22.x版本是正常的，等LibreHardwareMonitor更新后可能会修复。  

由于ESP32S3的外部IO接口有限，因此使用IDF默认的串口进行和上位机通讯，如果需要调试，请关闭下位机的性能监视器任务，或多插两个串口使用端口转发进行调试。  
## OTA功能
OTA组件修改[esp_ghota](https://github.com/Fishwaldo/esp_ghota),将Github的相关API改为Gitee。使用该组件可以使得下位机自动监测仓库地址的release是否有新的固件，若有则会从仓库release下载并执行更新，免去了搭建服务器的麻烦，由于总所周知的问题，国内并不能稳定的访问Github服务器，这也是为什么改为Gitee和建立Gitee镜像仓库的原因。  
## 天气功能
天气使用心知天气提供的免费API，通过HTTP的get方法获取到json信息后使用cJSON库进行解析，但目前无法做到根据IP自动获取地区，需要手动设置地区
## DLNA功能
局域网音乐投放目前只做了单声道，使用解码板附带的喇叭效果实在难以恭维，后期可能会升级双声道并换上MacBook的扬声器
## 本地传感器功能
使用SGP30和SHT30两款I2C接口的传感器监测温湿度和二氧化碳信息，使用I2C接口可以有效的节省下位机IO口资源，但SGP30似乎需要连续运行30个小时以上才准确，并使用的动态基准线技术实现校准，后续在开发NVS功能时会把这个缺点补上
## 代办清单
原本打算使用ESP32做一个网页服务器实现代办功能，但实用性实在有限，不如找一个可以提供外部API的代办清单，通过同步获取代办显示到下位机上  
通过Todoist的API，通过https的get方法实现获取待办，下位机勾选后，通过post方法实现删除待办
## 文件介绍
### ESP32Project
用于存放ESP32的代码
### SquareLineProject
SquareLineProject是LVGL的图形化UI生成工具，此文件夹用于存放SquareLine工程文件及其导出的文件
### ToDoList
用于前期验证备忘录的存储及其存储结构
### HxMonitor
./publish 上位机软件，支持windows，64位软件可以直接运行  
./HxMonitor QT工程文件  
### CmdMonitor
./publish 编译后的软件，可以直接运行


## [版本更新日志](./更新日志.md)

## [后续开发计划](./开发计划.md)

## [代码说明文件](./ESP32Project/README_CH.md)

## [gitee镜像仓库](https://gitee.com/herexiong/ESP32_ToDoList)

## Q&A
Q:为什么IDF无法成功编译  
A:1.如果使用了clash获取其他代理软件，尝试关闭代理后编译

Q:为什么烧录失效  
A:请查看[烧录说明](./ESP32Project/README_CH.md)
A:无法进入烧录模式请拔除外设电源线后重试

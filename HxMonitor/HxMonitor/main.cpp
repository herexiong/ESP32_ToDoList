#include "widget.h"
#include <windows.h>
#include <QApplication>
#include <QMessageBox>
#include <QFile>
#include <QString>

#define VERSION "V0.0"

#include <QApplication>

bool restartWithAdminPrivileges(const QString &appPath) {
    // 检查当前是否是管理员权限
    BOOL isAdmin = FALSE;
    SID_IDENTIFIER_AUTHORITY NtAuthority = SECURITY_NT_AUTHORITY;
    PSID AdministratorsGroup;
    if (AllocateAndInitializeSid(&NtAuthority, 2, SECURITY_BUILTIN_DOMAIN_RID,
                                 DOMAIN_ALIAS_RID_ADMINS, 0, 0, 0, 0, 0, 0, &AdministratorsGroup)) {
        CheckTokenMembership(NULL, AdministratorsGroup, &isAdmin);
        FreeSid(AdministratorsGroup);
    }

    if (isAdmin) {
        return true; // 已是管理员权限，无需提升
    }

    // 创建提升权限的进程
    SHELLEXECUTEINFO sei = { sizeof(SHELLEXECUTEINFO) };
    sei.lpVerb = L"runas"; // 提升权限关键参数
    sei.lpFile = (LPCWSTR)appPath.utf16();
    sei.hwnd = NULL;
    sei.nShow = SW_NORMAL;
    if (!ShellExecuteEx(&sei)) {
        QMessageBox::critical(nullptr, "Error", "Failed to restart as administrator.");
        return false; // 提升失败
    }
    return false; // 当前进程应退出
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // 获取当前程序路径
    QString appPath = QApplication::applicationFilePath();
    if (!restartWithAdminPrivileges(appPath)) {
        return 0; // 如果需要重新启动，则退出当前进程
    }

    Widget w;
    w.setWindowTitle(QString("HxMonitor ")+VERSION);
    w.show();

    return a.exec();
}

using HidSharp.Utility;
using LibreHardwareMonitor.Hardware;
using LibreHardwareMonitor.Hardware.Cpu;
using System.Net.NetworkInformation;
using System.Security.Principal;
using System;
using System.IO;
using System.Reflection.Metadata;

class Program
{
    private static double app_version = 0.1;
    private static long previousBytesSent = 0;
    private static long previousBytesReceived = 0;
    private static bool IsAdministrator = false;
    private static bool LogMode = false;
    private static bool ManualMode = false;

    // 检查是否以管理员身份运行
    static bool IsRunningAsAdministrator()
    {
        // 获取当前用户
        var identity = WindowsIdentity.GetCurrent();
        var principal = new WindowsPrincipal(identity);

        // 检查是否为管理员角色
        return principal.IsInRole(WindowsBuiltInRole.Administrator);
    }

    //根据传入的参数决定工作模式
    static void Main(string[] args)
    {
        if (args.Length !=0 && args[0] == "LogMode") LogMode = true;
        else if (args.Length != 0 && args[0] == "ManualMode") ManualMode = true; 

        Computer computer = new Computer
        {
            IsCpuEnabled = true,
            IsGpuEnabled = true,
            IsMemoryEnabled = true,
            IsNetworkEnabled = true
        };
        
        computer.Open();

        if (IsRunningAsAdministrator()) IsAdministrator = true;

        if (!LogMode)
        {
            DisplayPerformance(computer);
            if (ManualMode) 
            { 
                // 隐藏光标
                Console.CursorVisible = false;
            }
            System.Timers.Timer timer = new System.Timers.Timer(1000);// 每5秒更新一次
            timer.Elapsed += (sender, e) => DisplayPerformance(computer);
            timer.Start();
            Console.ReadLine();
            timer.Stop();
        }
        else
        {
            CatLog(computer);
        }
        computer.Close();
    }

    private static void DisplayPerformance(Computer computer)
    {
        //Console.Clear(); // 每秒清空控制台重新打印
        if (ManualMode)
        {
            Console.SetCursorPosition(0, 0);
            Console.WriteLine("性能监视器运行中，按任意键退出...");
            if (IsAdministrator) Console.WriteLine("管理员模式");
        }
        else//添加开始符号以供QT程序识别
        {
            if (!ManualMode) Console.WriteLine("BEGIN");
        }

        foreach (var hardware in computer.Hardware)
        {
            hardware.Update();

            // CPU 信息
            if (hardware.HardwareType == HardwareType.Cpu)
            {
                float totalCpuUsed = 0; float pkgCpuTemp = 0; float totalCpuPower = 0;
                Console.WriteLine($"->CPU:{hardware.Name}");

                foreach (var sensor in hardware.Sensors)
                {
                    if (sensor.SensorType == SensorType.Load && sensor.Name.Equals("CPU Total")) totalCpuUsed = (float)sensor.Value;//Console.WriteLine($"CPU 占用率: {sensor.Value:F2} %");
                    //AMD和Intel可能存在命名区别
                    if (sensor.SensorType == SensorType.Temperature) {
                        if (sensor.Name.Equals("Core (Tctl/Tdie)") || sensor.Name.Equals("CPU Package")) {
                            pkgCpuTemp = (float)sensor.Value;//Console.WriteLine($"CPU 温度: {sensor.Value:F2} °C");             
                        }
                    }
                    if (sensor.SensorType == SensorType.Power && sensor.Name.Equals("Package") || sensor.Name.Equals("CPU Package")) totalCpuPower = (float)sensor.Value;//Console.WriteLine($"CPU 功率: {sensor.Value:F2} W");
                }

                Console.WriteLine($"CPU利用率:{totalCpuUsed:F2}% 功率:{totalCpuPower:F1}W 温度:{pkgCpuTemp:F1}°C");
            }

            // GPU 信息
            if (hardware.HardwareType == HardwareType.GpuNvidia ||
                hardware.HardwareType == HardwareType.GpuAmd ||
                hardware.HardwareType == HardwareType.GpuIntel)
            {
                Console.WriteLine($"->GPU:{hardware.Name}");
                float totalGpuMem = 0; float freeGpuMem = 0; float usedGpuMem = 0;
                float pkgGpuTemp = 0; float totalGpuUsed = 0; float totalGpuPower = 0;
                foreach (var sensor in hardware.Sensors)
                {
                    if (sensor.Name.Equals("GPU Memory Total")) totalGpuMem = (float)sensor.Value;//Console.WriteLine($"显存: {sensor.Value} M");
                    if (sensor.Name.Equals("GPU Memory Free")) freeGpuMem = (float)sensor.Value;//Console.WriteLine($"GPU MEM Free: {sensor.Value} M");
                    if (sensor.Name.Equals("GPU Memory Used")) usedGpuMem = (float)sensor.Value;//Console.WriteLine($"GPU MEM Used: {sensor.Value} M");
                    if (sensor.SensorType == SensorType.Temperature)
                    {
                        if(sensor.Name.Equals("GPU VR SoC") || sensor.Name.Equals("GPU Core"))
                            pkgGpuTemp = (float)sensor.Value;//Console.WriteLine($"GPU 温度: {sensor.Value} °C");
                    }
                    if (sensor.SensorType == SensorType.Power) {
                        if (sensor.Name == "GPU Core" || sensor.Name == "GPU Package")
                        {
                            totalGpuPower = (float)sensor.Value;//Console.WriteLine($"GPU Load: {sensor.Value} %");
                        }
                    }
                    if (sensor.SensorType == SensorType.Load)
                    {
                        if (sensor.Name.Equals("GPU Core"))
                            totalGpuUsed = (float)sensor.Value;//Console.WriteLine($"GPU Load: {sensor.Value} %");
                    }
                }

                Console.WriteLine($"GPU利用率:{totalGpuUsed:F2}% 功率:{totalGpuPower:F1}W 温度:{pkgGpuTemp:F1}°C 显存:{usedGpuMem}M/{totalGpuMem}M");
            }

            // 内存信息
            if (hardware.HardwareType == HardwareType.Memory)
            {
                float totalRam = 0; float freeRam = 0; float usedRam = 0;
                foreach (var sensor in hardware.Sensors)
                {
                    //if (sensor.Name.Equals("Memory")) totalRam = (float)sensor.Value;//Console.WriteLine($"MEM Used: {sensor.Value:F2} %");
                    if (sensor.SensorType == SensorType.Data && sensor.Name.Equals("Memory Used")) usedRam = (float)sensor.Value;//Console.WriteLine($"MEM Used: {sensor.Value:F2} G");
                    if (sensor.SensorType == SensorType.Data && sensor.Name.Equals("Memory Available")) freeRam = (float)sensor.Value;//Console.WriteLine($"MEM Available: {sensor.Value:F2} G");
                    totalRam = usedRam + freeRam;
                }
                Console.WriteLine("->内存:");
                Console.WriteLine($"内存:{usedRam:F2}G/{totalRam:F2}G 已用:{usedRam/ totalRam*100:F2}%");
            }
        }
        // 网络速度
        DisplayNetworkSpeed();
        //添加结束符号以供QT程序识别
        if(!ManualMode) Console.WriteLine("END");
    }

    private static void CatLog(Computer computer)
    {
        // 获取程序运行目录
        string directory = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Logs"); // 构建相对路径
        Directory.CreateDirectory(directory); // 如果目录不存在，创建它
        // 获取当前系统时间并格式化为文件名
        string timestamp = DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
        string fileName = Path.Combine(directory, $"{timestamp}.txt");

        try
        {
            // 使用 StreamWriter 写入文件
            using (StreamWriter writer = new StreamWriter(fileName, append: true))
            {
                if (IsAdministrator)
                {
                    writer.WriteLine("Administrator Mode");
                }
                writer.WriteLine($"APP VERSION:{ app_version}");
                foreach (var hardware in computer.Hardware)
                {
                    hardware.Update();
                    // CPU 信息
                    if (hardware.HardwareType == HardwareType.Cpu)
                    {
                        writer.WriteLine("**********************************************");
                        writer.WriteLine($"CPU Model: {hardware.Name}");
                        foreach (var sensor in hardware.Sensors)
                        {
                            writer.WriteLine($"{sensor.SensorType} , {sensor.Name} , {sensor.Value}");
                        }
                    }
                    // GPU 信息
                    if (hardware.HardwareType == HardwareType.GpuNvidia ||
                        hardware.HardwareType == HardwareType.GpuAmd ||
                        hardware.HardwareType == HardwareType.GpuIntel)
                    {
                        writer.WriteLine("**********************************************");
                        Console.WriteLine($"GPU Model: {hardware.Name}");
                        foreach (var sensor in hardware.Sensors)
                        {
                            writer.WriteLine($"{sensor.SensorType} , {sensor.Name} , {sensor.Value}");
                        }
                    }
                    // 内存信息
                    if (hardware.HardwareType == HardwareType.Memory)
                    {
                        writer.WriteLine("**********************************************");
                        foreach (var sensor in hardware.Sensors)
                        {
                            writer.WriteLine($"{sensor.SensorType} , {sensor.Name} , {sensor.Value}");
                        }
                    }
                }
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine($"Error writing to file: {ex.Message}");
        }   
    }
    private static void ClearConsoleLine(int row)
    {
        // 将光标定位到指定行的开头
        Console.SetCursorPosition(0, row);

        // 用空格覆盖整行
        Console.Write(new string(' ', Console.WindowWidth));

        // 重置光标到该行的开头
        Console.SetCursorPosition(0, row);
    }

    private static void DisplayNetworkSpeed()
    {
        var networkInterface = NetworkInterface.GetAllNetworkInterfaces().FirstOrDefault(ni => ni.OperationalStatus == OperationalStatus.Up);
        if (networkInterface == null)
        {
            Console.WriteLine("->网络接口不可用");
            return;
        }
        Console.WriteLine("->网络名称:" + networkInterface.Name);
        
        var stats = networkInterface.GetIPStatistics();
        long bytesSent = stats.BytesSent;
        long bytesReceived = stats.BytesReceived;

        float uploadSpeed = (bytesSent - previousBytesSent) / 1024f; // KB/s
        float downloadSpeed = (bytesReceived - previousBytesReceived) / 1024f; // KB/s

        previousBytesSent = bytesSent;
        previousBytesReceived = bytesReceived;

        if(ManualMode) ClearConsoleLine(Console.CursorTop);
        if(uploadSpeed > 1024f) Console.WriteLine($"网络上传速度:{uploadSpeed/1024f:F2}M/s");
        else Console.WriteLine($"网络上传速度:{uploadSpeed:F2}KB/s");
        if (ManualMode) ClearConsoleLine(Console.CursorTop);
        if (downloadSpeed > 1024f) Console.WriteLine($"网络上传速度:{downloadSpeed / 1024f:F2}M/s");
        else Console.WriteLine($"网络下载速度:{downloadSpeed:F2}KB/s");
    }
}

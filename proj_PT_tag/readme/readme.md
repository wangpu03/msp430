
# 总说明
该文件主要用于PT和tag两端数据的采集。系统启动后，初始化所有设备，并进入低功耗模式。等待上位机发送的命令。
* p用于测试环境是否正常工作
* LIVE_MODE： 采样直接发送给上位机模式
    - 设置参考电压，ADC初始化，包括时钟频率，采样通道，缓冲寄存器，采样模式等。虽然设置为双通道采样，但只取单通道的采样值。
    - 设置计数器，用于控制ADC的采样频率
    - 计数器启动采样，将采样值发送给上位机，之后进行backscatter发射，循环采样。相当于不反射是采样一次，反射时采样一次，交替进行。
* FRAM_LOG_MODE
    - 设置参考电压，ADC初始化，包括时钟频率，采样通道，缓冲寄存器，采样模式等。双通道
    - 设置计数器，用于控制ADC的采样频率
    - 计数器启动采样，将双通道的采样值存储在FRAM存储器中所设定的存储块中，若存储块已存满数据，计数采样，退出该模式，进行下一模式
* FRAM_SEND_MODE
    - 将上述模式的所存储的数据发送给上位机。
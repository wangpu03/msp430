# 笔记
* 看门狗是为计算机系统安全考虑的一个外围设备，在计算机出现故障时，可以通过看门狗自动恢复计算机的继续运行。在看门狗启动之后，就必须不断在看门狗时间内将其内容通过程序清楚。
* 看门狗定时器设置一定时间，比如250毫秒，这个时间是所以用户程序一定能在此时间内执行外该程序的时间，设置好这个定时时间后，所以用户程序就必须在这个设定的时间内将看门狗计数器的值清零，所该计数器重新计数。如果CPU执行程序正确，则看门狗计数器始终能在规定的时间内被用户程序清零而不能计数到250毫秒。而当程序跑飞后，看门狗计数器得不到用户程序清零，能计数到 250 毫秒，发生溢出，导致 CPU 复位，这样 CPU 又会重新运行用户程序。
* 按键消除抖动，
    - 使用 R-S 触发器构成消抖动开关以消抖动
    - 使用电阻与电容构成积分器将抖动滤除
    - 使用软件延时的办法消除抖动

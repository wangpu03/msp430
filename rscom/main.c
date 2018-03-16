#include <msp430.h> 


/**
 * main.c
 */
int main(void){
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	UTCTL0 = SSEL0;             // UCLK = ACLK，选择时钟来源
	UBR00 = 0x03;               // 32k/9600 - 3.41 波特率寄存器低字节
	UBR10 = 0x00;               // 32k/9600 波特率寄存器高字节
	UMCTL0 = 0x51;              // 由于波特率计算有余数，填写波特率调整寄存器
	UCTL0 = CHAR;               // 数据格式为 8 位数据
	ME1 |= UTXE0 + URXE0;       // 使能串口 TXD 与 RXD
	IE1 |= URXIE0;              // 让串口接收到数据后能产生中断
	P2SEL |= 0x30;              // 定义 P2.4,P2.5 为串口功能引脚
	P2DIR |= 0x10;              // 串口发送数据端口为输出，接收数据端口为输入
	_EINT();                    // 整个系统使能中断（开总中断）
	_BIS_SR(LPM3_bits);         // 初始化完毕，进入睡眠状态，主程序完毕
	
	return 0;
}


interrupt[UART0RX_VECTOR] void usart0_rx (void){
    while ((IFG1 & UTXIFG0) == 0); // 当发送缓存为空时
    TXBUF0 = RXBUF0; //发送数据到串口
}

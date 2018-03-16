#include <msp430.h> 


/**
 * main.c
 */
int main(void){
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	UTCTL0 = SSEL0;             // UCLK = ACLK��ѡ��ʱ����Դ
	UBR00 = 0x03;               // 32k/9600 - 3.41 �����ʼĴ������ֽ�
	UBR10 = 0x00;               // 32k/9600 �����ʼĴ������ֽ�
	UMCTL0 = 0x51;              // ���ڲ����ʼ�������������д�����ʵ����Ĵ���
	UCTL0 = CHAR;               // ���ݸ�ʽΪ 8 λ����
	ME1 |= UTXE0 + URXE0;       // ʹ�ܴ��� TXD �� RXD
	IE1 |= URXIE0;              // �ô��ڽ��յ����ݺ��ܲ����ж�
	P2SEL |= 0x30;              // ���� P2.4,P2.5 Ϊ���ڹ�������
	P2DIR |= 0x10;              // ���ڷ������ݶ˿�Ϊ������������ݶ˿�Ϊ����
	_EINT();                    // ����ϵͳʹ���жϣ������жϣ�
	_BIS_SR(LPM3_bits);         // ��ʼ����ϣ�����˯��״̬�����������
	
	return 0;
}


interrupt[UART0RX_VECTOR] void usart0_rx (void){
    while ((IFG1 & UTXIFG0) == 0); // �����ͻ���Ϊ��ʱ
    TXBUF0 = RXBUF0; //�������ݵ�����
}
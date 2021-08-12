/*
 * Lchika1.c
 *
 * Created: 2020/05/11 14:28:55
 * Author : kuras
 */ 
#define F_CPU   1000000UL

#include <avr/io.h>
//#include "avr/iom328p.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include "iic.h"
#include "eeprom.h"
#include "adc.h"

#define WDT_8s 9
#define WDT_4s 8
#define WDT_2s 7
#define WDT_1s 6
#define WDT_500ms 5
#define WDT_250ms 4
#define WDT_125ms 3

void setup_WDT( uint8_t delay )
{
	// ウォッチドッグタイマのタイムアウト設定
	if( delay > 9 ) delay = 9;
	uint8_t reg_data = delay & 7;
	if( delay & 0x8 )
	{
		reg_data |= 0x20;
	}

	MCUSR &= ~(1 << WDRF);
	//WDTCSR |= (1 << WDCE) | (1 << WDE); // 設定変更許可
	//WDTCSR = reg_data | 1<<WDE;	// タイムアウト設定
	WDTCSR |= (1 << WDCE) | (1 << WDE); // 設定変更許可
	WDTCSR = reg_data | 0<<WDE;	// タイムアウト設定
	WDTCSR |= (1 << WDIE); // 割り込み許可
}

int write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
	iic_start();
	int ack = iic_send(dev_addr<<1);
	if( ack != 0 )
	{
		return 10;
	}

	ack = iic_send(reg_addr);
	if( ack != 0 )
	{
		return 11;
	}

	ack = iic_send(data);
	if( ack != 0 )
	{
		return 12;
	}

	iic_stop();
	return 0;
}

void lcd_cmd( uint8_t cmd )
{
	write_reg( 0x3e, 0, cmd );
}

void lcd_data( uint8_t d )
{
	write_reg( 0x3e, 0x40, d );
}

void lcd_puts( const char *mes )
{
	while(*mes)
	lcd_data(*mes++);
}

#define contrast 40
extern const char cgramdata[8][8];

void lcd_init()
{
	// reset
	_delay_ms(500);
	PORTC &= ~2; // reset = low
	_delay_ms(1);
	PORTC |= 2;  // reset = hi
	_delay_ms(10);

	// LCD initialize
	_delay_ms(40);
	lcd_cmd(0x38); // function set
	lcd_cmd(0x39); // function set
	lcd_cmd(0x14); // interval osc
	lcd_cmd(0x70 | (contrast & 15)); // contrast low
	lcd_cmd(0x5c | (contrast >> 4 & 3)); // contrast high / icon / power
	lcd_cmd(0x6c); // follower control
	_delay_ms(300);

	lcd_cmd(0x3c); // function set (1line). if set to 2lines->0x38, 1line->0x3c
	lcd_cmd(0x0c); // display on
	lcd_cmd(0x01); // clear display
	_delay_ms(2);
	// CGRAM書き込み
	for(int c=0; c<8; c++)
	{
		for(int r=0; r<8; r++)
		{
			lcd_cmd(0x40+c*8+r);
			lcd_data(cgramdata[c][r]);
		}
	}
	lcd_cmd(0x01); // clear display
}

void lcd_move(uint8_t pos)
{
	lcd_cmd(0x80 | pos);
}

#define BAUD_PRESCALE_RUNNING 103 // U2X=1 8MHz 9600B
#define BAUD_PRESCALE_POWERUP 8 // U2X=1 8MHz 115200B 計算上は８が正しい
#define BAUD_PRESCALE 12 // U2X=1 1MHz 9600B
//#define BAUD_PRESCALE_POWERUP 100 // U2X=1 8MHz 9600B計算上は１０３が正しい

volatile uint8_t bRecieve;
char *waiting_message;

void setup_USART(unsigned int prescale){
	// Set baud rate
	UBRR0H = (prescale >> 8);
	UBRR0L = prescale;// Load lower 8-bits into the low byte of the UBRR register

	// 倍速ビットON
	UCSR0A = 2;
	// Enable receiver and transmitter and receive complete interrupt
	UCSR0B = ((1<<TXEN0)|(1<<RXEN0) | (1<<RXCIE0));
	UCSR0C = 0b00000110;
}

void USART_SendByte(uint8_t u8Data){

	// Wait until last byte has been transmitted
	while((UCSR0A &(1<<UDRE0)) == 0);

	// Transmit data
	UDR0 = u8Data;
	
	// clear tx complete flag
	UCSR0A |= (1<<TXC0);
}

void USART_SendStr(const char * str)
{
	uint8_t i = 0;
	while( str[i] != '\0' )
	{
		USART_SendByte(str[i++]);
	}
	
	// wait for tx complete
	while( (UCSR0A & (1<<TXC0)) == 0 );
}

#define RCV_SIZE 100
char RCV_BUF[RCV_SIZE];
char MSG_BUF[RCV_SIZE];
uint8_t RCV_PTR = 0;
uint8_t RCV_FILTER = 0;

volatile uint8_t bline_cmp = 0;
char strAddr[5];
bool bAddrFilter = false;

ISR (USART_RX_vect)
{
	char value = UDR0;             //read UART register into value
	
	if( value == '\x0d')
	{
	}
	else if( value == '\x0a' )
	{
		RCV_BUF[RCV_PTR] = '\0';
		bool isTarget = false;
		if( RCV_FILTER != 0 )
		{
			if( strstr((const char *)RCV_BUF, "9999") != NULL )
			{
				if( bAddrFilter )
				{
					if( strstr((const char *)RCV_BUF, strAddr) != NULL ) isTarget = true;
				}
				else
				{
					isTarget = true;
				}
			}

			if( isTarget )
			{
				strcpy((char*)MSG_BUF,(const char*)RCV_BUF);
				bline_cmp = RCV_PTR;
			}
		}
		else
		{
			if( RCV_PTR != 0 )
			{
				strcpy((char*)MSG_BUF,(const char*)RCV_BUF);
				bline_cmp = RCV_PTR;
			}
		}
		RCV_PTR = 0;
	}
	else if(isprint(value))
	{
		RCV_BUF[RCV_PTR] = value;
		if( RCV_PTR < RCV_SIZE-1 )
		{
			RCV_PTR++;
		}
	}
}

ISR (WDT_vect)
{
}

void t_2_str(char *mes, char *buf)
{
	int tmp = (mes[0]-0x30)*1000;
	tmp += (mes[1]-0x30)*100;
	tmp += (mes[2]-0x30)*10;
	tmp += (mes[3]-0x30)*1;
	tmp += 5;
	sprintf(buf, "%2d.%d", tmp/100, (tmp/10)%10);
}

void p_2_str(char *mes, char *buf)
{
	int tmp = (mes[0]-0x30)*1000;
	tmp += (mes[1]-0x30)*100;
	tmp += (mes[2]-0x30)*10;
	tmp += (mes[3]-0x30)*1;
	sprintf(buf, "%4d", tmp);
}

void h_2_str(char *mes, char *buf)
{
	int tmp = (mes[0]-0x30)*1000;
	tmp += (mes[1]-0x30)*100;
	tmp += (mes[2]-0x30)*10;
	tmp += 50;
	sprintf(buf, "%2d", tmp/100);
}

void v_2_str(char *mes, char *buf)
{
	buf[0] = mes[1];
	buf[1] = '.';
	buf[2] = mes[2];
	buf[3] = mes[3];
	buf[4] = '\0';
}

void exec_cmd(char *msg, char *buf)
{
	for(uint8_t i=0; msg[i]!='\0'; i++)
	{
		if( !isalpha(msg[i])) continue;
		msg[i] = toupper(msg[i]);
	}

	if( strncmp(msg, "ADR", 3) == 0 )
	{
		// デバイスアドレス設定
		uint16_t adr = strtoul(msg+4, NULL, 16);
		write_eeprom( 0x08, (unsigned char)(adr & 0xff) );
		write_eeprom( 0x09, (unsigned char)(adr >> 8) );
		strcpy(buf, "ACK\x0d\x0a");
	}
	else if(strncmp(msg,"RR", 2) == 0 )
	{
		// ROM読み込み
		uint16_t adr = strtoul(msg+3, NULL, 16);
		uint16_t dat = read_eeprom(adr);
		sprintf(buf, "ACK %x\x0d\x0a", dat);
	}
	else if( strncmp(msg, "RW", 2) == 0 )
	{
		// ROM書き込み
		uint16_t adr = strtoul(msg+3, NULL, 16);
		uint16_t dat = strtoul(msg+6, NULL, 16);
		write_eeprom( adr, (unsigned char)dat );
		strcpy(buf, "ACK\x0d\x0a");
	}
	else
	{
		sprintf(buf, "ERR %s\x0d\x0a", msg);
	}
}

void setting_mode()
{
	// アドバタイズ開始
	USART_SendStr("a\x0d\x0a");
	// 接続チェック
	bool bConnect = false;
	for(int i=0; i<30; i++)
	{
		PORTD |= (1 << 4);
		_delay_ms(100);
		PORTD &= ~(1 << 4);

		_delay_ms(900);
		if( PINB & 0x4 )
		{
			bConnect = true;
			break;
		}
	}
	// 接続なし
	if( !bConnect )
	{
		// アドバイス停止
		USART_SendStr("y\x0d\x0a");
		return;
	}
	lcd_cmd(0x01); // clear display
	lcd_puts("Connected ...");
	// 接続あり
	PORTD |= (1 << 4);
	_delay_ms(1000);
	PORTD &= ~(1 << 4);
	// MLPDモード設定
	PORTB |= 2;
	// コマンド受付・実行
	RCV_FILTER = 0;
	sei();
	bline_cmp = 0;
	for(;;)
	{
		// コマンド受け取待ち
		for(;;)
		{
			// 接続が切れたら終了
			if( (PINB & 0x4) == 0 )
			{
				PORTB &= ~2; // cmd mode
				PORTD &= ~(1 << 4); // led off
				return;
			}
			// コマンド受け取ったらループ抜ける
			if( bline_cmp != 0 )
			{
				bline_cmp = 0;
				break;
			}
		}
		// コマンド実行
		char buf[100];
		exec_cmd((char*)MSG_BUF, buf);
		// 戻りを送信
		USART_SendStr(buf);
	}
}

void readAddrFilter()
{
	uint8_t addr8 = read_eeprom(8);
	uint8_t addr7 = read_eeprom(9);
	if( addr8 != 0xff && addr7 != 0xff )
	{
		sprintf(strAddr, "%02X%02X", addr7, addr8);
		bAddrFilter = true;
	}
	else
	{
		strcpy(strAddr, "FFFF");
	}
}

int main(void)
{
	uint8_t cal = read_eeprom(0);
	if(cal != 0xff)
	{
		OSCCAL = cal;
	}
	CLKPR = 0x80;// クロック分周比変更許可ビットON
	CLKPR = 0x03;// 分周比1/8 : クロックは1MHz
	
	readAddrFilter();
	
	// B0 : output : WAKE_SW
	DDRB |= 1;
	PORTB |= 1; // WAKE_SW = 0 active mode
	// B1 : output : CMD_MLDP
	DDRB |= 2;
	PORTB &= ~2; // CMD_MLDP = 0 cmd mode
	// B2 : input connect
	DDRB &= ~4;

	PORTD |= 1; // RXD端子プルアップ

	// D4 : output : LED
	DDRD = 1 << 4;
	PORTD &= ~(1 << 4);
	
	// C1 : output : lcd module reset
	DDRC |= 2;
	PORTC |= 2;
	
	// C2 : output : lcd module back light
	DDRC |= 4;
	PORTC |= 4;
	
	setup_iic();
	lcd_init();
	lcd_puts("Start BLE device");

	setup_USART(BAUD_PRESCALE);
	bline_cmp = 0;

	_delay_ms(5000); // RN4020が起動するまで待つ
	lcd_cmd(0x01); // clear display
	lcd_puts("Exec Advertise..");

	USART_SendStr("SR,1A000000\x0d\x0a"); // ペリフェラル、MLDP Enable、Auto MLDP disable
	setting_mode();
	lcd_cmd(0x01); // clear display
	lcd_puts("Display [ ");
	lcd_puts(strAddr);
	lcd_puts(" ]");
	RCV_FILTER = 1;
	bline_cmp = 0;
	sei();

	bool RcvSuccess = false;
	for(;;)
	{
		// 電池電圧測定
		enable_adc();
		setup_adc(0);
		uint16_t adc_val = exec_adc();
		_delay_ms(10);
		disable_adc();

		PORTB |= 1; // operation mode
		PORTB &= ~2; // CMD mode

		// アドバタイズスキャン開始
		_delay_ms(5);
		USART_SendStr("j,1\x0d\x0a");
		USART_SendStr("f\x0d\x0a");

		// 最大５秒待機
		for(int k=0; k<50; k++)
		{
			_delay_ms(100);
			if( bline_cmp != 0 ) break;	
		}
		// スキャン停止
		USART_SendStr("x\x0d\x0a");

		// 電池電圧を表示して１秒待機
		lcd_cmd(0x01);
		int vol = (int)(((double)adc_val * (3.3/1024) +0.005) *100);
		char buf[10];
		sprintf(buf, "%1d.%02dV", vol/100, vol%100);
		lcd_puts(buf);
		_delay_ms(2000);
		
		lcd_cmd(0x01); // clear display

		// 電池電圧をアイコン表示
		if( vol < 150 )
		lcd_data('\x00');
		else if(vol<170)
		lcd_data('\x01');
		else if(vol<190)
		lcd_data('\x02');
		else if(vol<210)
		lcd_data('\x03');
		else if(vol<230)
		lcd_data('\x04');
		else if(vol<250)
		lcd_data('\x05');
		else if(vol<270)
		lcd_data('\x06');
		else
		lcd_data('\x07');

		if( bline_cmp != 0 )
		{
			// 受信できた場合
			RcvSuccess = true;
			bline_cmp = 0;
			char *data = strstr((const char *)MSG_BUF, "9999");

			// 気温符号
			if(data[9] == '0')
				lcd_data(' ');
			else
				lcd_data('-');

			// 気温
			t_2_str(data+10, buf);
			lcd_puts(buf);
			lcd_data('\xf2'); // °

			if( strlen((const char*)data) > 18 )
			{
				// 湿度
				lcd_data(' ');
				h_2_str(data+18, buf);
				lcd_puts(buf);
				lcd_data('\x25'); // %

				// 気圧
				lcd_data(' ');
				p_2_str(data+22, buf);
				lcd_puts(buf);
			}
		}
		else
		{
			RcvSuccess = false;
			lcd_puts("No Signal: ");
			lcd_puts(strAddr);
		}


		_delay_ms(5);
		PORTB &= ~1; // sleep mode
		// 受信成功の場合64秒、失敗の場合24秒待機
		int wait;
		if( RcvSuccess ) wait = 8;
		else wait = 3;
		for(int j=0; j<wait; j++)
		{
			setup_WDT(WDT_8s);
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
			sleep_enable();
			// BOD禁止処理
			MCUCR |= (1<<BODSE) | (1<< BODS);
			MCUCR = (MCUCR & ~(1 << BODSE))|(1 << BODS);
			sleep_cpu();
			sleep_disable();
		}
	}
}

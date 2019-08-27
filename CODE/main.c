/*
 * main.c
 *
 *  Created on: 16 апр. 2019 г.
 *      Author: gavrilov.iv
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <avr/eeprom.h>
#include "bits_macros.h"
#include "lcd_lib_2.h"
#include "buttons.h"
#include "ADS1115.h"
#include "ds1307.h"

#define VERSION				10

#define CALC_INTERVAL		1
#define FACTOR				60.0 / (float)CALC_INTERVAL

#define	DEF_PAST_TEMP		55
#define	DEF_PAST_TIME		1800

#define MAX_PAST_TEMP		70
#define MIN_PAST_TEMP		40

#define	WATER_VOL		1.5F
#define	MILK_VOL		0.1F
#define	K1	(WATER_VOL / MILK_VOL)
#define	K2	(1.0 - K1)

#define MIN_POWER 		0
#define MAX_POWER		100

#define PWM_PIN_PORT		PORTB
#define PWM_PIN_DDR			DDRB
#define PWM_PIN				1

#define OUT1_PIN_PORT		PORTD
#define OUT1_PIN_DDR		DDRD
#define OUT1_PIN			6

#define OUT2_PIN_PORT		PORTD
#define OUT2_PIN_DDR		DDRD
#define OUT2_PIN			5

#define TEST_PIN_PORT		PORTC
#define TEST_PIN_DDR		DDRC
#define TEST_PIN			0

#define BUT_DEC	BUT_1_ID
#define BUT_ENT	BUT_2_ID
#define BUT_INC	BUT_3_ID

#define ACC_BITS			4
#define AccSizeMacro(val)	(1 << val)

uint8_t ButtonCode, ButtonEvent;
float TempVelocity = 0.0;
uint16_t PastTimeCounter = 0;

int16_t Buffer[AccSizeMacro(ACC_BITS)];

enum ModeList {
	StartModeList,
	Pasteurizer,
	HeatSpeedControl,
	ManualControl,
	EndModeList
};

enum PasteurizerActionEnum {
	PasteurizerHeating,
	PasteurizerDelay
};

enum HeatState {
	HeatShirt,
	WaitTempAreEqual,
	HeatMilk
};

enum TemperaturesName {
	Shirt,
	Milk
};

struct Parametr {
	uint8_t Version;
	uint8_t PastTemperature;
	uint8_t HSCTemperature;
	uint16_t PasteurizerTime;
} Parametr;

struct Flag {
	uint8_t TempMes : 1;
	uint8_t CalcVelocity : 1;
	uint8_t HSCEn : 1;
	uint8_t PastEn : 1;
	uint8_t ManEn : 1;
	uint8_t SaveParametr : 1;
	uint8_t SendEn : 1;
	/*uint8_t xxxx : 1;*/
} Flag;

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long constrain(long val, long min, long max) {
	if(val < min) return min;
	else if(val > max) return max;
	else return val;
}

float constrainF(float val, float min, float max) {
	if(val < min) return min;
	else if(val > max) return max;
	else return val;
}

volatile uint8_t PWMValue = 0;

uint8_t set_pwm(uint8_t val) {
	if(val <= 100) PWMValue = val;
	return PWMValue;
}

int16_t float_window(int16_t Value) {
	static uint16_t Index = 0;
	static int64_t Summ = 0;

	Summ += Value;

	Summ -= Buffer[Index];

	Buffer[Index++] = Value;

	if (Index == AccSizeMacro(ACC_BITS)) {
		Index = 0;
	}

	return (Summ >> ACC_BITS);
}

void save_eeprom() {
	cli();
	eeprom_update_block((uint8_t*)&Parametr, 0, sizeof(Parametr));
	sei();
}

void init_eeprom() {
	Parametr.Version = VERSION;
	Parametr.HSCTemperature = 50;
	Parametr.PastTemperature = DEF_PAST_TEMP;
	Parametr.PasteurizerTime = DEF_PAST_TIME;
	save_eeprom();
}

void get_but() {
	ButtonCode = BUT_GetBut();
	if(ButtonCode) {
		ButtonEvent = BUT_GetBut();
	}
}

void port_init() {
	SetBitVal(PWM_PIN_DDR, PWM_PIN, 1);
	SetBitVal(PWM_PIN_PORT, PWM_PIN, 0);
	SetBitVal(OUT1_PIN_DDR, OUT1_PIN, 1);
	SetBitVal(OUT1_PIN_PORT, OUT1_PIN, 0);
	SetBitVal(OUT2_PIN_DDR, OUT2_PIN, 1);
	SetBitVal(OUT2_PIN_PORT, OUT2_PIN, 0);
	// TEST PIN INIT
	SetBitVal(TEST_PIN_DDR, TEST_PIN, 1);
	SetBitVal(TEST_PIN_PORT, TEST_PIN, 0);
}

#define K_PROP			20.0	//2.85f
#define MAX_TEMP_SPEED	0.8f

uint8_t calc_power(float curTemp, float setTemp) {
	int8_t lPower;

	lPower = (uint8_t)((setTemp - curTemp) * K_PROP);

	if(lPower < 0) lPower = 0;

	return lPower;
}

uint8_t calc_spower(float curSpeed) {
	int16_t lPower;

	lPower = (uint16_t)(((MAX_TEMP_SPEED - curSpeed) / MAX_TEMP_SPEED) * 100.0);

	if(lPower > 100) lPower = 100;
	if(lPower < 0) lPower = 0;

	return lPower;
}

void timer1_init() {
    //  Timer 1 Initialization
    TCCR1A = 0x00;
    TCCR1B = ( 1 << CS12 ) | ( 0 << CS11 ) | ( 0 << CS10 ); // ќдна секунда
    TCNT1 = 0x0000;
    OCR1A = 0x7A11;
    OCR1B = 0x0000;
    ICR1 = 0x0000;
}

void timer2_init() {
	//  Timer 2 Initialization
	TCCR2 = ( 1 << CS22 ) | ( 0 << CS21 ) | ( 0 << CS20 );
	ASSR = 0x00;
	TCNT2 = 0x00;
	OCR2 = 0x7C;
}

uint8_t PID(float Set, float Value, float Kp, float Ki, float Kd)
{
	float E_pid;

	float Pr;
	float Integ = 0;
	float Df;
	static float E_pid_old;

	int16_t PWR;

	E_pid = (Set - Value); // *10 так как диапазон выходных данных до 1000, что бы получить мощность с дискретностью 0,1%
	Pr = Kp * E_pid; // пропорциональна€ составл€юща€
	Pr = constrainF( Pr, (-100.0), (100.0) ); // ограничиваем +/-1000

	if (Ki>0) {
		Integ = Integ + (E_pid / Ki); // интегральна€ составл€юща€
		Integ = constrainF( Integ, (MIN_POWER * 1.0), (MAX_POWER * 1.0) ); // ограничиваем интегральную составл€ющую 0..100%
	}
	else {
		Integ = 0;
	}
	Df = Kd * ( E_pid - E_pid_old ); // дифференциальна€ составл€юща€
	Df = constrainF( Df, (-100.0), (100.0) ); // ограничиваем +/-100
	E_pid_old = E_pid;

	PWR = (int16_t)(Pr + Integ + Df); // считаем текущую мощность

	PWR = constrain( PWR, (MIN_POWER * 1), (MAX_POWER * 1)); // ограничиваем текущую мощность 0..100%

	return PWR;
}

#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

void USART_Init( unsigned int ubrr )
{
	/* Set baud rate */
	UBRRH = (unsigned char)( ubrr >> 8 );
	UBRRL = (unsigned char)ubrr;
	/* Enable receiver and transmitter */
	UCSRB = (1 << TXCIE) | (1 << RXCIE)| (1 << TXEN) | (1 << RXEN);
	/* Set frame format: 8data, 1stop bit */
	UCSRC = (1 << URSEL) | (0 << USBS) | (3 << UCSZ0);
}

#define START_BYTE 	0x0A;
#define STOP_BYTE 	0x0D;

struct Packet
{
	unsigned char	Data[16];
	//unsigned char  	StoptByte;
} Packet;

uint8_t packetSize = 0;

void send_packet(uint8_t strl) {
	packetSize = strl + 2;
	Packet.Data[strl] = 0x0D;
	Packet.Data[strl + 1] = 0x0A;
	// «апись стартового байта в регистр UDR
	UDR = 0x09;	//Packet.Data[0];
}


ISR(USART_TXC_vect)
{
	uint8_t *Pointer = (uint8_t *)&(Packet.Data);

	static unsigned char TxIndex = 0;

	if (TxIndex < packetSize /*sizeof(Packet) - 1*/) {
		UDR = *(Pointer + TxIndex);
		TxIndex++;
	}
	else TxIndex = 0;
}

ISR(TIMER2_COMP_vect) {
	TCNT2 = 0x00;

	static uint8_t PWMCounter = 0;
	static uint8_t Cnt = 0;
	static uint8_t TempCnt = 0;

	if(++Cnt == 10) {
		Cnt = 0;
		if(PWMValue <= PWMCounter) {
			SetBitVal(PWM_PIN_PORT, PWM_PIN, 0);
		}
		else {
			SetBitVal(PWM_PIN_PORT, PWM_PIN, 1);
		}

		if(++PWMCounter == 100) {
			PWMCounter = 0;
		}
	}

	if(++TempCnt == 100) {
		TempCnt = 0;
		Flag.TempMes = 1;
	}

	InvBit(TEST_PIN_PORT, TEST_PIN);
	BUT_Poll();
}

// —екундный таймер
ISR(TIMER1_COMPA_vect)
{
	TCNT1 = 0x0000;

	if(PastTimeCounter > 0) PastTimeCounter--;

#if !INT0_EN
	static uint16_t Cnt = 0;

	if(++Cnt == (CALC_INTERVAL * 1)) {
		Cnt = 0;
		Flag.CalcVelocity = 1;
	}
	Flag.SendEn = 1;
#endif
}

char Text[16];

int main() {
	float TmpTemp = 0.0;
	float TempForPID = 0.0;
	float Temperature[2] = {0.0, 0.0};
	uint8_t Mode = Pasteurizer;
	uint8_t HeatState = HeatShirt;
	uint8_t PasteurizerAction = PasteurizerHeating;
	uint8_t FlagActivity = 0;

	ADS1115_Init(ADS1015_ADDRESS);
	ADS1115_setGain(GAIN_FOUR);

	USART_Init(MYUBRR);
	port_init();

	eeprom_read_block((uint8_t*)&Parametr, 0, sizeof(Parametr));

	if(Parametr.Version == 0xFF) {
		init_eeprom();
	}

	BUT_Init();
	timer1_init();
	timer2_init();
	LCD_Init();

	//i2c_Init();
	//uint8_t h, m, s;
	//rtc_init(0, 1, 0);
	//rtc_get_time( (uint8_t*)&h, (uint8_t*)&m, (uint8_t*)&s);
	//rtc_set_time(22, 1, 35);
	/*while(s != 0) {
		_delay_ms(500);
		rtc_get_time( (uint8_t*)&h, (uint8_t*)&m, (uint8_t*)&s);
		LCD_Goto(0, 0);
		sprintf(Text, "    %02d:%02d:%02d    ", h, m, s);
		LCD_SendStr(Text);
	}*/

	TIMSK = (1 << OCIE1A) | (1 << OCIE2) ;

	sei();

	Flag.CalcVelocity = 1;

	while(1) {
		get_but();
		FlagActivity = Flag.HSCEn + Flag.ManEn + Flag.PastEn;

		if(Flag.SaveParametr) {
			Flag.SaveParametr = 0;
			save_eeprom();
		}

		if(Flag.TempMes == 1) {
			Flag.TempMes = 0;
			Temperature[Shirt] = ADS1115_readADC_SingleEnded(Shirt) * 0.003125;
			Temperature[Milk] = ADS1115_readADC_SingleEnded(Milk) * 0.003125;

			if(Flag.CalcVelocity) {
				Flag.CalcVelocity = 0;

				if((Temperature[0] - TmpTemp) != Temperature[0]) {
					//TempVelocity = (Temperature[0] - TmpTemp) * FACTOR;
					TempVelocity = (float)(float_window((uint16_t)((Temperature[0] - TmpTemp) * FACTOR * 100.0))) / 100.0;	// —корость с усреднением
				}
				TmpTemp = Temperature[0];
			}


			/* ѕастеризаци€ */
			if(Flag.PastEn) {
				switch(HeatState) {
					case HeatShirt : {
						TempForPID = Temperature[Shirt];
						if(Temperature[Shirt] >= (float)Parametr.PastTemperature) {
							HeatState = WaitTempAreEqual;
						}
					} break;
					case WaitTempAreEqual : {
						TempForPID = Temperature[Shirt];
						// TempForPID = (Temperature[Shirt] * K1) + (Temperature[Milk] * K2);
						if(Temperature[Milk] >= Temperature[Shirt]) {
							HeatState = HeatMilk;
						}
					} break;
					case HeatMilk : {
						// TempForPID = Temperature[Shirt];
						TempForPID = (Temperature[Shirt] * K1) + (Temperature[Milk] * K2);
					} break;
					default : break;
				}

				PWMValue = PID(Parametr.PastTemperature, TempForPID, 20.0, 10.0, 2.0);
			}

			if(Flag.SendEn == 1) {
				Flag.SendEn = 0;
				for(uint8_t i = 0; i < sizeof(Text); i++) Text[i] = 0;
				sprintf(Text, "%d;%d;%d", (uint16_t)(Temperature[Shirt] * 10.0), (uint16_t)(Temperature[Milk] * 10.0), PWMValue);
				memcpy((uint8_t *)&(Packet.Data), Text, strlen(Text));
				send_packet(strlen(Text));
			}
		}

		if((FlagActivity == 0) && (ButtonCode == BUT_INC) && (ButtonEvent == BUT_RELEASED_LONG_CODE)) {
			if(++Mode == EndModeList) {
				Mode = StartModeList + 1;
			}
			LCD_Clear();
		}

		if((FlagActivity == 0) && (ButtonCode == BUT_DEC) && (ButtonEvent == BUT_RELEASED_LONG_CODE)) {
			if(--Mode == StartModeList) {
				Mode = EndModeList - 1;
			}
			LCD_Clear();
		}

		if((ButtonCode == BUT_ENT) && (ButtonEvent == BUT_PRESSED_CODE)) {
			switch(Mode) {
				case Pasteurizer : Flag.PastEn = 1; PasteurizerAction = PasteurizerHeating; HeatState = HeatShirt; break;
				case HeatSpeedControl : Flag.HSCEn = 1; break;
				case ManualControl : Flag.ManEn = 1; break;
				default : {
					Flag.PastEn = 0;
					Flag.HSCEn = 0;
					Flag.ManEn = 0;
				}; break;
			}
			Flag.SaveParametr = 1;
		}

		if((ButtonCode == BUT_ENT) && (ButtonEvent == BUT_DOUBLE_CLICK_CODE)) {
			Flag.PastEn = 0;
			Flag.HSCEn = 0;
			Flag.ManEn = 0;
			PWMValue = 0;
			Flag.SaveParametr = 1;
		}

		switch (Mode) {
			case Pasteurizer : {
				if((FlagActivity == 0) && (ButtonCode == BUT_INC) && (ButtonEvent == BUT_PRESSED_CODE)) {
					if(++Parametr.PastTemperature > MAX_PAST_TEMP) Parametr.PastTemperature = MAX_PAST_TEMP;
				}
				if((FlagActivity == 0) && (ButtonCode == BUT_DEC) && (ButtonEvent == BUT_PRESSED_CODE)) {
					if(--Parametr.PastTemperature < MIN_PAST_TEMP) Parametr.PastTemperature = MIN_PAST_TEMP;
				}

				LCD_Goto(0, 0);
				sprintf(Text, "%s %4.1f %4.1f %3d%%", ((!Flag.PastEn)?("p"):("P")), Temperature[Shirt], Temperature[Milk], PWMValue);
				LCD_SendStr(Text);

				LCD_Goto(0, 1);
				sprintf(Text, "t %2d   %4.1f   %d", Parametr.PastTemperature, TempForPID, HeatState);
				LCD_SendStr(Text);

				if(Flag.PastEn) {
					if(PasteurizerAction == PasteurizerHeating) {
						if(Temperature[Milk] >= Parametr.PastTemperature) {
							PasteurizerAction = PasteurizerDelay;
							PastTimeCounter = Parametr.PasteurizerTime;
						}
					}
					if(PasteurizerAction == PasteurizerDelay) {
						if(PastTimeCounter == 0) {
							set_pwm(0);
							PastTimeCounter = 0;
							LCD_Clear();
							Flag.PastEn = 0;
							Flag.SaveParametr = 1;
						}
					}
				}
			} break;

			case HeatSpeedControl : {
				LCD_Goto(0, 0);
				sprintf(Text, "H t:%4.1f  tH:%2d%s", Temperature[0], Parametr.HSCTemperature, ((Flag.HSCEn)?("*"):(" ")));
				LCD_SendStr(Text);

				LCD_Goto(0, 1);
				sprintf(Text, "P:%3d%%  V:%5.2f ", PWMValue, TempVelocity);
				LCD_SendStr(Text);

				if(Flag.HSCEn) {
					set_pwm(calc_spower(TempVelocity));

					if(Temperature[0] >= (float)Parametr.HSCTemperature) {
						Flag.HSCEn = 0;
						set_pwm(0);
					}
				}
			} break;

			case ManualControl : {
				if(Flag.ManEn) {
					if((ButtonCode == BUT_INC) && (ButtonEvent == BUT_PRESSED_CODE)) {
						if(PWMValue < 100) {
							PWMValue++;
						}
					}

					if((ButtonCode == BUT_DEC) && (ButtonEvent == BUT_PRESSED_CODE)) {
						if(PWMValue > 0) {
							PWMValue--;
						}
					}
				}

				sprintf(Text, "M t:%4.1f       %s", Temperature[0], ((Flag.ManEn)?("*"):(" ")));
				LCD_Goto(0, 0);
				LCD_SendStr(Text);

				sprintf(Text, "P:%3d%%  V:%5.2f", PWMValue, TempVelocity);
				LCD_Goto(0, 1);
				LCD_SendStr(Text);
			} break;

			default: break;
		}
	}
	return 0;
}

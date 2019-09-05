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
#include <avr/wdt.h>
#include "bits_macros.h"
#include "lcd_lib_2.h"
#include "buttons.h"
#include "ADS1115.h"
#include "ds1307.h"
#include "conv.h"

#define VERSION				10

#define EMUL				0

#define LOW					0
#define HIGH				1

#define CALC_INTERVAL		1
#define FACTOR				60.0 / (float)CALC_INTERVAL

#define	DEF_PAST_TEMP		65
#define	DEF_PAST_TIME		30		// минуты
#define DEF_MILK_VOL		30
#define DEF_TIME_HEATING	480
#define DEF_TIME_TEMP		95

#define MAX_PAST_TEMP		70
#define MIN_PAST_TEMP		40

#define	WATER_VOL			13.0F
#define	MILK_VOL			30.0F
#define	K1					(WATER_VOL / MILK_VOL)
#define	K2					(1.0 - K1)

#define MIN_POWER 		0
#define MAX_POWER		100
#define HEATING_POWER	3000.0F
#define HEATING_EFF		0.98F

#define PWM_PIN_PORT		PORTB
#define PWM_PIN_DDR			DDRB
#define PWM_PIN				1

#define OUT1_PORT		PORTD
#define OUT1_DDR		DDRD
#define OUT1_PIN		6

#define OUT2_PORT		PORTD
#define OUT2_DDR		DDRD
#define OUT2_PIN		5

#define TEST_PORT		PORTC
#define TEST_DDR		DDRC
#define TEST_PIN		0

#define BEEP_PORT		PORTB
#define BEEP_DDR		DDRB
#define BEEP_PIN		0

#define BUT_DEC	BUT_1_ID
#define BUT_ENT	BUT_2_ID
#define BUT_INC	BUT_3_ID

// #define ACC_BITS			4
// #define AccSizeMacro(val)	(1 << val)

enum ModeList {
	StartModeList,
	ShowTime,
	Pasteurizer,
	TimeControl,
	ManualControl,
	PIDSettings,
	FWUpgrade,
	EndModeList
};

enum PasteurizerActionEnum {
	PasteurizerHeating,
	PasteurizerDelay
};

enum PasteurizerParamName {
	PasteurizerNone = -1,
	PasteurizerTemp,
	PasteurizerTime
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

enum PIDCtrl {
	PID_None_Edit,
	P_Edit,
	I_Edit,
	D_Edit
};

enum PIDCoeffEnum {
	PCoeff,
	ICoeff,
	DCoeff
};

enum TimeHeatParam {
	THNone = -1,
	THTemperature,
	THTime,
	THVolume
};

enum MinMaxEnum {
	MMHTTemp,
	MMHTTime,
	MMHTVolume,
	MMHour,
	MMMinute,
	MMSecond,
	MMPastTemp,
	MMPastTime
};

enum MinMax {
	Min,
	Max
};

enum TimeArrName {
	TENone = -1,
	Hour,
	Minute,
	Second
};

uint8_t ButtonCode, ButtonEvent;
uint16_t PastTimeCounter = 0;
uint16_t BeepTime = 0, BeepCycle;
uint8_t PasteurizerAction = PasteurizerHeating;
// int16_t Buffer[AccSizeMacro(ACC_BITS)];

struct Parametr {
	uint8_t Version;
	int16_t PasteurizerParam[2];
	uint16_t THParam[3];
	int16_t MinMaxParam[2][8];
	float PIDCoeff[3];
} Parametr;

struct Flag {
	uint8_t TempMes : 1;
	uint8_t PastEn : 1;
	uint8_t ManEn : 1;
	uint8_t TimeEn : 1;
	uint8_t SaveParametr : 1;
	uint8_t SendEn : 1;
	uint8_t EditParam : 1;
	uint8_t Mute : 1;
	/*uint8_t xxxx : 1;*/
} Flag;

volatile uint8_t PWMValue = 0;

uint8_t set_pwm(uint8_t val) {
	if(val <= 100) PWMValue = val;
	return PWMValue;
}

/*int16_t float_window(int16_t Value) {
	static uint16_t Index = 0;
	static int64_t Summ = 0;

	Summ += Value;

	Summ -= Buffer[Index];

	Buffer[Index++] = Value;

	if (Index == AccSizeMacro(ACC_BITS)) {
		Index = 0;
	}

	return (Summ >> ACC_BITS);
}*/

void save_eeprom() {
	cli();
	eeprom_update_block((uint8_t*)&Parametr, 0, sizeof(Parametr));
	sei();
}

void init_eeprom() {
	Parametr.Version = VERSION;
	Parametr.THParam[THTemperature] = DEF_TIME_TEMP;
	Parametr.THParam[THTime] = DEF_TIME_HEATING;
	Parametr.THParam[THVolume] = DEF_MILK_VOL;

	Parametr.MinMaxParam[Min][MMHTTemp] = 50;
	Parametr.MinMaxParam[Max][MMHTTemp] = 95;

	Parametr.MinMaxParam[Min][MMHTTime] = 1;
	Parametr.MinMaxParam[Max][MMHTTime] = 960;

	Parametr.MinMaxParam[Min][MMHTVolume] = 10;
	Parametr.MinMaxParam[Max][MMHTVolume] = 30;

	Parametr.MinMaxParam[Min][MMHour] = 0;
	Parametr.MinMaxParam[Max][MMHour] = 23;
	Parametr.MinMaxParam[Min][MMMinute] = 0;
	Parametr.MinMaxParam[Max][MMMinute] = 59;
	Parametr.MinMaxParam[Min][MMSecond] = 0;
	Parametr.MinMaxParam[Max][MMSecond] = 59;

	Parametr.MinMaxParam[Min][MMPastTemp] = 40;
	Parametr.MinMaxParam[Max][MMPastTemp] = 95;
	Parametr.MinMaxParam[Min][MMPastTime] = 0;
	Parametr.MinMaxParam[Max][MMPastTime] = 99;

	Parametr.PasteurizerParam[PasteurizerTemp] = DEF_PAST_TEMP;
	Parametr.PasteurizerParam[PasteurizerTime] = DEF_PAST_TIME;
	Parametr.PIDCoeff[PCoeff] = 10.0;
	Parametr.PIDCoeff[ICoeff] = 0.5;
	Parametr.PIDCoeff[DCoeff] = 2.0;
	save_eeprom();
}

void get_but() {
	ButtonCode = BUT_GetBut();
	if(ButtonCode) {
		ButtonEvent = BUT_GetBut();
	}
}

#if EMUL
#define SHT_INC_BUT	1
#define SHT_DEC_BUT	2
#define MIT_INC_BUT	3
#define MIT_DEC_BUT	4
#endif

void port_init() {
	SetBitVal(PWM_PIN_DDR, PWM_PIN, HIGH);
	SetBitVal(PWM_PIN_PORT, PWM_PIN, LOW);
	SetBitVal(OUT1_DDR, OUT1_PIN, HIGH);
	SetBitVal(OUT1_PORT, OUT1_PIN, LOW);
	SetBitVal(OUT2_DDR, OUT2_PIN, HIGH);
	SetBitVal(OUT2_PORT, OUT2_PIN, LOW);
	// TEST PIN INIT
	SetBitVal(TEST_DDR, TEST_PIN, HIGH);
	SetBitVal(TEST_PORT, TEST_PIN, LOW);

	// BEEP PIN INIT
	SetBitVal(BEEP_DDR, BEEP_PIN, HIGH);
	SetBitVal(BEEP_PORT, BEEP_PIN, LOW);
#if EMUL
	SetBitVal(DDRC, SHT_INC_BUT, 0);
	SetBitVal(PORTC, SHT_INC_BUT, 1);
	SetBitVal(DDRC, SHT_DEC_BUT, 0);
	SetBitVal(PORTC, SHT_DEC_BUT, 1);
	SetBitVal(DDRC, MIT_INC_BUT, 0);
	SetBitVal(PORTC, MIT_INC_BUT, 1);
	SetBitVal(DDRC, MIT_DEC_BUT, 0);
	SetBitVal(PORTC, MIT_DEC_BUT, 1);
#endif
}

uint8_t calc_time_pwm(float temperature, uint16_t time, uint8_t vol) {

	uint32_t lpwm;
	float lp;

	lp = (((float)vol + WATER_VOL) * 1.163 * (Parametr.THParam[THTemperature] - temperature)) / (((float)time / 60.0) * HEATING_EFF);
	lpwm = lp / HEATING_POWER * 100.0;
	if (lpwm > 100) lpwm = 100;
	return lpwm;
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
void send_packet(char *pack) {
	uint8_t strl = strlen(pack);
	memcpy((uint8_t *)&(Packet.Data), pack, strl);
	packetSize = strl + 2;
	Packet.Data[strl] = 0x0D;
	Packet.Data[strl + 1] = 0x0A;
	// «апись стартового байта в регистр UDR
	UDR = 0x01;	//Packet.Data[0];
}

void beep(uint16_t time, uint8_t cycle) {
	if(!Flag.Mute) {
		BeepTime = time;
		BeepCycle = cycle;
	}
}

ISR(USART_TXC_vect)
{
	uint8_t *Pointer = (uint8_t *)&(Packet.Data);

	static unsigned char TxIndex = 0;

	if (TxIndex < packetSize) {
		UDR = *(Pointer + TxIndex);
		TxIndex++;
	}
	else TxIndex = 0;
}

// 1ms timer
ISR(TIMER2_COMP_vect) {
	TCNT2 = 0x00;

	static uint8_t PWMCounter = 0;
	static uint8_t Cnt = 0;
	//static uint8_t TempCnt = 0;
	static uint16_t lBeepTime = 0;
	static uint16_t lCyclePause = 0;

	wdt_reset();

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

	/*if(++TempCnt == 1000) {
		TempCnt = 0;
		Flag.TempMes = 1;
	}*/

	BUT_Poll();

	if(BeepCycle > 0) {
		if((++lBeepTime < BeepTime) && (lCyclePause == 0)) {
			SetBitVal(BEEP_PORT, BEEP_PIN, HIGH);
		}
		else {
			SetBitVal(BEEP_PORT, BEEP_PIN, LOW);
			if(++lCyclePause >= BeepTime) {
				lCyclePause = 0;
				lBeepTime = 0;
				BeepCycle--;
			}
		}
	}
}

// 1s timer
ISR(TIMER1_COMPA_vect)
{
	TCNT1 = 0x0000;
	if((PastTimeCounter > 0) && (PasteurizerAction == PasteurizerDelay)) {
		PastTimeCounter--;
	}
	Flag.SendEn = 1;
	Flag.TempMes = 1;
}

char Text[16];
char tmpText[3][6];

int main() {
	//float TmpTemp = 0.0;
	float TempForPID = 0.0;
	float Temperature[2] = {30.0, 25.0};
	uint8_t PIDEditMode = PID_None_Edit;
	int8_t THMode = THNone;
	int8_t PastEditMode = PasteurizerNone;

	uint8_t Mode = ShowTime;
	uint8_t HeatState = HeatShirt;
	uint8_t FlagActivity = 0;
	int8_t TimeEdit = TENone;

	// i2c_Init();

	ADS1115_Init(ADS1015_ADDRESS);
	ADS1115_setGain(GAIN_FOUR);

	USART_Init(MYUBRR);
	port_init();
	BUT_Init();
	LCD_Init();

	eeprom_read_block((uint8_t*)&Parametr, 0, sizeof(Parametr));

	if((Parametr.Version == 0xFF) || (BitIsClear(PINC, 7))) {
		LCD_Goto(0, 0);
		LCD_SendStr(" -Init  EEPROM- ");
		while(BitIsClear(PINC, 7));
		init_eeprom();
		_delay_ms(500);
		LCD_Clear();
	}

	timer1_init();
	timer2_init();

	int8_t Time[3];
	rtc_init(0, 1, 0);
	// rtc_set_time(0, 0, 0);
	rtc_get_time( (uint8_t*)&Time[Hour], (uint8_t*)&Time[Minute], (uint8_t*)&Time[Second]);

	TIMSK = (1 << OCIE1A) | (1 << OCIE2) ;

	wdt_reset();
	wdt_enable(WDTO_1S);

	sei();

	beep(250, 3);

	while(1) {
		get_but();
		FlagActivity = Flag.ManEn + Flag.PastEn + Flag.TimeEn;

		if(Flag.SaveParametr) {
			Flag.SaveParametr = 0;
			save_eeprom();
		}

		/*  нопки ************************************************************************************************************************/
		if((ButtonCode == BUT_ENT) && (ButtonEvent == BUT_RELEASED_CODE)) {
			switch(Mode) {
				case Pasteurizer : {
					SetBitVal(OUT1_PORT, OUT1_PIN, HIGH);
					Flag.PastEn = 1;
					PasteurizerAction = PasteurizerHeating;
					HeatState = HeatShirt;
				} break;
				case TimeControl : {
					SetBitVal(OUT1_PORT, OUT1_PIN, HIGH);
					Flag.TimeEn = 1;
					HeatState = HeatShirt;
					PWMValue = calc_time_pwm(Temperature[Shirt], Parametr.THParam[THTime], Parametr.THParam[THVolume]);
					} break;
				case ManualControl : {
					Flag.ManEn = 1;
					SetBitVal(OUT1_PORT, OUT1_PIN, HIGH);
				} break;
				default : {
					Flag.PastEn = 0;
					Flag.ManEn = 0;
					Flag.TimeEn = 0;
				}; break;
			}
			Flag.SaveParametr = 1;
		}

		if((ButtonCode == BUT_ENT) && (ButtonEvent == BUT_DOUBLE_CLICK_CODE)) {
			Flag.PastEn = 0;
			Flag.ManEn = 0;
			Flag.TimeEn = 0;
			PWMValue = 0;
			SetBitVal(OUT1_PORT, OUT1_PIN, LOW);
			Flag.SaveParametr = 1;
		}
		/************************************************************************************************************************  нопки */

#if EMUL
		if(BitIsClear(PINC, SHT_INC_BUT)) {
			while(BitIsClear(PINC, SHT_INC_BUT));
			Temperature[Shirt] += 1.0;
		}
		if(BitIsClear(PINC, SHT_DEC_BUT)) {
			while(BitIsClear(PINC, SHT_DEC_BUT));
			Temperature[Shirt] -= 1.0;
		}
		if(BitIsClear(PINC, MIT_INC_BUT)) {
			while(BitIsClear(PINC, MIT_INC_BUT));
			Temperature[Milk] += 1.0;
		}
		if(BitIsClear(PINC, MIT_DEC_BUT)) {
			while(BitIsClear(PINC, MIT_DEC_BUT));
			Temperature[Milk] -= 1.0;
		}
#endif
		if(Flag.TempMes == 1) {
			Flag.TempMes = 0;
#if !EMUL
			Temperature[Shirt] = ADS1115_readADC_SingleEnded(Shirt) * 0.003125;
			Temperature[Milk] = ADS1115_readADC_SingleEnded(Milk) * 0.003125;
#endif

			if(Flag.EditParam == 0) rtc_get_time( (uint8_t*)&Time[Hour], (uint8_t*)&Time[Minute], (uint8_t*)&Time[Second]);

			/* ”правление пастеризацией *********************************************************************************************/
			if(Flag.PastEn) {
				switch(HeatState) {
					case HeatShirt : {
						TempForPID = Temperature[Shirt];
						if(Temperature[Shirt] >= (float)Parametr.PasteurizerParam[PasteurizerTemp]) {
							HeatState = HeatMilk;
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
						TempForPID = Temperature[Milk];
						//TempForPID = (Temperature[Shirt] * K1) + (Temperature[Milk] * K2);
					} break;
					default : break;
				}

				PWMValue = PID((float)Parametr.PasteurizerParam[PasteurizerTemp], TempForPID, Parametr.PIDCoeff[PCoeff], Parametr.PIDCoeff[ICoeff], Parametr.PIDCoeff[DCoeff]);

				if(PasteurizerAction == PasteurizerHeating) {
					if(Temperature[Milk] >= Parametr.PasteurizerParam[PasteurizerTemp]) {
						PasteurizerAction = PasteurizerDelay;
						// PastTimeCounter = Parametr.PasteurizerParam[PasteurizerTime] * 60;
					}
				}

				if(PasteurizerAction == PasteurizerDelay) {
					if(PastTimeCounter == 0) {
						set_pwm(0);
						PastTimeCounter = Parametr.PasteurizerParam[PasteurizerTime] * 60;
						LCD_Clear();
						Flag.PastEn = 0;
						Flag.SaveParametr = 1;
					}
				}
			}
			/********************************************************************************************* ”правление пастеризацией */

			/* Ќарев по времени *****************************************************************************************************/
			if(Flag.TimeEn) {
				switch(HeatState) {
					case HeatShirt : {
						TempForPID = Temperature[Shirt];
						if(Temperature[Shirt] >= (float)Parametr.THParam[THTemperature]) {
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
						TempForPID = Temperature[Milk];
						//TempForPID = (Temperature[Shirt] * K1) + (Temperature[Milk] * K2);
						if(TempForPID >= Parametr.THParam[THTemperature]) {
							Flag.TimeEn = 0;
							PWMValue = 0;
						}
					} break;
					default : break;
				}
			}
			/***************************************************************************************************** Ќарев по времени */

			if(Flag.SendEn == 1) {
				Flag.SendEn = 0;
				for(uint8_t i = 0; i < sizeof(Text); i++) Text[i] = 0;
				float_to_str(tmpText[Shirt], Temperature[Shirt], 1);
				float_to_str(tmpText[Milk], Temperature[Milk], 1);
				sprintf(Text, "%s;%s;%d",tmpText[Shirt], tmpText[Milk], PWMValue);
				send_packet(Text);
			}
		}

		if((Flag.EditParam == 0) && (FlagActivity == 0) && (ButtonCode == BUT_INC) && (ButtonEvent == BUT_EV_HELD)) {
			if(++Mode == EndModeList) {
				Mode = StartModeList + 1;
			}
			if(Mode == Pasteurizer) {
				PastTimeCounter = Parametr.PasteurizerParam[PasteurizerTime] * 60;
			}
			LCD_Clear();
		}

		if((Flag.EditParam == 0) && (FlagActivity == 0) && (ButtonCode == BUT_DEC) && (ButtonEvent == BUT_EV_HELD)) {
			if(--Mode == StartModeList) {
				Mode = EndModeList - 1;
			}
			if(Mode == Pasteurizer) {
				PastTimeCounter = Parametr.PasteurizerParam[PasteurizerTime] * 60;
			}
			LCD_Clear();
		}

		switch (Mode) {
			case ShowTime : {
				sprintf(Text, "    %2d:%02d:%02d    ", Time[Hour], Time[Minute], Time[Second]);
				LCD_Goto(0, 0);
				LCD_SendStr(Text);

				if((ButtonCode == BUT_ENT) && (ButtonEvent == BUT_EV_HELD)) {
					if(++TimeEdit > Second) TimeEdit = TENone;

					if(TimeEdit != TENone) {
						Flag.EditParam = 1;
						LCD_Goto((TimeEdit * 3), 1);
						LCD_SendStr("    --");
					}
					else {
						Flag.EditParam = 0;
						LCD_Goto(0, 1);
						LCD_SendStr("                ");
						rtc_set_time(Time[Hour], Time[Minute], Time[Second]);
					}
				}

				if(Flag.EditParam == 1) {
					if((ButtonCode == BUT_INC) && (ButtonEvent == BUT_RELEASED_CODE)) {
						Time[TimeEdit]++;
						if(Time[TimeEdit] > Parametr.MinMaxParam[Max][TimeEdit + MMHour]) {
							Time[TimeEdit] = Parametr.MinMaxParam[Min][TimeEdit + MMHour];
						}
					}

					if((ButtonCode == BUT_DEC) && (ButtonEvent == BUT_RELEASED_CODE)) {
						Time[TimeEdit]--;
						if(Time[TimeEdit] < Parametr.MinMaxParam[Min][TimeEdit + MMHour]) {
							Time[TimeEdit] = Parametr.MinMaxParam[Max][TimeEdit + MMHour];
						}
					}
				}
			} break;

			case Pasteurizer : {
				if((ButtonCode == BUT_ENT) && (ButtonEvent == BUT_EV_HELD) && (FlagActivity == 0)) {
					if(++PastEditMode > PasteurizerTime) PastEditMode = PasteurizerNone;
					if(PastEditMode == PasteurizerNone) {
						Flag.EditParam = 0;
						Flag.SaveParametr = 1;
						PastTimeCounter = Parametr.PasteurizerParam[PasteurizerTime] * 60;
					}
					else Flag.EditParam = 1;
				}

				if((PastEditMode != PasteurizerNone) && (FlagActivity == 0)) {
					if((ButtonCode == BUT_INC) && (ButtonEvent == BUT_RELEASED_CODE)) {
						if(Parametr.PasteurizerParam[PastEditMode] < Parametr.MinMaxParam[Max][PastEditMode + 6]) Parametr.PasteurizerParam[PastEditMode]++;
					}

					if((ButtonCode == BUT_DEC) && (ButtonEvent == BUT_RELEASED_CODE)) {
						if(Parametr.PasteurizerParam[PastEditMode] > Parametr.MinMaxParam[Min][PastEditMode + 6]) Parametr.PasteurizerParam[PastEditMode]--;
					}
				}

				float_to_str(tmpText[Shirt], Temperature[Shirt], 1);
				float_to_str(tmpText[Milk], Temperature[Milk], 1);
				// мощность -----------------
				// t молока ------------    |
				// t рубашки -------   |    |
				// инд. режим --   |   |    |
				//             |   |   |    |
				sprintf(Text, "%s %4s %4s %3d%%", ((!Flag.PastEn)?("p"):("P")), tmpText[Shirt], tmpText[Milk], PWMValue);
				LCD_Goto(0, 0);
				LCD_SendStr(Text);

				float_to_str(tmpText[0], TempForPID, 1);
				if(PastEditMode == PasteurizerTime) sprintf(tmpText[1], "%2d:  ", Parametr.PasteurizerParam[PasteurizerTime]);
				else sprintf(tmpText[1], "%2d:%02d", PastTimeCounter / 60, PastTimeCounter % 60);
				// состо€ние --------------
				// t дл€ PID регул€тора    |
				// t пастеризации     |    |
				//                |   |    |
				sprintf(Text, "%s:%2d %4s  %s", ((PastEditMode == PasteurizerTemp)?("T"):("t")), Parametr.PasteurizerParam[PasteurizerTemp], tmpText[0], tmpText[1]);
				LCD_Goto(0, 1);
				LCD_SendStr(Text);
			} break;

			case TimeControl : {
				float_to_str(tmpText[Shirt], Temperature[Shirt], 1);
				float_to_str(tmpText[Milk], Temperature[Milk], 1);
				// мощность ---------------
				// t молока ------------  |
				// t рубашки -------   |  |
				// инд. режим --   |   |  |
				//             |   |   |  |
				sprintf(Text, "%s %4s %4s %3d%%", ((!Flag.TimeEn)?("t"):("T")), tmpText[Shirt], tmpText[Milk], PWMValue);
				LCD_Goto(0, 0);
				LCD_SendStr(Text);

				if((ButtonCode == BUT_ENT) && (ButtonEvent == BUT_EV_HELD) && (FlagActivity == 0)) {
					if(++THMode > THVolume) THMode = THNone;
					if(THMode == THNone) {
						Flag.EditParam = 0;
						Flag.SaveParametr = 1;
					}
					else Flag.EditParam = 1;
				}

				if((THMode != THNone) && (FlagActivity == 0)) {
					if((ButtonCode == BUT_INC) && (ButtonEvent == BUT_RELEASED_CODE)) {
						if(Parametr.THParam[THMode] < Parametr.MinMaxParam[Max][THMode]) Parametr.THParam[THMode]++;
					}

					if((ButtonCode == BUT_DEC) && (ButtonEvent == BUT_RELEASED_CODE)) {
						if(Parametr.THParam[THMode] > Parametr.MinMaxParam[Min][THMode]) Parametr.THParam[THMode]--;
					}
				}

				sprintf(Text, "%s:%d %s:%3d %s:%2d", ((THMode == THTemperature)?("T"):("t")),
													  Parametr.THParam[THTemperature],
													  ((THMode == THTime)?("Ti"):("ti")),
													  Parametr.THParam[THTime],
													  ((THMode == THVolume)?("V"):("v")),
													  Parametr.THParam[THVolume]);
				LCD_Goto(0, 1);
				LCD_SendStr(Text);
			} break;

			case ManualControl : {
				if(Flag.ManEn) {
					if((ButtonCode == BUT_INC) && (ButtonEvent == BUT_RELEASED_CODE)) {
						if(PWMValue < 100) {
							PWMValue++;
						}
					}
					if((ButtonCode == BUT_DEC) && (ButtonEvent == BUT_RELEASED_CODE)) {
						if(PWMValue > 0) {
							PWMValue--;
						}
					}
				}

				float_to_str(tmpText[Shirt], Temperature[Shirt], 1);
				float_to_str(tmpText[Milk], Temperature[Milk], 1);
				// мощность -----------------
				// t молока ------------    |
				// t рубашки -------   |    |
				// инд. режим --   |   |    |
				//             |   |   |    |
				sprintf(Text, "%s %4s %4s %3d%%", ((!Flag.ManEn)?("m"):("M")), tmpText[Shirt], tmpText[Milk], PWMValue);
				LCD_Goto(0, 0);
				LCD_SendStr(Text);
			} break;

			case PIDSettings : {
				if((ButtonCode == BUT_ENT) && (ButtonEvent == BUT_EV_HELD)) {
					if(++PIDEditMode > D_Edit) PIDEditMode = PID_None_Edit;
					if(PIDEditMode == PID_None_Edit) {
						Flag.EditParam = 0;
						Flag.SaveParametr = 1;
					}
					else Flag.EditParam = 1;
				}

				if(PIDEditMode != PID_None_Edit) {
					if((ButtonCode == BUT_INC) && (ButtonEvent == BUT_RELEASED_CODE)) {
						if(Parametr.PIDCoeff[PIDEditMode - 1] < 20.0) {
							Parametr.PIDCoeff[PIDEditMode - 1] += 0.1;
						}
					}
					if((ButtonCode == BUT_DEC) && (ButtonEvent == BUT_RELEASED_CODE)) {
						if(Parametr.PIDCoeff[PIDEditMode - 1] > 0.0) {
							Parametr.PIDCoeff[PIDEditMode - 1] -= 0.1;
						}
					}
				}

				float_to_str(tmpText[PCoeff], Parametr.PIDCoeff[PCoeff], 1);
				float_to_str(tmpText[ICoeff], Parametr.PIDCoeff[ICoeff], 1);
				float_to_str(tmpText[DCoeff], Parametr.PIDCoeff[DCoeff], 1);
				sprintf(Text, " %4s %4s %4s ", tmpText[PCoeff], tmpText[ICoeff], tmpText[DCoeff]);
				LCD_Goto(0, 0);
				LCD_SendStr(Text);

				sprintf(Text, "    %s    %s    %s ", ((PIDEditMode == P_Edit)?("P"):("p")),
											         ((PIDEditMode == I_Edit)?("I"):("i")),
											         ((PIDEditMode == D_Edit)?("D"):("d")));
				LCD_Goto(0, 1);
				LCD_SendStr(Text);

			} break;

			case FWUpgrade : {
				LCD_Goto(0, 0);
				LCD_SendStr("-- Upgrade FW --");
				LCD_Goto(0, 1);
				LCD_SendStr("Press <ENT> butt");

				if((ButtonCode == BUT_ENT) && (ButtonEvent == BUT_EV_HELD)) {
					LCD_Goto(0, 0);
					LCD_SendStr("-- Upgrade FW --");
					LCD_Goto(0, 1);
					LCD_SendStr("----------------");
					cli();
					while(1);
				}
			} break;

			default: break;
		}
	}
	return 0;
}


#include "stm32f3xx_hal.h"
#include "general_math.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"
#include "stdarg.h"
#include "arm_math.h"




extern uint32_t psk;
extern uint32_t Period;
extern uint32_t Pulse_1;
extern uint32_t Pulse_2;
extern uint32_t Pulse_3;
extern uint32_t Pulse_4;
extern uint32_t DeathTime_u;
extern uint32_t DeathTime;
extern int32_t  Overlap_u;
extern uint32_t Overlap;
extern uint32_t TimeLimit_u;
extern uint32_t TimeLimit;
extern uint32_t Tick;
extern uint8_t	Trig;
extern float 		BR;
extern uint8_t	State;
extern uint16_t  DataLenght;
extern uint8_t  Position;
extern uint8_t  Data[];
extern uint32_t f1;
extern uint32_t f2;
extern uint32_t BufferADC_1;
extern uint32_t BufferADC_2;
extern uint32_t BufferADC_3;
extern uint8_t  SoftStart;
extern uint8_t  mData[];
extern uint8_t	Mode;
extern uint16_t fgen;
extern DMA_HandleTypeDef hdma_usart1_tx;

uint8_t TransMode = 0;	// ����� �����������: 0 - �������� ���������, 1 - ���������������, 2 - ��������, 3 - ���������
uint8_t f_change = 0;

uint32_t ADC1buff;
uint32_t ADC2buff;
uint32_t ADC3buff;
uint32_t ADC4buff;

float U1rise1 = CONST;				
float U1fall1 = CONST;

float U2rise1 = CONST;				
float U2fall1 = CONST;	

float U3rise1 = CONST;				
float U3fall1 = CONST;	

float U4rise1 = CONST;				
float U4fall1 = CONST;	

float U1rise2 = CONST;				
float U1fall2 = CONST;

float U2rise2 = CONST;				
float U2fall2 = CONST;	

float U3rise2 = CONST;				
float U3fall2 = CONST;	

float U4rise2 = CONST;				
float U4fall2 = CONST;

float mean1 = 0;
float mean2 = 0;
float mean3 = 0;
float mean4 = 0;


uint8_t defcnt = 0;

// ������� � ��������
float C1;
float C2;
float C3;
float C4;

// ���������� �������� ������� � ��������
float C1corr;
float C2corr;
float C3corr;
float C4corr;

// ������������� ������
float L1;
float L2;
float L3;
float L4;

// ������������� ������
float R1;
float R2;
float R3;
float R4;

// �������� � ��������
float P1;
float P2;
float P3;
float P4;

// ������� ������������� ������
float R1a;
float R2a;
float R3a;
float R4a;

// ������� �������� � ��������
float P1a;
float P2a;
float P3a;
float P4a;

// ������������ �������� � ��������
float P1m;
float P2m;
float P3m;
float P4m;

// ���������� �� ������������ �������
float UL1;
float UL2;
float UL3;
float UL4;

// ���������� ������� �������
float Us1;
float Us2;
float Us3;
float Us4;

// ��� � ��������
float I1;
float I2;
float I3;
float I4;

// ������� ���
float I1a;
float I2a;
float I3a;
float I4a;

// ������������ ���
float I1m;
float I2m;
float I3m;
float I4m;

// ������
float T;

// ������������� �������������� ����������
float L0 = 3200e-9f;  // 40

// ���������� �������
float Us = 310; // �������� ���������� �������

	// ���������� �� ��������
float Us1 = 98;
float Us2 = 98;
float Us3 = 48;
float Us4 = 48;

	// ���������� ��� ����������� 
uint32_t DiagTime = 1;
uint32_t DiagCNT = 0;
uint32_t DiagPulse = 0;
uint8_t	 RiseDelay = 80;
uint8_t	 FallDelay = 80;
uint8_t	 PauseDelay = 80;

extern uint8_t	sec;
extern uint32_t	min;

uint8_t n = 0;
uint8_t n_IC = 0;
uint16_t	CCR1tmp = 0;
uint8_t n_stop = 0;

uint8_t diag = 0, m = 0, def = 0;

uint8_t CodeMode = 0;
uint8_t SendForm = 0;			// 0 - ������, 1 - �������, 2 - ������ � �������
uint8_t RepeatNum = 0;

extern uint8_t  SoftStart;
extern UART_HandleTypeDef huart1;

uint16_t freq;
uint32_t tmpData = 43690;
uint8_t  tmpComand = 0;

_Bool StartSend = 0;
_Bool StopDiag = 0;

extern uint32_t aCNT;
extern uint8_t  bCNT;

StatusSystem_t STATUS;
SettingParametrs_t SETUP;
UART2_Queue_Data UART2RecvData;

UART2Recv_t UART2_RecvType;

void CommandPrep(Cmd_Type cmd, const char fmt, ...);

	// ������ ������������� ���������
void TimingCalc(void)
	{
			// ����������� ������������ �������� � ������� ������
		switch(SoftStart)
			{
				case ON:
					if(n_stop == 0)	
					{
						if(TimeLimit < TimeLimit_u*Tick)
							TimeLimit += TimeLimit/2;
						if(TimeLimit >= TimeLimit_u*Tick)
							TimeLimit = TimeLimit_u*Tick;
					}
					if(n_stop >= 1)
					{
						if(TimeLimit <= TimeLimit_u*Tick)
							TimeLimit = TimeLimit/2;
						if(TimeLimit <= 10*Tick)
							TimeLimit = 10*Tick;
					}	
						break;
				case OFF:
					TimeLimit = TimeLimit_u*Tick;
					break;
			}
			
			
			// ������ ����� � ����������
		if((psk+2*DeathTime_u*Tick + Overlap_u*Tick >= TimeLimit) && (psk >= TimeLimit) && (psk-2*DeathTime_u*Tick-Overlap_u*Tick >= TimeLimit))	// ������� ���� ����������� �������
			{
				DeathTime =  psk-TimeLimit;
				//Overlap		=  psk-TimeLimit-2*DeathTime_u*Tick; 				//	�������� � ������������ ����� ����������
				Overlap		=  -psk+TimeLimit;														//	����������� ����� �������
			}
			else if(psk < 2*DeathTime_u*Tick+Overlap_u*Tick+10*Tick)																																										// ����������� ������� �������
			{
				psk=2*DeathTime_u*Tick+Overlap_u*Tick+10*Tick;
				DeathTime = 2*DeathTime_u*Tick+Overlap_u*Tick;
				Overlap = Overlap_u*Tick;
			}
			else																																																																		// ��������� �������
			{
				DeathTime = 2*DeathTime_u*Tick+Overlap_u*Tick;
				Overlap = Overlap_u*Tick;
			}
			
			// ������ ������������ ���������
			
			Period  = 2*psk;					// ������
			Pulse_1 = psk+DeathTime;	// ������������ �������� ����� ��������� A			+			8
			Pulse_2 = psk-Overlap;		// ������������ ������� ����� ��������� A				-			9
			Pulse_3 = psk-DeathTime;	// ������������ �������� ����� ��������� B			-			10
			Pulse_4 = psk+Overlap;		// ������������ ������� ����� ��������� B				+			11
			
	}
	
		// ��������� ���������� ���������
void SetTiming(void)
	{
		//aCNT = Period;
		//bCNT = 1;
		//TIM1->CR1 &= ~(1 << 0);
		TIM1->ARR  = Period;
		TIM1->CCR1 = Pulse_1;
		TIM1->CCR2 = Pulse_2;
		TIM1->CCR3 = Pulse_3;
		TIM1->CCR4 = Pulse_4;
		//TIM1->CR1 |= (1 << 0);
	}
		// ������ ���
void StartPWM(void)
	{
			//CommandReply(U2CT_STATE, 'i', STATUS.trans_state);
			
			CommandReply(U2CT_STATE, 'i', STATUS.trans_state);
			HAL_TIM_OC_Stop_IT(&htim16,TIM_CHANNEL_1);
			sec = 0;
			min = 0;
			n = 0;				// ���� ��������� �������� ������
			n_stop = 0;			
			TIM1 -> CNT = 0;
			TIM1 -> SR &= TIM_SR_CC1IF;
			TIM1 -> SR &= TIM_SR_CC2IF;
			TIM1 -> SR &= TIM_SR_CC3IF;
			TIM1 -> SR &= TIM_SR_CC4IF;
			TIM1 -> SR &= TIM_SR_CC5IF;
			TIM1 -> SR &= TIM_SR_CC6IF;
			HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);				// ������ ���
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
				
		
			TIM1 -> BDTR |= ~TIM_BREAK_DISABLE;							// ��������� ������� ���
			HAL_GPIO_WritePin(GPIOC,GPIO_TRANS_Pin,GPIO_PIN_SET);				// ��������� ���������� ��������
			
			
			//TIM7 -> CNT = 0;
			//TIM7 -> ARR = Period - 2*DeathTime;
			
			
			HAL_TIM_Base_Stop_IT(&htim7);
			if(SoftStart == OFF)
			{
				TIM7 -> CNT = 0;
				TIM7 -> EGR |= TIM_EGR_UG;
				TIM7 -> ARR = Pulse_1 - DeathTime + 20*Tick;
				HAL_TIM_Base_Start_IT(&htim7);				// ������ ������� ���
			}
			
		
	}
		
		// ��������� ���
void StopPWM(void)
	{
			
			HAL_GPIO_WritePin(GPIOC,GPIO_TRANS_Pin,GPIO_PIN_RESET);		// ���������� ���������� ��������
			if(diag == 1)
				StopDiag = 1;
			
			if(Mode == MAIN)
				HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);				// ��������� ������� �������� ���������� ��� �������� �������
			
			//if(n_stop == 0)
			//{
				HAL_TIM_Base_Stop_IT(&htim6);											// ���������� ������� ������
				HAL_TIM_Base_Stop_IT(&htim7);											// ��������� ������� ���
			
			switch(SoftStart)
			{
				case ON:
					
			if(n_stop == 10)
			{
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);					// ��������� ����� ���������
				HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
			}
			
			if(n_stop == 10)
			{
			HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);					// ��������� ������ ���������
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
						
							
			TIM1 -> BDTR &= TIM_BREAK_ENABLE;										// ���������� ������� ���
			n = 0;
			TIM1 -> CNT = 0;
			TIM1 -> ARR = 0;
			TIM6 -> ARR = 0;
			TIM7 -> ARR = 0;
			TIM6 -> CNT = 0;
			HAL_TIM_OC_Start_IT(&htim16,TIM_CHANNEL_1);
			TIM7 -> CNT = 0;
			STATUS.trans_state = 0;
			CommandReply(U2CT_STATE, 'i', STATUS.trans_state);
			}
			n_stop++;																										// ���� �������� ��������
			break;
				case OFF:
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);					// ��������� ����� ���������
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
					HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);					// ��������� ������ ���������
					HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
					
				TIM1 -> BDTR &= TIM_BREAK_ENABLE;										// ���������� ������� ���
				n = 0;
				TIM1 -> CNT = 0;
				TIM1 -> ARR = 0;
				TIM6 -> ARR = 0;
					TIM7 -> ARR = 0;
				TIM6 -> CNT = 0;
				HAL_TIM_OC_Start_IT(&htim16,TIM_CHANNEL_1);
				TIM7 -> CNT = 0;
				STATUS.trans_state = 0;
				CommandReply(U2CT_STATE, 'i', STATUS.trans_state);
				break;
			}
			
			
				
	}
		
	// ����������� � �������� ������
void DiagnSendData(void)
{
	SendForm = 0;
	StartSend = 1;
	StopDiag = 0;
	Diag();
}

	// ����������� � �������
void DiagnSendComand(void)
{
	SendForm = 1;
	StartSend = 1;
	StopDiag = 0;
	Diag();
}

	// ����������� � ����� � ��������
void DiagnSendComandnData(void)
{
	SendForm = 2;
	StartSend = 1;
	StopDiag = 0;
	Diag();
}

	// ����������� � ������ ������
void DiagnSendAlarm(void)
{
	SendForm = 3;
	StartSend = 1;
	StopDiag = 0;
	Diag();
}

		// ������ ��������
void SendData(uint32_t data)
	{	
		// ���������� �����������
		
		//Diag();
		#ifndef DEBUG
		if((STATUS.ant_break[0]==1)||(STATUS.ant_break[1]==1)||(STATUS.ant_break[2]==1)||(STATUS.ant_break[3]==1)||
			 (STATUS.ant_fuse[0]==1)||(STATUS.ant_fuse[1]==1)||(STATUS.ant_fuse[2]==1)||(STATUS.ant_fuse[3]==1)){}
			else if((SETUP.ant[0]==1)||(SETUP.ant[1]==1)||(SETUP.ant[2]==1)||(SETUP.ant[3]==1))
		#endif
			{
		// ��������� ��������� ����������
		Tick = SystemCoreClock/((TIM1->PSC+1)*2*1000000);					// ������ ������ ���� ������� ������������� 1 ���
		Position = 0;
		aCNT = 0;
		DeathTime = 2*DeathTime_u*Tick+Overlap_u*Tick;						// ������ ��������
		Overlap = Overlap_u*Tick;																	// ������ ����������
		TimeLimit = 10*Tick;																			// ��������� ������������ 10 �����	
		State = pwmSTART;																					// ������ ���
		TransMode = SENDDATA;
		//SoftStart = ON;
		SoftStart = SETUP.softsrart;
		diag = 0;
		Parsing(data,0);
		//STATUS.trans_state = 1;
		
		TIM6 -> ARR = SystemCoreClock/((TIM6->PSC+1)*BR);					// ��������� �������� ��������
		
		TIM6 -> CNT = 0;
		if(def == 0)
			HAL_TIM_Base_Start_IT(&htim6);														// ������ ��������
	}
	}
	
	
	void SendComand(uint8_t comand)
	{	
		// ���������� �����������
		
		//Diag();
		#ifndef DEBUG
		if((STATUS.ant_break[0]==1)||(STATUS.ant_break[1]==1)||(STATUS.ant_break[2]==1)||(STATUS.ant_break[3]==1)||
			 (STATUS.ant_fuse[0]==1)||(STATUS.ant_fuse[1]==1)||(STATUS.ant_fuse[2]==1)||(STATUS.ant_fuse[3]==1)){}
			else if((SETUP.ant[0]==1)||(SETUP.ant[1]==1)||(SETUP.ant[2]==1)||(SETUP.ant[3]==1))
		#endif		
			{
		// ��������� ��������� ����������
		Tick = SystemCoreClock/((TIM1->PSC+1)*2*1000000);					// ������ ������ ���� ������� ������������� 1 ���
		Position = 0;
		aCNT = 0;
		DeathTime = 2*DeathTime_u*Tick+Overlap_u*Tick;						// ������ ��������
		Overlap = Overlap_u*Tick;																	// ������ ����������
		TimeLimit = 10*Tick;																			// ��������� ������������ 10 �����	
		State = pwmSTART;																					// ������ ���
		TransMode = SENDDATA;
		//SoftStart = ON;
		SoftStart = SETUP.softsrart;
		diag = 0;
		Parsing(0,comand);
		//STATUS.trans_state = 1;
		
		TIM6 -> ARR = SystemCoreClock/((TIM6->PSC+1)*BR);					// ��������� �������� ��������
		
		TIM6 -> CNT = 0;
		if(def == 0)
			HAL_TIM_Base_Start_IT(&htim6);														// ������ ��������
	}
	}
	
	void SendComandnData(uint32_t data, uint8_t comand)
	{	
		// ���������� �����������
		
		//Diag();
		#ifndef DEBUG
		if((STATUS.ant_break[0]==1)||(STATUS.ant_break[1]==1)||(STATUS.ant_break[2]==1)||(STATUS.ant_break[3]==1)||
			 (STATUS.ant_fuse[0]==1)||(STATUS.ant_fuse[1]==1)||(STATUS.ant_fuse[2]==1)||(STATUS.ant_fuse[3]==1)){}
			else if((SETUP.ant[0]==1)||(SETUP.ant[1]==1)||(SETUP.ant[2]==1)||(SETUP.ant[3]==1))
		#endif		
			{
		// ��������� ��������� ����������
		Tick = SystemCoreClock/((TIM1->PSC+1)*2*1000000);					// ������ ������ ���� ������� ������������� 1 ���
		Position = 0;
		aCNT = 0;
		DeathTime = 2*DeathTime_u*Tick+Overlap_u*Tick;						// ������ ��������
		Overlap = Overlap_u*Tick;																	// ������ ����������
		TimeLimit = 10*Tick;																			// ��������� ������������ 10 �����	
		State = pwmSTART;																					// ������ ���
		TransMode = SENDDATA;
		//SoftStart = ON;
		SoftStart = SETUP.softsrart;
		diag = 0;
		Parsing(data,comand);
		//STATUS.trans_state = 1;
		
		TIM6 -> ARR = SystemCoreClock/((TIM6->PSC+1)*BR);					// ��������� �������� ��������
		
		TIM6 -> CNT = 0;
		if(def == 0)
			HAL_TIM_Base_Start_IT(&htim6);														// ������ ��������
	}
	}

	// ������ ����������� ����������
void SendAlarm(uint32_t data)
{
	// ���������� �����������
		
		//Diag();
		#ifndef DEBUG
		if((STATUS.ant_break[0]==1)||(STATUS.ant_break[1]==1)||(STATUS.ant_break[2]==1)||(STATUS.ant_break[3]==1)||
			 (STATUS.ant_fuse[0]==1)||(STATUS.ant_fuse[1]==1)||(STATUS.ant_fuse[2]==1)||(STATUS.ant_fuse[3]==1)){}
			else if((SETUP.ant[0]==1)||(SETUP.ant[1]==1)||(SETUP.ant[2]==1)||(SETUP.ant[3]==1))
		#endif
			{
		// ��������� ��������� ����������
				Tick = SystemCoreClock/((TIM1->PSC+1)*2*1000000);					// ������ ������ ���� ������� ������������� 1 ���
				Position = 0;
				aCNT = 0;
				DeathTime = 2*DeathTime_u*Tick+Overlap_u*Tick;						// ������ ��������
				Overlap = Overlap_u*Tick;																	// ������ ����������
				TimeLimit = 10*Tick;																			// ��������� ������������ 10 �����	
				State = pwmSTART;							
				TransMode = ALARM;
				SoftStart = SETUP.softsrart;
				
				if(data==65535)
					psk = SystemCoreClock/((TIM1->PSC+1)*4*f1);
				else if(data == 0)
					psk = SystemCoreClock/((TIM1->PSC+1)*4*f2);
	
				TimingCalc();																								// ������ ������������ ���������
				SetTiming();																								// ��������� ������������� ���������
		
				
				diag = 0;
		
				TIM6 -> ARR = SystemCoreClock/(TIM6->PSC+1);					// ��������� �������� ��������
		
				TIM6 -> CNT = 0;
				if(def == 0)
					HAL_TIM_Base_Start_IT(&htim6);														// ������ ��������
		
					
				}
}
		
	void ProcAlarm(void)
	{
			STATUS.repeatcur = 0;
			STATUS.bitnum = 0;
				

		
		// ���������/������ ���
		switch(State)
		{
			case pwmSTART:
				STATUS.trans_state = 1;
				StartPWM();
				State = pwmBISY;
			break;
			case pwmSTOP:
				STATUS.trans_state = 0;
				StopPWM();
				State = pwmBISY;
			break;
		}
				
		
		FormSTATUS();
		SendPacketUART();																	// �������� ��������� ����������� �� ���� � ������ �������� � ����� ������� ����

		
	}
		// ��������� 
void Generator(void)
{
	Tick = SystemCoreClock/((TIM1->PSC+1)*2*1000000);					// ������ ������ ���� ������� ������������� 1 ���
	Position = 0;
	DeathTime = 2*DeathTime_u*Tick+Overlap_u*Tick;						// ������ ��������
	Overlap = Overlap_u*Tick;																	// ������ ����������
	TimeLimit = 10*Tick;																			// ��������� ������������ 10 �����	
	State = pwmSTART;							
	TransMode = GENERATOR;
	SoftStart = SETUP.softsrart;
	psk = SystemCoreClock/((TIM1->PSC+1)*4*fgen);	
	
	TimingCalc();																								// ������ ������������ ���������
	SetTiming();																								// ��������� ������������� ���������
		
		
		switch(State)																								// ��������� ��� ������ ���
		{
			case pwmSTART:
				StartPWM();
				State = pwmBISY;
			break;
			case pwmSTOP:
				StopPWM();
				State = pwmBISY;
			break;
		}
	
}
	// ���������������
void Demagnetization(void)
{
	freq = 5000;																								// ��������� �������
	DeathTime = 2*DeathTime_u*Tick+Overlap_u*Tick;							// ������ ��������
	Overlap = Overlap_u*Tick;																		// ������ ����������
	TimeLimit = 4*Tick;																					// ��������� ��������� ������������ 4 ����
	State = pwmSTART;																						// ������ ���			
	TransMode = DEMAG;
	
	TIM6 -> ARR = SystemCoreClock/((TIM6->PSC+1)*10);						// ��������� �������� ����� �������
	
	TIM6 -> CNT = 0;
	HAL_TIM_Base_Start_IT(&htim6);															// ������ ��������������
}


	// ���������� ���������������
void ProcDemagnetization(void)
{
	
	if(freq <= 1000)
		{
			State = pwmSTOP;																					// ��������� �� �������� �������
		}
		
		psk = SystemCoreClock/((TIM1->PSC+1)*4*freq);								// ������ ������� �������
		
		TimingCalc();																								// ������ ������������ ���������
		SetTiming();																								// ��������� ������������� ���������
		
		freq-=100;																									// ��������� �������
		
		switch(State)																								// ��������� ��� ������ ���
		{
			case pwmSTART:
				StartPWM();
				State = pwmBISY;
			break;
			case pwmSTOP:
				StopPWM();
				State = pwmBISY;
			break;
		}
	

}

	// ������� ������ � ������
void Parsing(uint32_t data, uint8_t command)
{
	if(RepeatNum == 0)
		RepeatNum = 1;
			
	//	s - start
	//	d - data
	//	p - stop
	//	l - LSB
	//	k - �������
	//	m	-	MSB
	
	uint8_t tind;
	
	uint8_t	dataBIN[16];
	uint8_t dataR1[8];
	uint8_t dataR2[10];
	uint8_t datacom[4];
	uint8_t BaseLenght;
	
	switch(CodeMode){
		case 0:									// �������� 
			BaseLenght = 16;
			DataLenght = BaseLenght*RepeatNum;
			if(DataLenght > MAXDAT)
				DataLenght = ceil(DataLenght/BaseLenght)*RepeatNum;
			for(uint8_t i = 0; i < 16; i++)
				dataBIN[i] = (data >> i)&1;					// ����� ���������� ����������� ������� ����� ������
		
		
		for(uint8_t ind = 0; ind < DataLenght; ind++)
			{
				
				tind = ind - floorl(ind/BaseLenght)*(BaseLenght);
				
					// � �������� ����� ����������� ������� ����� ������ case 0 = dataBIN[15] 
					// ���� ������ ������� ������, �� ����� �������� ������� ��� case 0 = dataBIN[0] 
				switch(tind){
					case 0:					// data 15 ������� ���
						Data[ind] = dataBIN[15];
					break;
					case 1:					 // data 14
						Data[ind] = dataBIN[14];
					break;
					case 2:					// data 13
						Data[ind] = dataBIN[13];
					break;
					case 3:					// data 12
						Data[ind] = dataBIN[12];
					break;
					case 4:					// data 11
						Data[ind] = dataBIN[11];
					break;
					case 5:					// data 10
						Data[ind] = dataBIN[10];
					break;
					case 6:					 // data 9
						Data[ind] = dataBIN[9];
					break;
					case 7:					// data 8
						Data[ind] = dataBIN[8];
					break;
					case 8:					// data 7
						Data[ind] = dataBIN[7];
					break;
					case 9:					// data 6
						Data[ind] = dataBIN[6];
					break;
					case 10:				// data 5
						Data[ind] = dataBIN[5];
					break;
					case 11:					// data 4
						Data[ind] = dataBIN[4];
					break;
					case 12:					// data 3
						Data[ind] = dataBIN[3];
					break;
					case 13:					// data 2
						Data[ind] = dataBIN[2];
					break;
					case 14:					// data 1
						Data[ind] = dataBIN[1];
					break;
					case 15:					// data 0 ������� ���
						Data[ind] = dataBIN[0];
					break;}
				}
		break;
		case 1:									// ������ 1
							  // s d p
			BaseLenght = 1+8+2;
			DataLenght = BaseLenght*RepeatNum;
			if(DataLenght > MAXDAT)
				DataLenght = ceil(DataLenght/BaseLenght)*RepeatNum;
			for(uint8_t i = 0; i < 8; i++)
				dataR1[i] = (data >> i)&1;
		
		
			for(uint8_t ind = 0; ind < DataLenght; ind++)
			{
				
				tind = ind - floorl(ind/BaseLenght)*(BaseLenght);
				
				switch(tind){
					case 0:					// start
						Data[ind] = START_BIT;
					break;
					case 1:					// data 0
						Data[ind] = dataR1[0];
					break;
					case 2:					// data 1
						Data[ind] = dataR1[1];
					break;
					case 3:					// data 2
						Data[ind] = dataR1[2];
					break;
					case 4:					// data 3
						Data[ind] = dataR1[3];
					break;
					case 5:					// data 4
						Data[ind] = dataR1[4];
					break;
					case 6:					// data 5
						Data[ind] = dataR1[5];
					break;
					case 7:					// data 6
						Data[ind] = dataR1[6];
					break;
					case 8:					// data 7
						Data[ind] = dataR1[7];
					break;
					case 9:					// stop
						Data[ind] = STOP_BIT;
					break;
					case 10:				// stop
						Data[ind] = STOP_BIT;
					break;
					}
				
				
				
				
			}
		break;
		case 2:									// ������ 2
								//	s d	m	p	s	k	d	l	p
			BaseLenght = (1+7+1+2+1+4+3+1+2);
			DataLenght = BaseLenght*RepeatNum;
			if(DataLenght > MAXDAT)
				DataLenght = ceil(DataLenght/BaseLenght)*RepeatNum;
			for(uint8_t i = 0; i < 10; i++)
					dataR2[i] = (data >> i)&1;
		
			for(uint8_t i = 0; i < 4; i++)
					datacom[i] = (command >> i)&1;
		
			for(uint8_t ind = 0; ind < DataLenght; ind++)
			{
				
				tind = ind - floorl(ind/BaseLenght)*(BaseLenght);
				
				switch(tind){
					case 0:					// start
						Data[ind] = START_BIT;
					break;
					case 1:					// com 0
						Data[ind] = datacom[0];
					break;
					case 2:					// com 1
						Data[ind] = datacom[1];
					break;
					case 3:					// com 2
						Data[ind] = datacom[2];
					break;
					case 4:					// com 3
						Data[ind] = datacom[3];
					break;
					case 5:					// data 0
						Data[ind] = dataR2[0];
					break;
					case 6:					// data 1
						Data[ind] = dataR2[1];
					break;
					case 7:					// data 2
						Data[ind] = dataR2[2];
					break;
					case 8:					// LSB
						Data[ind] = LSB;
					break;
					case 9:					// stop
						Data[ind] = STOP_BIT;
					break;
					case 10:				// stop
						Data[ind] = STOP_BIT;
					break;
					case 11:					// start
						Data[ind] = START_BIT;
					break;
					case 12:					// data 3
						Data[ind] = dataR2[3];
					break;
					case 13:					// data 4
						Data[ind] = dataR2[4];
					break;
					case 14:					// data 5
						Data[ind] = dataR2[5];
					break;
					case 15:					// data 6
						Data[ind] = dataR2[6];
					break;
					case 16:					// data 7
						Data[ind] = dataR2[7];
					break;
					case 17:					// data 8
						Data[ind] = dataR2[8];
					break;
					case 18:					// data 9
						Data[ind] = dataR2[9];
					break;
					case 19:					// MSB
						Data[ind] = MSB;
					break;
					case 20:					// stop
						Data[ind] = STOP_BIT;
					break;
					case 21:				// stop
						Data[ind] = STOP_BIT;
					break;
					
					}
			}
		break;
	}
}


		// ��������� ������
void ProcData(void)
	{
		
		//Parsing(mData[m-1]);
		
		
			STATUS.repeatcur = floorl(Position/DataLenght)+1;
			STATUS.bitnum = Position - floorl(Position/DataLenght)*(DataLenght/RepeatNum);
				
//		if(Position > (DataLenght-1))
//		{
//			State = pwmSTOP;
//			STATUS.trans_state = 0;													// ��������� ��������� ��������
//			
//		}
		if(Position < DataLenght)
		{
		switch(Data[Position]){														// ����� ������ �� �����
			case 1:
				psk = SystemCoreClock/((TIM1->PSC+1)*4*f1);
				break;
			case 0:
				psk = SystemCoreClock/((TIM1->PSC+1)*4*f2);
				break;
		}
		// ���� ��������� ���������, �� ���������
		Position++;
		}
		else
		{
			State = pwmSTOP;
			STATUS.trans_state = 0;													// ��������� ��������� ��������
		}
		
		/*
		if(Position == 0)
				psk = SystemCoreClock/((TIM1->PSC+1)*4*f1);
		else
				psk = SystemCoreClock/((TIM1->PSC+1)*4*f2);
		
		Position = ~Position;*/
		// ������ � ��������� �������������
		if(State != pwmSTOP)
		{
			if(psk != aCNT)
			{
				aCNT = psk;
				TimingCalc();
				SetTiming();
			}
			
		}
		
		
		// ���������/������ ���
		switch(State)
		{
			case pwmSTART:
				STATUS.trans_state = 1;
				StartPWM();
				State = pwmBISY;
			break;
			case pwmSTOP:
				STATUS.trans_state = 0;
				StopPWM();
				State = pwmBISY;
			break;
		}
				
		
		FormSTATUS();
		SendPacketUART();																	// �������� ��������� ����������� �� ���� � ������ �������� � ����� ������� ����

		
	}
	

	//  ������ ����������� ������
	void Diag(void)
	{
		StopDiag = 0;
		Tick = SystemCoreClock/((TIM1->PSC+1)*2*1000000);
		freq = (f1 + f2)/2;																					// ��������� �������
		DeathTime = 2*DeathTime_u*Tick+Overlap_u*Tick;							// ������ ��������
		Overlap = Overlap_u*Tick;																		// ������ ����������
		TimeLimit = 10*Tick;																				// ��������� ��������� ������������ 10 �����
		State = pwmSTART;																						// ������ ���			
		TransMode = DIAG;																						// ����� ������ �����������
		diag = 0;
		n = 0;

		
		SoftStart = OFF;//
		
		
		TIM6 -> ARR = SystemCoreClock/((TIM6->PSC+1)*1);						// ��������� ������������ �����������
	
		TIM6 -> CNT = 0;
		
		if(def == 0)
			HAL_TIM_Base_Start_IT(&htim6);															// ������ �����������
	}
	
	// ��������� �������� ������
	void ProcDiag(void)
	{
		if(diag == 1)
		{
			State = pwmSTOP;																					// ��������� �� �������� �������
			FormSTATUS();
			SendPacketUART();
			//StopDiag = 1;
		}
		else{
		psk = SystemCoreClock/((TIM1->PSC+1)*2*freq);								// ������ ������� �������
		TimeLimit_u = 2*TimeLimit_u;
		TimingCalc();																								// ������ ������������ ���������
		SetTiming();																								// ��������� ������������� ���������
		f_change = 0;
		}																							
		psk = SystemCoreClock/((TIM1->PSC+1)*40*freq);								// ������ ������� �������
		diag = 1;
		
		switch(State)																								// ��������� ��� ������ ���
		{
			case pwmSTART:
				STATUS.trans_state = 2;
				StartPWM();
				State = pwmBISY;
				//CommandReply(U2CT_STATE, 'i', STATUS.trans_state);
			break;
			case pwmSTOP:
				STATUS.trans_state = 0;
				StopPWM();
				SoftStart = ON;//
				State = pwmBISY;
				//CommandReply(U2CT_STATE, 'i', STATUS.trans_state);
			break;
		}
	
	}
	
	//	������ �������� ����������
	void StartExternPWM(void)
	{
		Tick = SystemCoreClock/((TIM1->PSC+1)*2*1000000);					// ������ ������ ���� ������� ������������� 1 ���
		DeathTime = 2*DeathTime_u*Tick+Overlap_u*Tick;						// ������ ��������
		Overlap = Overlap_u*Tick;																	// ������ ����������
		TimeLimit = 10*Tick;	
		n_IC = 0;
		CCR1tmp = 0;
		
		State = pwmSTART;	
		
		if(HAL_GPIO_ReadPin(GPIOB,AUX_R_IN_Pin) == GPIO_PIN_RESET)			//	���������� ���� ��������� ���������
			HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);										//	������ �������������� �������
		else if(HAL_GPIO_ReadPin(GPIOB,AUX_R_IN_Pin) == GPIO_PIN_SET)
			StopPWM();
		
		

		
	}
	
		// ��������� �������� ����������
	void ExternPWM(void)
	{
		tmpData = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
		freq = SystemCoreClock/((TIM2->PSC+1)*HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1))-2;
		psk = SystemCoreClock/((TIM1->PSC+1)*4*freq);		// ��������� ������������ ��������
		TIM2 -> CNT = 0;
		
		
		
		if((n_IC == 1)&&(psk>0))
			State = pwmSTART;
		if(psk > 0x800F)				// ���������� ��� ���������� �������
			State = pwmSTOP;
		if(psk > 0x7FFF)				// ����������� ���������� �������
			psk = 0x7FFF;
		
		
		//if(psk != CCR1tmp)
		//{
			if((freq > CCR1tmp+45)||(freq < CCR1tmp-45))
			{
			//	n_IC = 0;
				CCR1tmp = freq;
				TimingCalc();						// ������ � ��������� ������������� ���������
				SetTiming();
			}
		//	n_IC++;
	//	}	
			
		
		switch(State)						// ���������/������ ���
		{
			case pwmSTART:
				StartPWM();
				State = pwmBISY;
				n_IC  = 0;
			break;
			case pwmSTOP:
				StopPWM();
				State = pwmBISY;
				n_IC = 1;
			break;
		}
	}

	// ������������� ������� ���������
	void CorrectTIM(void)
	{
		
		switch(n)
		{
			case 0:
				n = 1;
				break;
			case 1:
				TIM7 -> ARR = Pulse_3/2 - 80*Tick;	// ������ ��������
				n = 9;
				//HAL_TIM_Base_Start_IT(&htim7);
				break;
			case 2:
				TIM7 -> ARR = DeathTime + 20*Tick;	// ������ ��������
				HAL_TIM_Base_Start_IT(&htim7);
				n = 6;
				break;
			case 6:
				n = 3;
				break;
			case 3:
				TIM7 -> ARR = Pulse_3 - 80*Tick;	// ����� ��������
			if(diag ==1 )
					HAL_TIM_Base_Stop_IT(&htim7);
				n = 4;
				break;
			case 4:
				TIM7 -> ARR = DeathTime + 80*Tick;		// ������ ��������
				n = 5;
				break;
			case 5:
				TIM7 -> ARR = Period - Pulse_1 - 80*Tick;		// ����� ��������
				n = 7;
				break;
			case 7:
				HAL_TIM_Base_Stop_IT(&htim7);	// ��������� ������� ���
				n = 8;
				break;
			case 9:
				HAL_TIM_Base_Stop_IT(&htim7);	// ��������� ������� ���
				n = 10;
				break;
		}
		
		
	}
	
	// ��������� ���
void GetVoltage(void)
	{
		
		CorrectTIM();			// ������������� ������� ��������� 
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
		
		if(!((n==3)||(n==1))){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_SET);
		HAL_ADC_Start(&hadc1);			// ������ ���
		HAL_ADC_Start(&hadc2);
		HAL_ADC_Start(&hadc3);
		HAL_ADC_Start(&hadc4);
					
		HAL_ADC_PollForConversion(&hadc1,1);
		HAL_ADC_PollForConversion(&hadc2,1);
		HAL_ADC_PollForConversion(&hadc3,1);
		HAL_ADC_PollForConversion(&hadc4,1);

		ADC1buff = HAL_ADC_GetValue(&hadc1);
		ADC2buff = HAL_ADC_GetValue(&hadc2);
		ADC3buff = HAL_ADC_GetValue(&hadc3);
		ADC4buff = HAL_ADC_GetValue(&hadc4);
		
		HAL_ADC_Stop(&hadc1);
		HAL_ADC_Stop(&hadc2);
		HAL_ADC_Stop(&hadc3);
		HAL_ADC_Stop(&hadc4);
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
		}
		
		I1 = 0;
		I2 = 0;
		I3 = 0;
		I4 = 0;
			
				
		P1 = 0;
		P2 = 0;
		P3 = 0;
		P4 = 0;
		
		switch(n)
		{
			case 9:
				//DiagCNT++;
				U1rise1 = (ADC1buff)*Vsupp/Effbit-mean1;
				U2rise1 = (ADC2buff)*Vsupp/Effbit-mean2;
				U3rise1 = (ADC3buff)*Vsupp/Effbit-mean3;
				U4rise1 = (ADC4buff)*Vsupp/Effbit-mean4;
			
			if(SETUP.ant[0] == 1)
				{
					if(fabs(U1rise1) < BREAK)
						STATUS.ant_break[0] = 1;
					else
						STATUS.ant_break[0] = 0;
					
					if(fabs(U1rise1) > FUSE)
						STATUS.ant_fuse[0] = 1;
					else
						STATUS.ant_fuse[0] = 0;
				}
				else
				{
					STATUS.ant_break[0] = 0;
					STATUS.ant_fuse[0] = 0;
				}
				
				if(SETUP.ant[1] == 1)
				{
					if(fabs(U2rise1) < BREAK)
						STATUS.ant_break[1] = 1;
					else
						STATUS.ant_break[1] = 0;
					
					if(fabs(U2rise1) > FUSE)
						STATUS.ant_fuse[1] = 1;
					else
						STATUS.ant_fuse[1] = 0;
				}
				else
				{
					STATUS.ant_break[1] = 0;
					STATUS.ant_fuse[1] = 0;
				}
					
				if(SETUP.ant[2] == 1)
				{
					if(fabs(U3rise1) < BREAK)
						STATUS.ant_break[2] = 1;
					else
						STATUS.ant_break[2] = 0;
					
					if(fabs(U3rise1) > FUSE)
						STATUS.ant_fuse[2] = 1;
					else
						STATUS.ant_fuse[2] = 0;
					
				}
				else
				{
					STATUS.ant_break[2] = 0;
					STATUS.ant_fuse[2] = 0;
				}
					
				if(SETUP.ant[3] == 1)
				{
					if(fabs(U4rise1) < BREAK)
						STATUS.ant_break[3] = 1;
					else
						STATUS.ant_break[3] = 0;
					
					if(fabs(U4rise1) > FUSE)
						STATUS.ant_fuse[3] = 1;
					else
						STATUS.ant_fuse[3] = 0;
				}
				else
				{
					STATUS.ant_break[3] = 0;
					STATUS.ant_fuse[3] = 0;
				}
				
							
				L1 = CalculateL(U1rise1,Us1);
				L2 = CalculateL(U2rise1,Us2);
				L3 = CalculateL(U3rise1,Us3);
				L4 = CalculateL(U4rise1,Us4);
				
												
				C1corr = CalculateC(L1);
				C2corr = CalculateC(L2);
				C3corr = CalculateC(L3);
				C4corr = CalculateC(L4);
				
				
				
				break;
			case 10:																						// ������ �������
				
				U1fall1 = ADC1buff;
				U2fall1 = ADC2buff;
				U3fall1 = ADC3buff;
				U4fall1 = ADC4buff;
				
				break;
			case 5:																						// ������ �������
				
				U1fall1 = ADC1buff;
				U2fall1 = ADC2buff;
				U3fall1 = ADC3buff;
				U4fall1 = ADC4buff;
				
				break;
			case 7:																						// ������ ��������
				DiagCNT++;
				U1rise1 = (ADC1buff)*Vsupp/Effbit-mean1;
				U2rise1 = (ADC2buff)*Vsupp/Effbit-mean2;
				U3rise1 = (ADC3buff)*Vsupp/Effbit-mean3;
				U4rise1 = (ADC4buff)*Vsupp/Effbit-mean4;
			
			///*									//	�������� ������ �� ����������� �� ����� ��������
			if(SETUP.ant[0] == 1)
				{
					if(fabs(U1rise1) < BREAK)
						STATUS.ant_break[0] = 1;
					else
						STATUS.ant_break[0] = 0;
					
					if(fabs(U1rise1) > FUSE)
						STATUS.ant_fuse[0] = 1;
					else
						STATUS.ant_fuse[0] = 0;
				}
				else
				{
					STATUS.ant_break[0] = 0;
					STATUS.ant_fuse[0] = 0;
				}
				
				if(SETUP.ant[1] == 1)
				{
					if(fabs(U2rise1) < BREAK)
						STATUS.ant_break[1] = 1;
					else
						STATUS.ant_break[1] = 0;
					
					if(fabs(U2rise1) > FUSE)
						STATUS.ant_fuse[1] = 1;
					else
						STATUS.ant_fuse[1] = 0;
				}
				else
				{
					STATUS.ant_break[1] = 0;
					STATUS.ant_fuse[1] = 0;
				}
					
				if(SETUP.ant[2] == 1)
				{
					if(fabs(U3rise1) < BREAK)
						STATUS.ant_break[2] = 1;
					else
						STATUS.ant_break[2] = 0;
					
					if(fabs(U3rise1) > FUSE)
						STATUS.ant_fuse[2] = 1;
					else
						STATUS.ant_fuse[2] = 0;
					
				}
				else
				{
					STATUS.ant_break[2] = 0;
					STATUS.ant_fuse[2] = 0;
				}
					
				if(SETUP.ant[3] == 1)
				{
					if(fabs(U4rise1) < BREAK)
						STATUS.ant_break[3] = 1;
					else
						STATUS.ant_break[3] = 0;
					
					if(fabs(U4rise1) > FUSE)
						STATUS.ant_fuse[3] = 1;
					else
						STATUS.ant_fuse[3] = 0;
				}
				else
				{
					STATUS.ant_break[3] = 0;
					STATUS.ant_fuse[3] = 0;
				}
			//*/
				I1 = CalculateI(U1rise1);
				I2 = CalculateI(U2rise1);
				I3 = CalculateI(U3rise1);
				I4 = CalculateI(U4rise1);
				
				I1a += I1;
				I2a += I2;
				I3a += I3;
				I4a += I4;
				
				if(I1 > I1m)
					I1m = I1;
				if(I2 > I2m)
					I2m = I2;
				if(I3 > I3m)
					I3m = I3;
				if(I4 > I4m)
					I4m = I4;
			
				if(DiagCNT > 1){
					
				U1fall2 *= Vsupp/Effbit;
				U2fall2 *= Vsupp/Effbit;
				U3fall2 *= Vsupp/Effbit;
				U4fall2 *= Vsupp/Effbit;
					
				U1fall2 -= mean1;
				U2fall2 -= mean2;
				U3fall2 -= mean3;
				U4fall2 -= mean4;
				
				if(STATUS.trans_state == 2){
				R1 = CalculateR(U1rise2,U1fall2, C1, L1);
				R2 = CalculateR(U2rise2,U2fall2, C2, L2);
				R3 = CalculateR(U3rise2,U3fall2, C3, L3);
				R4 = CalculateR(U4rise2,U4fall2, C4, L4);
					
				R1a = R1;
				R2a = R2;
				R3a = R3;
				R4a = R4;
				}
				P1 = CalculateP(I1, R1);
				P2 = CalculateP(I2, R2);
				P3 = CalculateP(I3, R3);
				P4 = CalculateP(I4, R4);
				
				P1a += P1;
				P2a += P2;
				P3a += P3;
				P4a += P4;
				
				if(P1 > P1m)
					P1m = P1;
				if(P2 > P2m)
					P2m = P2;
				if(P3 > P3m)
					P3m = P3;
				if(P4 > P4m)
					P4m = P4;
				
				
				}
				break;
			case 8:																						// ����� ��������
				U1fall2 = ADC1buff;
				U2fall2 = ADC2buff;
				U3fall2 = ADC3buff;
				U4fall2 = ADC4buff;				
				
				break;
			case 4:																						// ������ ��������
				DiagCNT++;
				U1rise2 = (ADC1buff)*Vsupp/Effbit-mean1;
				U2rise2 = (ADC2buff)*Vsupp/Effbit-mean2;
				U3rise2 = (ADC3buff)*Vsupp/Effbit-mean3;
				U4rise2 = (ADC4buff)*Vsupp/Effbit-mean4;
			
				U1fall1 *= Vsupp/Effbit;
				U2fall1 *= Vsupp/Effbit;
				U3fall1 *= Vsupp/Effbit;
				U4fall1 *= Vsupp/Effbit;
			
				U1fall1 -= mean1;
				U2fall1 -= mean2;
				U3fall1 -= mean3;
				U4fall1 -= mean4;
			
			///*													//	�������� ������ �� ����������� �� ����� ��������
			if(SETUP.ant[0] == 1)
				{
					if(fabs(U1rise2) < BREAK)
						STATUS.ant_break[0] = 1;
					else
						STATUS.ant_break[0] = 0;
					
					if(fabs(U1rise2) > FUSE)
						STATUS.ant_fuse[0] = 1;
					else
						STATUS.ant_fuse[0] = 0;
				}
				else
				{
					STATUS.ant_break[0] = 0;
					STATUS.ant_fuse[0] = 0;
				}
				
				if(SETUP.ant[1] == 1)
				{
					if(fabs(U2rise2) < BREAK)
						STATUS.ant_break[1] = 1;
					else
						STATUS.ant_break[1] = 0;
					
					if(fabs(U2rise2) > FUSE)
						STATUS.ant_fuse[1] = 1;
					else
						STATUS.ant_fuse[1] = 0;
				}
				else
				{
					STATUS.ant_break[1] = 0;
					STATUS.ant_fuse[1] = 0;
				}
				
					
				if(SETUP.ant[2] == 1)
				{
					if(fabs(U3rise2) < BREAK)
						STATUS.ant_break[2] = 1;
					else
						STATUS.ant_break[2] = 0;
					
					if(fabs(U3rise2) > FUSE)
						STATUS.ant_fuse[2] = 1;
					else
						STATUS.ant_fuse[2] = 0;
					
				}
				else
				{
					STATUS.ant_break[2] = 0;
					STATUS.ant_fuse[2] = 0;
				}
					
				if(SETUP.ant[3] == 1)
				{
					if(fabs(U4rise2) < BREAK)
						STATUS.ant_break[3] = 1;
					else
						STATUS.ant_break[3] = 0;
					
					if(fabs(U4rise2) > FUSE)
						STATUS.ant_fuse[3] = 1;
					else
						STATUS.ant_fuse[3] = 0;
				}
				else
				{
					STATUS.ant_break[3] = 0;
					STATUS.ant_fuse[3] = 0;
				}
			
				//*/										//	�������� ������ �� ����������� �� ����� ��������
			
				I1 = CalculateI(U1rise2);
				I2 = CalculateI(U2rise2);
				I3 = CalculateI(U3rise2);
				I4 = CalculateI(U4rise2);
				
				I1a += I1;
				I2a += I2;
				I3a += I3;
				I4a += I4;
				
				if(I1 > I1m)
					I1m = I1;
				if(I2 > I2m)
					I2m = I2;
				if(I3 > I3m)
					I3m = I3;
				if(I4 > I4m)
					I4m = I4;
			
				
				if(STATUS.trans_state == 2){		
				R1 = CalculateR(U1rise1,U1fall1, C1, L1);
				R2 = CalculateR(U2rise1,U2fall1, C2, L2);
				R3 = CalculateR(U3rise1,U3fall1, C3, L3);
				R4 = CalculateR(U4rise1,U4fall1, C4, L4);
				
				R1a = R1;
				R2a = R2;
				R3a = R3;
				R4a = R4;
				}
				P1 = CalculateP(I1, R1);
				P2 = CalculateP(I2, R2);
				P3 = CalculateP(I3, R3);
				P4 = CalculateP(I4, R4);
				
				P1a += P1;
				P2a += P2;
				P3a += P3;
				P4a += P4;
				
				
				if(P1 > P1m)
					P1m = P1;
				if(P2 > P2m)
					P2m = P2;
				if(P3 > P3m)
					P3m = P3;
				if(P4 > P4m)
					P4m = P4;
				
				
				
				//HAL_TIM_Base_Stop_IT(&htim7);
				
				break;
		}
		
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15,GPIO_PIN_RESET);
	}
	
	
			// ������ �������������
	float CalculateL(float Ur, float Usupp)
	{
		
		float Ltmp = 0;
			

		Ltmp = Usupp*L0/fabs(Ur);
		
		return Ltmp;																
	
	}
		
		// ������ ����
	float CalculateI(float Ur)
	{
		float Itmp;
		float f;
		
		f = SystemCoreClock/((TIM1->PSC+1)*4*psk);
		Itmp = fabs(Ur)/(2*PI*f*L0);								
		
		return Itmp;
	
	}
		
		
	float CalculateR(float Ur, float Uf, float Cap, float L)
	{
		float Rtmp;
		float T;
		
		if(SoftStart == OFF)
			T = 1/(2.0f*((float)freq));
		else
			T = ((1e-6f*((float)Period))/((float)Tick));
		
		if(SETUP.cap == 1)
			Rtmp = (T/Cap)*(fabs(Ur/(Ur-Uf+1e-6f))-0.5f);
		else
			Rtmp = fabs(logf(fabs(Ur/Uf)))*(L/T);				
		
		return fabs(Rtmp);
		
	}
		
		
	float CalculateP(float I, float R)
	{
		float Ptmp;
		
		Ptmp = 0.5f*powf(fabs(I),2)*R;
		
		if(fabs(Ptmp) > 9000)
			Ptmp = 9001;
		
		if(fabs(Ptmp) < 1e-6f)
			Ptmp = 0;
		
		return fabs(Ptmp);
	}
	
	float CalculateC(float L)
	{
		float Ctmp;
		float f;
		
		f = (f1+f2)/2;
		Ctmp = 1e6f/(L*powf((2*PI*f),2));
		
		return Ctmp;
	}
	
	void FormSTATUS(void)
	{
		
		//	�������� ������ �� ����������� �� ����� ��������
		// ��������� �������� ��� ������
		
		for(uint8_t i = 0; i < 4; i++)
		{
			if(STATUS.ant_break[i])
				StopPWM();
			if(STATUS.ant_fuse[i])
				StopPWM();
		}
		
		if(SETUP.ant[0] == 1)
		{
			STATUS.ia[0] = 0.5f*I1a/DiagCNT;
			STATUS.im[0] = I1m;
			STATUS.l[0]  = L1*1000;						//1000 � ���
			STATUS.ra[0] = R1a;///DiagCNT;
			STATUS.pa[0] = 0.5f*P1a/(DiagCNT*1000);				// � ���
			STATUS.pm[0] = P1m/1000;								// � ���
			STATUS.c[0]  = C1corr;							// � ���
		}
		else
		{
			STATUS.ia[0] = 0;
			STATUS.im[0] = 0;
			STATUS.l[0]  = 0;						//1000 � ���
			STATUS.ra[0] = 0;
			STATUS.pa[0] = 0;				// � ���
			STATUS.pm[0] = 0;								// � ���
			STATUS.c[0]  = 0;							// � ���
		}
		
		if(SETUP.ant[1] == 1)
		{
			STATUS.ia[1] = 0.5f*I2a/DiagCNT;
			STATUS.im[1] = I2m;
			STATUS.l[1]  = L2*1000;
			STATUS.ra[1] = R2a;///DiagCNT;
			STATUS.pa[1] = 0.5f*P2a/(DiagCNT*1000);
			STATUS.pm[1] = P2m/1000;
			STATUS.c[1]  = C2corr;
		}
		else
		{
			STATUS.ia[1] = 0;
			STATUS.im[1] = 0;
			STATUS.l[1]  = 0;
			STATUS.ra[1] = 0;
			STATUS.pa[1] = 0;
			STATUS.pm[1] = 0;
			STATUS.c[1]  = 0;
		}
		
		if(SETUP.ant[2] == 1)
		{
			STATUS.ia[2] = 0.5f*I3a/DiagCNT;
			STATUS.im[2] = I3m;
			STATUS.l[2]  = L3*1000;
			STATUS.ra[2] = R3a;///DiagCNT;
			STATUS.pa[2] = 0.5f*P3a/(DiagCNT*1000);
			STATUS.pm[2] = P3m/1000;
			STATUS.c[2]  = C3corr;
		}
		else
		{
			STATUS.ia[2] = 0;
			STATUS.im[2] = 0;
			STATUS.l[2]  = 0;
			STATUS.ra[2] = 0;
			STATUS.pa[2] = 0;
			STATUS.pm[2] = 0;
			STATUS.c[2]  = 0;
		}
		
		if(SETUP.ant[3] == 1)
		{
			STATUS.ia[3] = 0.5f*I4a/DiagCNT;
			STATUS.im[3] = I4m;
			STATUS.l[3]  = L4*1000;
			STATUS.ra[3] = R4a;///DiagCNT;
			STATUS.pa[3] = 0.5f*P4a/(DiagCNT*1000);
			STATUS.pm[3] = P4m/1000;
			STATUS.c[3]  = C4corr;
		}
		else
		{
			STATUS.ia[3] = 0;
			STATUS.im[3] = 0;
			STATUS.l[3]  = 0;
			STATUS.ra[3] = 0;
			STATUS.pa[3] = 0;
			STATUS.pm[3] = 0;
			STATUS.c[3]  = 0;
		}
	
		DiagCNT =0;
				
		I1a = 0;
		I2a = 0;
		I3a = 0;
		I4a = 0;
		
		/*R1a = 0;
		R2a = 0;
		R3a = 0;
		R4a = 0;*/
		
		P1a = 0;
		P2a = 0;
		P3a = 0;
		P4a = 0;
		
		I1m = 0;
		I2m = 0;
		I3m = 0;
		I4m = 0;
		
		P1m = 0;
		P2m = 0;
		P3m = 0;
		P4m = 0;		
	
	}
	
		
	// ������������ �������� � �������
	void SendPacketUART(void)
	{
			Cmd_Type cmd;


		// ������� ������ ���� (�������), � ���������!!!
		cmd = U2CT_STATUS;
		HAL_UART_Transmit(&huart1, (uint8_t *)&cmd, sizeof(cmd), HAL_MAX_DELAY);
    // ������� ������
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)&STATUS, sizeof(STATUS));
	}
	
	 // �������� ������ � ����������
	void CommandReply(Cmd_Type cmd, const char fmt, ...)
	{
				
		UART2_Queue_Data data;
		va_list al;
  
		va_start(al, fmt);
		data.cmd = cmd;
		switch (fmt) {
			case 'i':
				data.value.i = va_arg(al, int);
				break;
			case 'f':
				data.value.f = va_arg(al, double);
				break;
		}
		va_end(al);
	
				
		HAL_UART_Transmit(&huart1, (uint8_t *)&data.cmd, sizeof(data.cmd), HAL_MAX_DELAY);
		//HAL_UART_Transmit_IT(&huart1, (uint8_t *)&data.value, sizeof(data.value));				// � ������������
		HAL_UART_Transmit(&huart1, (uint8_t *)&data.value, sizeof(data.value), HAL_MAX_DELAY); // ��� ����������
	}
	
	//	����� ������ �� ���� � �����������
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		if (UART2_RECV_CMD == UART2_RecvType) {
			// ����� ��� ��������
			UART2_RecvType = UART2_RECV_VALUE;
			// � ����������� �� ������� ��������� ������� �������� ������ �����
			if (UART2RecvData.cmd == U2CT_SETUP)
				HAL_UART_Receive_IT(&huart1,(uint8_t*)&SETUP, sizeof(SETUP));
			else
				HAL_UART_Receive_IT(&huart1,(uint8_t*)&UART2RecvData.value, sizeof(UART2RecvData.value));
		} else {
		
				
		switch (UART2RecvData.cmd) {

    // ��������� ���� ��������
		case U2CT_DATA:																								// ����� ������� 
			SETUP.data = UART2RecvData.value.i;
			tmpData		 = SETUP.data;
			break;
    case U2CT_F1:																									// ������� 1
      SETUP.freq1 = UART2RecvData.value.i;
			f1 					= SETUP.freq1;
      break;
    case U2CT_F2:																									// ������� 2
      SETUP.freq2 = UART2RecvData.value.i;
			f2 					= SETUP.freq2;
      break;
    case U2CT_BR:																									// �������� �������� 
      SETUP.boudrate = UART2RecvData.value.i;
			BR 						 = (float)(SETUP.boudrate)/10.0f;
      break;
    case U2CT_DEATH_TIME:																					// �������
      SETUP.deadtime = UART2RecvData.value.i;
			DeathTime_u 	 = SETUP.deadtime;
      break;
    case U2CT_OVERLAP:																						// ����������
      SETUP.overlap = UART2RecvData.value.i;
			Overlap_u 		= -fabs(SETUP.overlap);
      break;
    case U2CT_TIME_LIMIT:																					// ������������ ������������
      SETUP.duration_limit = UART2RecvData.value.i;
			TimeLimit_u 				 = SETUP.duration_limit;
      break;
    case U2CT_DIAG_PERIOD:																				// ������ �������� ������
      SETUP.check_ant_period = UART2RecvData.value.i;
			DiagTime = SETUP.check_ant_period;
      break;
    case U2CT_CURRENT_LIMIT_A:																		// ����������� ����
      SETUP.current_limit = UART2RecvData.value.i;
      break;
    case U2CT_OVER_TICK:																					// ���������� ������������ ������ �� ����
      SETUP.count_halt_limit = UART2RecvData.value.i;
      break;
		case U2CT_C1:																									// ������� � �������1
			SETUP.capatity_ant[0] = UART2RecvData.value.f;
			C1 = SETUP.capatity_ant[0]*1e-6f;
			break;
		case U2CT_C2:																									// ������� � �������2
			SETUP.capatity_ant[1] = UART2RecvData.value.f;
			C2 = SETUP.capatity_ant[1]*1e-6f;
			break;
		case U2CT_C3:																									// ������� � �������3
			SETUP.capatity_ant[2] = UART2RecvData.value.f;
			C3 = SETUP.capatity_ant[2]*1e-6f;
			break;
		case U2CT_C4:																									// ������� � �������4
			SETUP.capatity_ant[3] = UART2RecvData.value.f;
			C4 = SETUP.capatity_ant[3]*1e-6f;
			break;
		case U2CT_REPEATNUM:																					// ����� �������� ��������
			SETUP.repeatnum = UART2RecvData.value.i;
			RepeatNum = SETUP.repeatnum;
			break;
		case U2CT_ENABLE:																							// ������/������� ��������
			SETUP.enable = UART2RecvData.value.i;
			
			if((SETUP.enable == 1)&&(STATUS.trans_state == 0))																				// ������ ��������
		{
			
			if(Mode == MAIN){
				if((tmpData == 0) || (tmpData == 65535))
					DiagnSendAlarm();
				else
					//SendData(tmpData);
				DiagnSendData();
			}
			else if(Mode == AUX)
				StartExternPWM();
			else if(Mode == GEN)
				Generator();
			
		}
		else if(SETUP.enable == 0)
			StopPWM();
		
			break;
		case U2CT_MODE:																								// ����� ������ ��������/�������
			SETUP.mode = UART2RecvData.value.i;
			Mode 			 = SETUP.mode;
			break;
		case U2CT_SOFT_START:																					// ������� ������
			SETUP.softsrart = UART2RecvData.value.i;
			SoftStart				= SETUP.softsrart;
			break;
		case U2CT_CURRENT_LIMIT:																			// ����� ����������� ���� ���/����
			SETUP.currentlimit = UART2RecvData.value.i;
			break;
		case U2CT_DIAG:																								// ����� �������� �������������, �������
			SETUP.diag = UART2RecvData.value.i;
			
			if((SETUP.diag == 1)&&(STATUS.trans_state == 0))
				Diag();
			
			SETUP.diag = 0;
			break;
		case U2CT_CAP:																								// ������� ���/����
			SETUP.cap = UART2RecvData.value.i;
			break;
		case U2CT_MODUlATION:																					// ��������� ���/���
			SETUP.modulation = UART2RecvData.value.i;
			break;
		case U2CT_CODEMODE:																						// ���������
			SETUP.codemode = UART2RecvData.value.i;
			CodeMode = SETUP.codemode;
			break;
		case U2CT_OVERLOAD_MODE:																			// ���������� �� ���� ����/ �����������
			SETUP.overloadmode = UART2RecvData.value.i;
			break;
		case U2CT_SUPPVOLTAGE:
			SETUP.suppvoltage = UART2RecvData.value.i;
			Us1 = CalculateSupply(SETUP.antlevel[0],SETUP.supplevel,SETUP.suppvoltage,1);
			Us2 = CalculateSupply(SETUP.antlevel[1],SETUP.supplevel,SETUP.suppvoltage,2);
			Us3 = CalculateSupply(SETUP.antlevel[2],SETUP.supplevel,SETUP.suppvoltage,3);
			Us4 = CalculateSupply(SETUP.antlevel[3],SETUP.supplevel,SETUP.suppvoltage,4);
			break;
		case U2CT_ALARMMSG:
			SETUP.alarmmsg = UART2RecvData.value.i;
			tmpComand = SETUP.alarmmsg;
			if(STATUS.trans_state == 0)																				// ������ ��������
		{
			if(Mode == MAIN)
				DiagnSendComand();
		}
			break;
		case U2CT_ANTENNA_1:																					// ��������� �������1 ����������/���������
			SETUP.ant[0] = UART2RecvData.value.i;
			//SaveSetting();
			break;
		case U2CT_ANTENNA_2:																					// ��������� �������2 ����������/���������
			SETUP.ant[1] = UART2RecvData.value.i;
			break;
		case U2CT_ANTENNA_3:																					// ��������� �������3 ����������/���������
			SETUP.ant[2] = UART2RecvData.value.i;
			break;
		case U2CT_ANTENNA_4:																					// ��������� �������4 ����������/���������
			SETUP.ant[3] = UART2RecvData.value.i;
			break;
		case U2CT_ANT_LEVEL_1:																				// ���������� �� �������1 �������/������
			SETUP.antlevel[0] = UART2RecvData.value.i;
			Us1 = CalculateSupply(SETUP.antlevel[0],SETUP.supplevel,SETUP.suppvoltage,1);
			break;
		case U2CT_ANT_LEVEL_2:																				// ���������� �� �������2 �������/������
			SETUP.antlevel[1] = UART2RecvData.value.i;
			Us2 = CalculateSupply(SETUP.antlevel[1],SETUP.supplevel,SETUP.suppvoltage,2);			
			break;
		case U2CT_ANT_LEVEL_3:																				// ���������� �� �������3 �������/������
			SETUP.antlevel[2] = UART2RecvData.value.i;
			Us3 = CalculateSupply(SETUP.antlevel[2],SETUP.supplevel,SETUP.suppvoltage,3);
			break;
		case U2CT_ANT_LEVEL_4:																				// ���������� �� �������4 �������/������
			SETUP.antlevel[3] = UART2RecvData.value.i;
			Us4 = CalculateSupply(SETUP.antlevel[3],SETUP.supplevel,SETUP.suppvoltage,4);
			break;
		case U2CT_SUPPLEVEL:																					// ���������� �� ������� ������� �������/������
			SETUP.supplevel = UART2RecvData.value.i;
			Us1 = CalculateSupply(SETUP.antlevel[0],SETUP.supplevel,SETUP.suppvoltage,1);
			Us2 = CalculateSupply(SETUP.antlevel[1],SETUP.supplevel,SETUP.suppvoltage,2);
			Us3 = CalculateSupply(SETUP.antlevel[2],SETUP.supplevel,SETUP.suppvoltage,3);
			Us4 = CalculateSupply(SETUP.antlevel[3],SETUP.supplevel,SETUP.suppvoltage,4);
			break;
		case U2CT_SETUP:
			f1 = SETUP.freq1;
			f2 = SETUP.freq2;
			BR = (float)(SETUP.boudrate)/10.0f;
			DeathTime_u = SETUP.deadtime;
			Overlap_u = -SETUP.overlap;
			TimeLimit_u = SETUP.duration_limit;
			DiagTime = SETUP.check_ant_period;
			C1 = SETUP.capatity_ant[0]*1e-6f;
			C2 = SETUP.capatity_ant[1]*1e-6f;
			C3 = SETUP.capatity_ant[2]*1e-6f;
			C4 = SETUP.capatity_ant[3]*1e-6f;
			RepeatNum = SETUP.repeatnum;
			Mode 			 = SETUP.mode;
			SoftStart				= SETUP.softsrart;
			CodeMode = SETUP.codemode;
			Us1 = CalculateSupply(SETUP.antlevel[0],SETUP.supplevel,SETUP.suppvoltage,1);
			Us2 = CalculateSupply(SETUP.antlevel[1],SETUP.supplevel,SETUP.suppvoltage,2);
			Us3 = CalculateSupply(SETUP.antlevel[2],SETUP.supplevel,SETUP.suppvoltage,3);
			Us4 = CalculateSupply(SETUP.antlevel[3],SETUP.supplevel,SETUP.suppvoltage,4);
			//SaveSetting();
			break;
    default: break;
  }  // switch

													
		UART2_RecvType = UART2_RECV_CMD;
		HAL_UART_Receive_IT(&huart1,(uint8_t*)&UART2RecvData.cmd, sizeof(UART2RecvData.cmd));
			
			
			
		}
		
	}
	
	float CalculateSupply(int ANTlvl, int TRAlvl, int SUPlvl, char ANTnum)
	{
		float Utmp;
		
		if(SUPlvl == 0)
			Us = 310;
		else
			Us = 540;
					
		switch(ANTnum)
		{
			case 1:
				if(ANTlvl == 1){
					if(TRAlvl == 1)
						Utmp = Us*O1H/INH;
					else
						Utmp = Us*O1H/INL;}
				else{
					if(TRAlvl == 1)
						Utmp = Us*O1L/INH;
					else
						Utmp = Us*O1L/INL;}
				break;
			case 2:
				if(ANTlvl == 1){
					if(TRAlvl == 1)
						Utmp = Us*O2H/INH;
					else
						Utmp = Us*O2H/INL;}
				else{
					if(TRAlvl == 1)
						Utmp = Us*O2L/INH;
					else
						Utmp = Us*O2L/INL;}
				break;
			case 3:
				if(ANTlvl == 1){
					if(TRAlvl == 1)
						Utmp = Us*O3H/INH;
					else
						Utmp = Us*O3H/INL;}
				else{
					if(TRAlvl == 1)
						Utmp = Us*O3L/INH;
					else
						Utmp = Us*O3L/INL;}
				break;
			case 4:
				if(ANTlvl == 1){
					if(TRAlvl == 1)
						Utmp = Us*O4H/INH;
					else
						Utmp = Us*O4H/INL;}
				else{
					if(TRAlvl == 1)
						Utmp = Us*O4L/INH;
					else
						Utmp = Us*O4L/INL;}
				break;
		}
		return Utmp;
	}
		// ��������� ������� �� ������ � ������
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{

		switch(GPIO_Pin)
		{
#ifdef BUTT
			case GPIO_PIN_10:							// ������ ������
				if((def == 0)&&(STATUS.trans_state == 0)&&(sec > 0)){
					if(Mode == MAIN){
						if((tmpData == 0) || (tmpData == 65535))
							DiagnSendAlarm();
						else
							//SendData(tmpData);
							//Diag();
							DiagnSendData();
			}
					else if(Mode == AUX)
						StartExternPWM();
					else if(Mode == GEN)
						Generator();}
				break;
			case GPIO_PIN_11:							// �������/ �����
				StopPWM();
				if(def == 1)
				{
					HAL_GPIO_WritePin(GPIOC,GPIO_DEF_Pin,GPIO_PIN_RESET);
					HAL_TIM_OC_Start(&htim15,TIM_CHANNEL_1);
					def = 0;
				}
				break;
#endif
			case GPIO_PIN_12:							// ������
				def = 1;
				StopPWM();
				defcnt++;
				STATUS.overload_cnt = defcnt;
				STATUS.trans_ok = 0;
				STATUS.overload_curr = 0;
			
			for(uint8_t i = 0; i < sizeof(STATUS.im[0]); i++)
			{
				if(STATUS.im[i] > STATUS.overload_curr)
					STATUS.overload_curr = STATUS.im[i];
			}
			
		
			// �������� ��������� � ���� ����������
				CommandReply(U2CT_TRANS_OK, 'i', STATUS.trans_ok);
				CommandReply(U2CT_OVERLOAD_CNT, 'i', STATUS.overload_cnt);
				CommandReply(U2CT_OVERLOAD_CURR, 'f', STATUS.overload_curr);
				HAL_GPIO_WritePin(GPIOC,GPIO_DEF_Pin,GPIO_PIN_SET);
			break;
		}

		
	}
	
	void	HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp)
	{
		if(COMP_EXTI_GET_FLAG(COMP_EXTI_LINE_COMP2) != RESET)
  {
    /* Clear COMP EXTI pending bit */
    HAL_GPIO_WritePin(GPIOC,GPIO_DEF_Pin,GPIO_PIN_SET);
  }
		
	}

	void SaveSetting(void)
{
	FLASH_EraseInitTypeDef Flash;
	HAL_StatusTypeDef Status;
	uint8_t data = 0;
	uint32_t err;
	Status = HAL_FLASH_Unlock();
	
	Flash.PageAddress = ADR_START;
	Flash.TypeErase = FLASH_TYPEERASE_PAGES;
	Flash.NbPages = 1;
	HAL_FLASHEx_Erase(&Flash, &err);
	
	//FLASH_PageErase(ADR_START);

    // ������� 1
      Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START, *(uint32_t *)&f1);
		// ������� 2
      Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+4, *(uint32_t *)&f2);
      // �������� �������� 
      Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+8, *(uint32_t *)&BR);
      // �������
      Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+12, *(uint32_t *)&DeathTime_u);
      // ����������
      Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+16, *(uint32_t *)&Overlap_u);
      // ������������ ������������
      Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+20, *(uint32_t *)&TimeLimit_u);
      // ������ �������� ������
      Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+24, *(uint32_t *)&DiagTime);
      // ����������� ����
      Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+28, *(uint32_t *)&data);
      // ���������� ������������ ������ �� ����
      Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+32, *(uint32_t *)&defcnt);
      // ������� � �������1
			Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+36, *(uint32_t *)&C1);
			// ������� � �������2
			Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+40, *(uint32_t *)&C2);
			// ������� � �������3
			Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+44, *(uint32_t *)&C3);
			// ������� � �������4
			Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+48, *(uint32_t *)&C4);
			// ����� �������� ��������
			Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+52, *(uint32_t *)&RepeatNum);
			// ����� ������ ��������/�������
			Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+56, *(uint32_t *)&Mode);
			// ������� ������
			Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+60, *(uint32_t *)&SoftStart);
			// ����� ����������� ���� ���/����
			Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+64, *(uint32_t *)&data);
			// ����� �������� �������������, �������
			// ������� ���/����
			Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+68, *(uint32_t *)&SETUP.cap);
			// ��������� ���/���
			Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+72, *(uint32_t *)&data);
			// ���������
			Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+76, *(uint32_t *)&CodeMode);
			// ���������� �� ���� ����/ �����������
			Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+80, *(uint32_t *)&data);
			// ���������� �������
			Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+84, *(uint32_t *)&Us);
			// ���������� �������1
			Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+88, *(uint32_t *)&Us1);
			// ���������� �������2
			Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+92, *(uint32_t *)&Us2);
			// ���������� �������3
			Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+96, *(uint32_t *)&Us3);
			// ���������� �������4
			Status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ADR_START+100, *(uint32_t *)&Us4);
					

			
	Status = HAL_FLASH_Lock();
	
	
	
}

void LoadSetting(void)
{
	// ������� 1
	f1 =							*(__IO uint32_t*)(ADR_START);
	// ������� 2
	f2 =							*(__IO uint32_t*)(ADR_START + 4);
	// �������� ��������
	BR =							*(__IO float*)(ADR_START + 8);
	// �������
	DeathTime_u =			*(__IO uint32_t*)(ADR_START + 12);
  // ����������
  Overlap_u =				*(__IO uint32_t*)(ADR_START + 16);
  // ������������ ������������
  TimeLimit_u =			*(__IO uint32_t*)(ADR_START + 20);
  // ������ �������� ������
  DiagTime =				*(__IO uint32_t*)(ADR_START + 24);
  //// ����������� ����
  //data =						*(__IO uint32_t*)(ADR_START + 28);
  // ���������� ������������ ������ �� ����
  defcnt =					*(__IO uint32_t*)(ADR_START + 32);
  // ������� � �������1
	C1 =							*(__IO float*)(ADR_START + 36);
	// ������� � �������2
	C2 =							*(__IO float*)(ADR_START + 40);
	// ������� � �������3
	C3 =							*(__IO float*)(ADR_START + 44);
	// ������� � �������4
	C4 =							*(__IO float*)(ADR_START + 48);
	// ����� �������� ��������
	RepeatNum =				*(__IO uint32_t*)(ADR_START + 52);
	// ����� ������ ��������/�������
	Mode =						*(__IO uint32_t*)(ADR_START + 56);
	// ������� ������
	SoftStart =				*(__IO uint32_t*)(ADR_START + 60);
	//// ����� ����������� ���� ���/����
	//data =						*(__IO uint32_t*)(ADR_START + 64);
	// ����� �������� �������������, �������
	// ������� ���/����
	SETUP.cap =				*(__IO uint32_t*)(ADR_START + 68);
	//// ��������� ���/���
	//Status =					*(__IO uint32_t*)(ADR_START + 72);
	// ���������
	CodeMode =				*(__IO uint32_t*)(ADR_START + 76);
	//// ���������� �� ���� ����/ �����������
	//data =						*(__IO uint32_t*)(ADR_START + 80);
	// ���������� �������
	Us =							*(__IO float*)(ADR_START + 84);
	// ���������� �������1
	Us1 =							*(__IO float*)(ADR_START + 88);
	// ���������� �������2
	Us2 =							*(__IO float*)(ADR_START + 92);
	// ���������� �������3
	Us3 =							*(__IO float*)(ADR_START + 96);
	// ���������� �������4
	Us4 =							*(__IO float*)(ADR_START + 100);
}




#ifndef __GENERAL_MATH_H
#define __GENERAL_MATH_H

#include "stm32f3xx_hal.h"

// ������ �����������
#define pwmSTART		0
#define pwmBISY			1
#define pwmSTOP			2

// ����� ������
#define MAIN				1
#define AUX					0
#define GEN					2

// ����� ��������
#define SENDDATA		0
#define DEMAG				1
#define DIAG				2
#define GENERATOR		3
#define ALARM				4
#define TESTTAG			5


#define ALARM_TRANS	64
#define ALARM_PAUSE	32

// ����� �������� ������
#define FUSE				400e-3f				// ���������
#define BREAK				10e-3f		// �����

#define MEAN_CNT		1000

#define ON					1
#define OFF					0

// ����� ������������� ���������
#define Length			8

#define START_BIT		0
#define STOP_BIT		1
#define	LSB					0
#define MSB					1

#define ADR_START		0x0803F000U
#define MAXDAT			256				// ����� ������ �������� 

// ������ ������� ������
#define UART_TX_LENGTH 100
#define UART_RX_LENGTH 72

#define PI 3.1416f
#define CONST 1.58f
#define Vsupp 3.3f
#define Effbit 4096

// ���������� ������ � ��������
	// �������
#define INH 110			//88
#define INL 150			//110
	
	// �������� 1
#define O1H 150			//70
#define O1L 75			//35

	// �������� 2
#define O2H 110			//70
#define O2L 55			//35

	// �������� 3
#define O3H 50			//34
#define O3L 25			//17

	// �������� 4 
#define O4H 50			//34
#define O4L 25			//17

// ��������� �������� ���������� (������ ����� ��� �������� � ���� �� ����� ��� ���������
typedef struct StatusSystem {
	
  // ��������� �� �������� � ����������
  uint8_t overload_cnt;
  float overload_curr;
	uint8_t bitnum;						// ������� ���
	uint8_t repeatcur;				// ������� ������
	
  float ia[4];
	float im[4];
	
  float pa[4];
  float pm[4];
	
  float ra[4];
	
	float c[4];						// �������
	float l[4];						// �������������
	
  uint8_t trans_ok;
  uint8_t trans_state;
	uint8_t drvenable;				// ������� �������
	
	uint8_t ant_fuse[4];
	uint8_t ant_break[4];
  	
} StatusSystem_t, *pStatusSystem_t;

typedef struct SettingParametrs {
	// ��������� �����������
	uint32_t data;
	uint16_t freq1;
	uint16_t freq2;
	uint16_t boudrate;
	uint8_t deadtime;
	int8_t  overlap;
	uint16_t duration_limit;		//����������� ������������
	uint16_t check_ant_period;	// ������ ��������
	uint8_t current_limit;			// ����������� ����(�)
	uint8_t count_halt_limit;	// ����� ������������ ������
	float capatity_ant[4];
	uint8_t	 repeatnum;		//
	uint8_t	 enable;
	uint8_t	 mode;
	uint8_t	 softsrart;
	uint8_t	 currentlimit;				// ����� ����������� ����
	uint8_t	 diag;
	uint8_t	 cap;
	uint8_t	 modulation;
	uint8_t	 codemode;		//
	uint8_t	 overloadmode;
	uint8_t	 suppvoltage;	//
	uint8_t  alarm_msg;
	uint8_t  test_tag;
	uint8_t  command;
	
	uint8_t	 ant[4];
	uint8_t	 antlevel[4];
	uint8_t	 supplevel;		//
	
} SettingParametrs_t, *pSettingParametrs_t;

// ������ ������
typedef enum {CT_UNDEFINED = -1, 
							// I2C
              I2C_GET_TEMP_LVL_1_ON,
							I2C_GET_TEMP_LVL_2_ON,
							I2C_GET_TEMP_LVL_1_OFF,
							I2C_GET_TEMP_LVL_2_OFF,
							I2C_SET_TEMP_LVL_1_ON,
							I2C_SET_TEMP_LVL_2_ON,
							I2C_SET_TEMP_LVL_1_OFF,
							I2C_SET_TEMP_LVL_2_OFF,
							I2C_LED_ON,
							I2C_LED_OFF,
              // ������� �����������
              U2CT_DATA,								// ����� �������
              U2CT_F1,									// ������� 1
              U2CT_F2,									// ������� 2
              U2CT_BR,									// �������( �������� �������� ������)
              U2CT_DEATH_TIME,					// �������
              U2CT_OVERLAP,							// ����������
              U2CT_TIME_LIMIT,					// ����������� ������������
              U2CT_DIAG_PERIOD,					// ������ �������� ������
              U2CT_CURRENT_LIMIT_A,			// ����������� ����
              U2CT_OVER_TICK,						// ���������� ������������ ������
              U2CT_C1,									// ������� 1
              U2CT_C2,									// ������� 2
              U2CT_C3,									// ������� 3
              U2CT_C4,									// ������� 4
							U2CT_REPEATNUM,						// ����� �������� ��������
              U2CT_ENABLE,							// ������/������� ��������
              U2CT_MODE,								// ����� ������ ��������/�������
              U2CT_SOFT_START,					// ������� ����
              U2CT_CURRENT_LIMIT,				// ����� ����������� ����
              U2CT_DIAG,								// ����� �������� ������
              U2CT_CAP,									// ������� �������
              U2CT_MODUlATION,					// ���������
							U2CT_CODEMODE,						// ���������
              U2CT_OVERLOAD_MODE,				// ����� ���������� �� ���� ��������������/������
							U2CT_SUPPVOLTAGE,					// ���������� �������
							U2CT_ALARM_MSG,						// ������ ���������� ����������
							U2CT_TEST_TAG,						// �������� �����
							U2CT_COMMAND,							// ������� ��� �������
              U2CT_ANTENNA_1,						// ����������� ������� 1
              U2CT_ANTENNA_2,						// ����������� ������� 2
              U2CT_ANTENNA_3,						// ����������� ������� 3
              U2CT_ANTENNA_4,						// ����������� ������� 4
              U2CT_ANT_LEVEL_1,					// ���������� �� ������� 1
              U2CT_ANT_LEVEL_2,					// ���������� �� ������� 2
              U2CT_ANT_LEVEL_3,					// ���������� �� ������� 3
              U2CT_ANT_LEVEL_4,					// ���������� �� ������� 4
							U2CT_SUPPLEVEL,						// ���������� �� �������������� 
              U2CT_OVERLOAD_CNT,				// ����� ������������ ������ �� ����
              U2CT_OVERLOAD_CURR,				// ��� ������������ ������
							U2CT_BITNUM,							// ����� ����
							U2CT_REPEATCUR,						// ����� ������� �������
              U2CT_IA_1,								// ��� � ������� ������� 1
              U2CT_IM_1,								// ��� � ������� ������������ 1
              U2CT_IA_2,								// ��� � ������� ������� 2
              U2CT_IM_2,								// ��� � ������� ������������ 2
              U2CT_IA_3,								// ��� � ������� ������� 3
              U2CT_IM_3,								// ��� � ������� ������������ 3
              U2CT_IA_4,								// ��� � ������� ������� 4
              U2CT_IM_4,								// ��� � ������� ������������ 4
              U2CT_PA_1,								// �������� � ��������
              U2CT_PM_1,
              U2CT_PA_2,								// �������
              U2CT_PM_2,								// ������������								
              U2CT_PA_3,
              U2CT_PM_3,
              U2CT_PA_4,
              U2CT_PM_4,
              U2CT_RA_1,								// ������������� ������
              U2CT_RA_2,
              U2CT_RA_3,
              U2CT_RA_4,
							U2CT_CAP1,								// ���������� �������� ��������
							U2CT_CAP2,
							U2CT_CAP3,
							U2CT_CAP4,
							U2CT_L_1,									// ������������� ������
							U2CT_L_2,
							U2CT_L_3,
							U2CT_L_4,
              U2CT_TRANS_OK,						// ���������� ����������� 
              U2CT_STATE,								// ��������� ��������
							U2CT_DRVENABLE,						// ��������� �������� �����������
							U2CT_ANT_FUSE_1,					// �� � ��������
              U2CT_ANT_BREAK_1,					// ����� � ��������
              U2CT_ANT_FUSE_2,
              U2CT_ANT_BREAK_2,
              U2CT_ANT_FUSE_3,
              U2CT_ANT_BREAK_3,
              U2CT_ANT_FUSE_4,
              U2CT_ANT_BREAK_4,
							U2CT_SETUP,								// ������/�������� ��������
							U2CT_STATUS,							// �������� �������
              // ������
              CT_SETTINGS_STORE,
              CT_GET_STATUS
							
						} Cmd_Type;

// ��������� ������������ �������� ��� ���� Queue_Object_Data
// ������ ��������� - 4 �����
typedef union {
    uint32_t u32;
    uint16_t u16;
    uint8_t u8;
    int32_t i32;
    int16_t i16;
    int8_t i8;
    int i;
    float f;
} Queue_Object_Data_Value;

// ��������� ������ � �������
// ����� ���������:
//   Sender_Type              - 1 ����
//   Cmd_Type                 - 1 ����
//   ...                      - 2 ����� �� ������������ (������������ �� 4)
//   Queue_Object_Data_Value  - 4 �����

typedef struct {
  // �������
  Cmd_Type cmd;
  // ������
  Queue_Object_Data_Value value;
} UART2_Queue_Data;


typedef enum {UART2_RECV_CMD, UART2_RECV_VALUE} UART2Recv_t;



void TimingCalc(void);					// ������ ������������� ����������� ���������
void SetTiming(void);						// ��������� ���������
void StartPWM(void);						// ������ ���
void StopPWM(void);							// ��������� ��������

void CommandReply(Cmd_Type cmd, const char fmt, ...);		// �������� ������� �� ����
void SendPacketUART(void);															// �������� �������


void FormSTATUS(void);																	// ������������ ������� 

float CalculateSupply(int ANTlvl, int TRAlvl, int SUPlvl, char ANTnum);	//

// ����� �����
void SendData(uint32_t data);						
void ProcData(void);
void DiagnSendData(void);

// ������������ �����
void TestTag(uint32_t data);
void ProcTestTag(void);
void DiagnSendTestTag(void);

// ��������� �����
void SendAlarm(uint32_t data);
void ProcAlarm(void);
void DiagnSendAlarm(void);
void StopAlarm(void);

// ����� �������
void SendComand(uint8_t comand);
void DiagnSendComand(void);

// ����� ������� � �������
void SendComandnData(uint32_t data, uint8_t comand);
void DiagnSendComandnData(void);

// ������� ������ � ������ 
void Parsing(uint32_t data, uint8_t command);

// ������ ����������
float CalculateL(float Ur, float Usupp);
float CalculateI(float Ur);
float CalculateR(float Ur, float Uf, float Cap, float L);
float CalculateP(float I, float R);
float CalculateC(float L);

void CheckAntState(float U1r, float U2r, float U3r, float U4r);
void CalibrateMean(void);
void TransferInterrupt(void);

// ������ � ������� �����������
void StartExternPWM(void);
void ExternPWM(void);

// ��������� ��������� ���
void GetVoltage(void);
void CorrectTIM(void);

// ���������� ����������� ������
void Diag(void);
void ProcDiag(void);



void Demagnetization(void);
void ProcDemagnetization(void);

void ParsStruct(void);


void HAL_UART1_RxCpltCallback(UART_HandleTypeDef *huart);

// ��������� ����������
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void SaveSetting(void);
void LoadSetting(void);

void Generator(void);





#endif



#ifndef __GENERAL_MATH_H
#define __GENERAL_MATH_H

#include "stm32f3xx_hal.h"

// статус передатчика
#define pwmSTART		0
#define pwmBISY			1
#define pwmSTOP			2

// режим работы
#define MAIN				1
#define AUX					0
#define GEN					2

// режим передачи
#define SENDDATA		0
#define DEMAG				1
#define DIAG				2
#define GENERATOR		3
#define ALARM				4
#define TESTTAG			5


#define ALARM_TRANS	64
#define ALARM_PAUSE	32

// порог контроля антенн
#define FUSE				400e-3f				// замыкание
#define BREAK				10e-3f		// обрыв

#define MEAN_CNT		1000

#define ON					1
#define OFF					0

// длина передаваемого сообщения
#define Length			8

#define START_BIT		0
#define STOP_BIT		1
#define	LSB					0
#define MSB					1

#define ADR_START		0x0803F000U
#define MAXDAT			256				// длина буфера передачи 

// размер массива команд
#define UART_TX_LENGTH 100
#define UART_RX_LENGTH 72

#define PI 3.1416f
#define CONST 1.58f
#define Vsupp 3.3f
#define Effbit 4096

// количесиво витков в обмотках
	// входная
#define INH 110			//88
#define INL 150			//110
	
	// выходная 1
#define O1H 150			//70
#define O1L 75			//35

	// выходная 2
#define O2H 110			//70
#define O2L 55			//35

	// выходная 3
#define O3H 50			//34
#define O3L 25			//17

	// выходная 4 
#define O4H 50			//34
#define O4L 25			//17

// структура статусов устройства (отсюда берем для передачи и сюда же пишем все состояния
typedef struct StatusSystem {
	
  // состояния на отправку в передатчик
  uint8_t overload_cnt;
  float overload_curr;
	uint8_t bitnum;						// текущий бит
	uint8_t repeatcur;				// текущий повтор
	
  float ia[4];
	float im[4];
	
  float pa[4];
  float pm[4];
	
  float ra[4];
	
	float c[4];						// емкость
	float l[4];						// индуктивность
	
  uint8_t trans_ok;
  uint8_t trans_state;
	uint8_t drvenable;				// драйвер включен
	
	uint8_t ant_fuse[4];
	uint8_t ant_break[4];
  	
} StatusSystem_t, *pStatusSystem_t;

typedef struct SettingParametrs {
	// настройки передатчика
	uint32_t data;
	uint16_t freq1;
	uint16_t freq2;
	uint16_t boudrate;
	uint8_t deadtime;
	int8_t  overlap;
	uint16_t duration_limit;		//ограничение длительности
	uint16_t check_ant_period;	// период контроля
	uint8_t current_limit;			// ограничение тока(А)
	uint8_t count_halt_limit;	// число срабатываний защиты
	float capatity_ant[4];
	uint8_t	 repeatnum;		//
	uint8_t	 enable;
	uint8_t	 mode;
	uint8_t	 softsrart;
	uint8_t	 currentlimit;				// режим ограничения тока
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

// список команд
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
              // драйвер передатчика
              U2CT_DATA,								// номер шахтера
              U2CT_F1,									// частота 1
              U2CT_F2,									// частота 2
              U2CT_BR,									// бодрейт( скорость передачи данных)
              U2CT_DEATH_TIME,					// дедтайм
              U2CT_OVERLAP,							// перекрытие
              U2CT_TIME_LIMIT,					// ограничение длительности
              U2CT_DIAG_PERIOD,					// период контроля антенн
              U2CT_CURRENT_LIMIT_A,			// ограничение тока
              U2CT_OVER_TICK,						// количество срабатываний защиты
              U2CT_C1,									// емкость 1
              U2CT_C2,									// емкость 2
              U2CT_C3,									// емкость 3
              U2CT_C4,									// емкость 4
							U2CT_REPEATNUM,						// число повторов передачи
              U2CT_ENABLE,							// запуск/останов передачи
              U2CT_MODE,								// режим работы основной/внешний
              U2CT_SOFT_START,					// плавный пуск
              U2CT_CURRENT_LIMIT,				// режим ограничения тока
              U2CT_DIAG,								// режим контроля антенн
              U2CT_CAP,									// наличие емкости
              U2CT_MODUlATION,					// модуляция
							U2CT_CODEMODE,						// кодировка
              U2CT_OVERLOAD_MODE,				// режим перегрузки по току автоматический/ручной
							U2CT_SUPPVOLTAGE,					// напряжение питания
							U2CT_ALARM_MSG,						// запуск аварийного оповещения
							U2CT_TEST_TAG,						// проверка тагов
							U2CT_COMMAND,							// команда для радиуса
              U2CT_ANTENNA_1,						// подключение антенны 1
              U2CT_ANTENNA_2,						// подключение антенны 2
              U2CT_ANTENNA_3,						// подключение антенны 3
              U2CT_ANTENNA_4,						// подключение антенны 4
              U2CT_ANT_LEVEL_1,					// напряжение на антенне 1
              U2CT_ANT_LEVEL_2,					// напряжение на антенне 2
              U2CT_ANT_LEVEL_3,					// напряжение на антенне 3
              U2CT_ANT_LEVEL_4,					// напряжение на антенне 4
							U2CT_SUPPLEVEL,						// напряжение на трансформаторе 
              U2CT_OVERLOAD_CNT,				// число срабатывания защиты по току
              U2CT_OVERLOAD_CURR,				// ток срабатывания защиты
							U2CT_BITNUM,							// номер бита
							U2CT_REPEATCUR,						// номер текущей посылки
              U2CT_IA_1,								// ток в антенне средний 1
              U2CT_IM_1,								// ток в антенне максимальный 1
              U2CT_IA_2,								// ток в антенне средний 2
              U2CT_IM_2,								// ток в антенне максимальный 2
              U2CT_IA_3,								// ток в антенне средний 3
              U2CT_IM_3,								// ток в антенне максимальный 3
              U2CT_IA_4,								// ток в антенне средний 4
              U2CT_IM_4,								// ток в антенне максимальный 4
              U2CT_PA_1,								// мощности в антеннах
              U2CT_PM_1,
              U2CT_PA_2,								// средняя
              U2CT_PM_2,								// максимальная								
              U2CT_PA_3,
              U2CT_PM_3,
              U2CT_PA_4,
              U2CT_PM_4,
              U2CT_RA_1,								// сопротивление антенн
              U2CT_RA_2,
              U2CT_RA_3,
              U2CT_RA_4,
							U2CT_CAP1,								// уточненное значение емкостей
							U2CT_CAP2,
							U2CT_CAP3,
							U2CT_CAP4,
							U2CT_L_1,									// индуктивность антенн
							U2CT_L_2,
							U2CT_L_3,
							U2CT_L_4,
              U2CT_TRANS_OK,						// целосность передатчика 
              U2CT_STATE,								// состояние передачи
							U2CT_DRVENABLE,						// состояние драйвера передатчика
							U2CT_ANT_FUSE_1,					// кз в антеннах
              U2CT_ANT_BREAK_1,					// обрыв в антеннах
              U2CT_ANT_FUSE_2,
              U2CT_ANT_BREAK_2,
              U2CT_ANT_FUSE_3,
              U2CT_ANT_BREAK_3,
              U2CT_ANT_FUSE_4,
              U2CT_ANT_BREAK_4,
							U2CT_SETUP,								// запрос/отправка настроек
							U2CT_STATUS,							// отправка статуса
              // разное
              CT_SETTINGS_STORE,
              CT_GET_STATUS
							
						} Cmd_Type;

// структура передаваемых значений для типа Queue_Object_Data
// размер структуры - 4 байта
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

// структура данных в очереди
// схема структуры:
//   Sender_Type              - 1 байт
//   Cmd_Type                 - 1 байт
//   ...                      - 2 байта не используются (выравнивание до 4)
//   Queue_Object_Data_Value  - 4 байта

typedef struct {
  // команда
  Cmd_Type cmd;
  // данные
  Queue_Object_Data_Value value;
} UART2_Queue_Data;


typedef enum {UART2_RECV_CMD, UART2_RECV_VALUE} UART2Recv_t;



void TimingCalc(void);					// расчет длительностей управляющих импульсов
void SetTiming(void);						// установка регистров
void StartPWM(void);						// запуск шим
void StopPWM(void);							// остановка передачи

void CommandReply(Cmd_Type cmd, const char fmt, ...);		// отправка команды по уарт
void SendPacketUART(void);															// отправка статуса


void FormSTATUS(void);																	// формирование статуса 

float CalculateSupply(int ANTlvl, int TRAlvl, int SUPlvl, char ANTnum);	//

// вызов метки
void SendData(uint32_t data);						
void ProcData(void);
void DiagnSendData(void);

// тестирование меток
void TestTag(uint32_t data);
void ProcTestTag(void);
void DiagnSendTestTag(void);

// аварийный вызов
void SendAlarm(uint32_t data);
void ProcAlarm(void);
void DiagnSendAlarm(void);
void StopAlarm(void);

// вызов команды
void SendComand(uint8_t comand);
void DiagnSendComand(void);

// вызов команды с номером
void SendComandnData(uint32_t data, uint8_t comand);
void DiagnSendComandnData(void);

// парсинг номера в массив 
void Parsing(uint32_t data, uint8_t command);

// расчет параметров
float CalculateL(float Ur, float Usupp);
float CalculateI(float Ur);
float CalculateR(float Ur, float Uf, float Cap, float L);
float CalculateP(float I, float R);
float CalculateC(float L);

void CheckAntState(float U1r, float U2r, float U3r, float U4r);
void CalibrateMean(void);
void TransferInterrupt(void);

// работа с внешним генератором
void StartExternPWM(void);
void ExternPWM(void);

// обработка измерений ацп
void GetVoltage(void);
void CorrectTIM(void);

// проведение диагностики антенн
void Diag(void);
void ProcDiag(void);



void Demagnetization(void);
void ProcDemagnetization(void);

void ParsStruct(void);


void HAL_UART1_RxCpltCallback(UART_HandleTypeDef *huart);

// обработка прерываний
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void SaveSetting(void);
void LoadSetting(void);

void Generator(void);





#endif

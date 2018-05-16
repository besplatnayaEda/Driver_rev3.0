

#ifndef __GENERAL_MATH_H
#define __GENERAL_MATH_H

#include "stm32f3xx_hal.h"

// статус передатчика
#define pwmSTART		0	// запуск таймера
#define pwmBUSY			1	// таймер занят
#define pwmSTOP			2	// останов таймера

// режим работы
#define MAIN				1	// основной режим работы
#define AUX					0	// внешний режим работы
#define GEN					2	// генератор

// режим передачи
#define SENDDATA		0		// передача номера
#define DEMAG				1		// размагничивание сердечника (не используется)
#define DIAG				2		// диагностика
#define GENERATOR		3		// генератор
#define ALARM				4		// аварийное оповещение
#define TESTTAG			5		// тестирование меток


#define ALARM_TRANS	64	// время аварийного оповещения
#define ALARM_PAUSE	32	// пауза между аварийным оповещением

// порог контроля антенн
#define FUSE				400e-3f				// замыкание
#define BREAK				1e-3f		// обрыв

#define MEAN_CNT		1000		// усреднение 
#define MA_BUFF_LEN	10			// размер буфера скользящего среднего

#define ON					1	
#define OFF					0

// длина передаваемого сообщения
#define Length			8

#define START_BIT		0		// значение старт бита
#define STOP_BIT		1		// значение стоп бита
#define	LSB					0		// младшиший бит
#define MSB					1		// старший бит

#define ADR_START		0x0803F000U	//адрес для сохранения настроек
#define MAXDAT			1280				// длина буфера передачи 

// размер массива команд (неиспользуется)
#define UART_TX_LENGTH 100
#define UART_RX_LENGTH 72

// константы
#define PI 3.1416f
#define CONST 1.58f
#define Vsupp 3.3f
#define Effbit 4096

// версия ПО и платы драйвера
#define FIRMWARE	0x2293U		// ver 4.5.18-3
#define HARDWARE	0x2991U

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
  uint8_t overload_cnt;			// кличество срабатываний защиты
  float overload_curr;			// ток срабатывания защиты
	uint8_t bitnum;						// текущий бит
	uint8_t repeatcur;				// текущий повтор
	
  float ia[4];							// усредненный амплетудный ток в антенне
	float im[4];							// максимальное значение тока в антенне
	
  float pa[4];							// усредненное значение мощности в антенне
  float pm[4];							// максимальное значение мощности в антенне
	
  float ra[4];							// сопротивление антенны
	
	float c[4];						// емкость антенны
	float l[4];						// индуктивность антенны
	
  uint8_t trans_ok;			// состояние целостности передатчика 1 - рабочий, 0 - сработало ограничение по защите
  uint8_t trans_state;	// состояние работы передачи 1 - передача, 2 - контроль, 0 - ожидание
	uint8_t drvenable;				// состояние активности драйвера 1 - драйвер включен, 0 - драйвер в режиме ожидания 
	
	uint8_t ant_fuse[4];			// костояние кз в антенне, 0 - нет, 1 - кз
	uint8_t ant_break[4];			// состояние обрыва антенны, 0 - нет, 1 - обрыв
	uint16_t driver_fw;			//версия по драйвера
	uint16_t driver_hw;			//версия блока драйвера
  	
} StatusSystem_t, *pStatusSystem_t;

typedef struct SettingParametrs {
	// настройки передатчика
	uint32_t data;					// вызываемый номер метки
	uint16_t freq1;					// частота "1" 
	uint16_t freq2;					// частота "0"
	uint16_t boudrate;			// бодрейт
	uint8_t deadtime;				// пауза между открытиями ключей в мкс
	int8_t  overlap;				// перекрытие нижних ключей в мкс
	uint16_t duration_limit;		//ограничение длительности импульса в мкс
	uint16_t check_ant_period;	// период контроля	в мин
	uint8_t current_limit;			// ограничение тока(А)
	uint8_t count_halt_limit;	// число срабатываний защиты
	float capatity_ant[4];		// ёмкость в антеннах
	uint8_t i_break[4];			// ток обрыва антенны
	uint8_t i_fuse[4];			// ток короткого замыкания антенны
	uint8_t	 repeatnum;		// число повторов передачи
	uint8_t	 enable;			// запуск передачи
	uint8_t	 mode;				// режим работы  1 - основной, 0 - внешний, 2 - генератор
	uint8_t	 softsrart;		// плавный запуск
	uint8_t	 currentlimit;				// режим ограничения тока
	uint8_t	 diag;				// запуск диагностики
	uint8_t	 cap;					// емкость подлючена/отключена
	uint8_t	 modulation;	// модуляция psk/fsk
	uint8_t	 codemode;		// кодировка 0 - бинарная, 1 - радиус 1, 2 - радиус 2  
	uint8_t	 overloadmode;	// режим перегрузки по току 1 - автоматический, 0 - одиночный
	uint8_t	 suppvoltage;	// напряжение питания 540 - высокое, 310 - низкое
	uint8_t  alarm_msg;		// запуск аварийного оповещения
	uint8_t  test_tag;		// запуск тестового сообщения 
	uint8_t  command;	  // запуск передачи команды (радиус) 
	
	uint8_t	 ant[4];		// состояние антенн 1  - подключена, 0 отключена, -1 - отсутствует
	uint8_t	 antlevel[4];	// выход антенны 1  - высокое, 0 - низкое
	uint16_t turns_ant[4]; // количество витков во вторичной обмотке (полное)
	uint16_t turns_prim;	// количество витков в первичной обмотке
	uint8_t	 supplevel;		//	напряжение первичной обмотки относительно выхода 1 -высокое, 0 низкое
	uint8_t standby;			//режим ожидания вкл/выкл
	
} SettingParametrs_t, *pSettingParametrs_t;

// список команд
typedef enum {CT_UNDEFINED = -1, 
							// I2C
              I2C_GET_TEMP_LVL_1_ON,		// команды ля i2c датчика температуры (не используются в драйвере)
              I2C_GET_TEMP_LVL_2_ON,
              I2C_GET_TEMP_LVL_1_OFF,
              I2C_GET_TEMP_LVL_2_OFF,
              I2C_SET_TEMP_LVL_1_ON,
              I2C_SET_TEMP_LVL_2_ON,
              I2C_SET_TEMP_LVL_1_OFF,
              I2C_SET_TEMP_LVL_2_OFF,
              I2C_LED_ON,								// команды для i2c индикации (не используются в драйвере)
              I2C_LED_OFF,
              // драйвер передатчика
              U2CT_DATA,//i				// передача номера в драйвер
              U2CT_F1,//i16				// передача f1 в драйвер
              U2CT_F2,//i16				// передача f2 в драйвер
              U2CT_BR,//i16				// передача бодрейта в драйвер
              U2CT_DEATH_TIME,//i8	// передача паузы в драйвер
              U2CT_OVERLAP,//i8			// передача перекрытия в драйвер
              U2CT_TIME_LIMIT,//i16			// передача ограничения длительности импульса в драйвер
              U2CT_DIAG_PERIOD,//i16		// передача периода диагностики в драйвер
              U2CT_CURRENT_LIMIT_A,//i8		// передача значения ограничения тока в драйвер
              U2CT_OVER_TICK,//i8				// передача количества срабатывания защиты в драйвер
              U2CT_C1,//f		// передача емкости в антенне в драйвер
              U2CT_C2,//f
              U2CT_C3,//f
              U2CT_C4,//f
				U2CT_I_BREAK_ANT_1,//i8	//передача тока обрыва антенны  в драйвер
				U2CT_I_BREAK_ANT_2,//i8
				U2CT_I_BREAK_ANT_3,//i8
				U2CT_I_BREAK_ANT_4,//i8
				U2CT_I_FUSE_ANT_1,//i8	// передача тока замыкания антенны	 в драйвер
				U2CT_I_FUSE_ANT_2,//i8
				U2CT_I_FUSE_ANT_3,//i8
				U2CT_I_FUSE_ANT_4,//i8
              U2CT_REPEATNUM,//i8		// передача числа повторов в драйвер
              U2CT_ENABLE,//i8			// команда запуска передачи в драйвер
              U2CT_MODE,//i8			// передача режима работы генератора в драйвер
              U2CT_SOFT_START,//i8 // передача режима плавного запуска в драйвер
              U2CT_CURRENT_LIMIT,//i8	// передача режима ограничкения тока в драйвер
              U2CT_DIAG,//i8	// команда запуска диагностики в драйвер
              U2CT_CAP,//i8  // передача режима емкости в драйвер
              U2CT_MODUlATION,//i8	// передача режима модуляции в драйвер
              U2CT_CODE_MODE,//i8 // передача кодировки в драйвер
              U2CT_OVERLOAD_MODE,//i8	// передача режима перегрузки по току в драйвер
              U2CT_SUPP_VOLTAGE,//i8	// передача режима напряжения питания в драйвер
              U2CT_ALARM_MSG,//i8 // передача команды на запуск аварийного оповещения в драйвер
              U2CT_TEST_TAG,//i8		// перердача команды на тест меток в драйвер
              U2CT_COMMAND,//i8		// передача отправки команды  в драйвер
              U2CT_ANTENNA_1,//i8	// передача режима работы антенн в драйвер
              U2CT_ANTENNA_2,//i8
              U2CT_ANTENNA_3,//i8
              U2CT_ANTENNA_4,//i8
              U2CT_ANT_LEVEL_1,//i8	// передача выходного напряжения антенн в драйвер
              U2CT_ANT_LEVEL_2,//i8
              U2CT_ANT_LEVEL_3,//i8
              U2CT_ANT_LEVEL_4,//i8
				U2CT_TURNS_ANT_1,//i16		// передача кол-ва витков в антенне в драйвер
				U2CT_TURNS_ANT_2,//i16
				U2CT_TURNS_ANT_3,//i16
				U2CT_TURNS_ANT_4,//i16
				U2CT_TURNS_PRIM,//i16
              U2CT_SUPPLY_LEVEL,//i8	//передача напряжения первичной обмотки в драйвер
				U2CT_STANDBY,//i8						// передача режима ожидания в драйвер
              U2CT_OVERLOAD_CNT,//i8	// передача количества срабатывания защиты в бу
              U2CT_OVERLOAD_CURR,//f	// передача тока срабатываня защиты в бу
              U2CT_BITNUM,//i8		// передача текущего бита  в бу
              U2CT_REPEAT_CUR,//i8 // передача текущего повтора в бу
              U2CT_IA_1,//f	// передача усредненного значения амплитуды тока в бу
              U2CT_IM_1,//f	// перердача макимального значения амплитуды тока в бу
              U2CT_IA_2,//f
              U2CT_IM_2,//f
              U2CT_IA_3,//f
              U2CT_IM_3,//f
              U2CT_IA_4,//f
              U2CT_IM_4,//f
              U2CT_PA_1,//f	// передача усредненного значения мощности в бу
              U2CT_PM_1,//f	// перердача макимального значения мощности в бу
              U2CT_PA_2,//f
              U2CT_PM_2,//f
              U2CT_PA_3,//f
              U2CT_PM_3,//f
              U2CT_PA_4,//f
              U2CT_PM_4,//f
              U2CT_RA_1,//f	 //перердача сопротивления антенны в бу
              U2CT_RA_2,//f
              U2CT_RA_3,//f
              U2CT_RA_4,//f
              U2CT_C_1,//f	// передача  расчитанного значения емкости в бу
              U2CT_C_2,//f
              U2CT_C_3,//f
              U2CT_C_4,//f
              U2CT_L_1,//f	 // передача расчитанного значения индуктивности в бу
              U2CT_L_2,//f
              U2CT_L_3,//f
              U2CT_L_4,//f
              U2CT_TRANS_OK,//i8	// передача целостности передатчика в бу
              U2CT_STATE,//i8		// передача состояниня передатчика в бу
              U2CT_DRVENABLE,//i8 // передача состояния режима ожидания в бу
              U2CT_ANT_FUSE_1,//i8	// передача аварии по кз в антенне в бу
              U2CT_ANT_BREAK_1,//i8 // передача аварии по обрыву в антенне в бу
              U2CT_ANT_FUSE_2,//i8
              U2CT_ANT_BREAK_2,//i8
              U2CT_ANT_FUSE_3,//i8
              U2CT_ANT_BREAK_3,//i8
              U2CT_ANT_FUSE_4,//i8
              U2CT_ANT_BREAK_4,//i8
				U2CT_DRIVER_FW,//i16		// перердача версии По драйвера в бу
				U2CT_DRIVER_HW,//i16		// передача версии платы драйвера в бу
              U2CT_SETUP,				// передача структуры настроек в драйвер  в бу
              U2CT_STATUS,			// передача структуры статуса в БУ в бу
              // разное
              CT_SETTINGS_STORE,	// не используются в драйвере в бу
              CT_GET_STATUS			// не используются в драйвере в бу
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


// тип принимаемых данных в uart
typedef enum {UART2_RECV_CMD, UART2_RECV_VALUE} UART2Recv_t;



void TimingCalc(void);					// расчет длительностей управляющих импульсов
void SetTiming(void);						// установка регистров
void StartPWM(void);						// запуск шим
void StopPWM(void);							// остановка передачи
void StartPWMdiag(void);				// запуск диагностики
void StopPWMdiag(void);					// остановка диагностики

void CommandReply(Cmd_Type cmd, const char fmt, ...);		// отправка команды по уарт
void SendPacketUART(void);															// отправка статуса


void FormSTATUS(void);																	// формирование статуса
float MAfilter(float * buff, float data);								// фильтр скользящее среднее

float CalculateSupply(int ANTlvl, int TRAlvl, int SUPlvl, char ANTnum);	// расчет выходного напряжения питания на антенне

float CalculateTransVoltage(uint8_t I_forb, uint8_t Utype);		// расчет порога срабатывания защиты по току в антеннах

// вызов метки
void SendData(uint32_t data);							// вызов метки
void ProcData(void);											// обработчик вызова
void DiagnSendData(void);									// запуск диагностики перед вызовом

// тестирование меток
void TestTag(uint32_t data);							// тест меток
void ProcTestTag(void);										// обработчик тестового вызова
void DiagnSendTestTag(void);							// запуск диагностики перед тестовывм вызовом

// аварийный вызов
void SendAlarm(uint32_t data);						// запуск аварийного оповещения
void ProcAlarm(void);											// обработчик аварийного оповещения
void DiagnSendAlarm(void);								// запуск диагностики перед оповещением
void StopAlarm(void);											// остановка аварийного оповещения

// вызов команды
void SendComand(uint8_t comand);					// отправка команды в метку (радиус-2)
void DiagnSendComand(void);								// запуск диагностики перед отправкой

// вызов команды с номером
void SendComandnData(uint32_t data, uint8_t comand);	// отправка команды в метку с номером (радиус-2)
void DiagnSendComandnData(void);											// запуск диагностики перед отправкой

// парсинг номера в массив 
void Parsing(uint32_t data, uint8_t command);		// преобразования номера в кодированный массив бит

// расчет параметров I,L,R,P,C
float CalculateL(float Ur, float Uf, float Usupp);
float CalculateI(float Ur);
float CalculateR(float Ur, float Uf, float Cap, float L);
float CalculateP(float I, float R);
float CalculateC(float L);

// работа с антеннами
void CheckAntState(float U1r, float U2r, float U3r, float U4r);	// проверка состояния антенн
void CalibrateMean(void);		// калибровка средней точки
void TransferInterrupt(void);	// прерывание передачи для отключения контактора

// работа с внешним генератором
void StartExternPWM(void);	// запуск от внешнего генератора
void ExternPWM(void);				// обработка сигналов с внешнего генератора

// обработка измерений ацп
void GetVoltage(void);			// измерение сигналов с трансформаторов напряжения
void CorrectTIM(void);			// корректировка момента измерения 

// проведение диагностики антенн
void Diag(void);		// диагностика антенн
void ProcDiag(void);	// обработка диагностики антенн


// размагничивание (не используется)
void Demagnetization(void);		// запуск размагничивания 
void ProcDemagnetization(void);	// обработка размагичивания



// обработка данных с UART
void HAL_UART1_RxCpltCallback(UART_HandleTypeDef *huart);

// обработка прерываний
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


void SaveSetting(void);		// сохранение настроек
void LoadSetting(void);		// загрузка настроек

void Generator(void);		// запуск генратора

uint8_t CRC8(uint8_t *pcBlock, uint8_t len);	// расчет crc8 радиус2м



#endif

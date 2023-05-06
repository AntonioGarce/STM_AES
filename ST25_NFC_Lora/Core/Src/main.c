/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <string.h>
#include <stdint.h>

#include "aes.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	BOARD_TEST		1

// #define VDD_APPLI                      ((uint32_t)3114)    /* Value of analog voltage supply Vdda (unit: mV) */
#define RANGE_12BITS                   ((uint32_t)4095)    /* Max value with a full range of 12 bits */
#define TIMER_FREQUENCY_HZ             ((uint32_t)1000)    /* Timer frequency (unit: Hz). With SysClk set to 2MHz, timer frequency TIMER_FREQUENCY_HZ range is min=1Hz, max=33.3kHz. */

#define VBAT_CHANNEL										0
#define PS_IN_CHANNEL										1
#define ADC_VBAT_CHANNEL								ADC_CHANNEL_0
#define ADC_PS_IN_CHANNEL								ADC_CHANNEL_4

#define VBAT_RATIO											(double)(1)		
#define PS_IN_RATIO											(double)(1)
	
#define COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE_33(ADC_DATA)                        \
  ( (ADC_DATA) * 3114 / RANGE_12BITS)

#define COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE_30(ADC_DATA)                        \
  ( (ADC_DATA) * 2830 / RANGE_12BITS)
	
#define LORA_MAX_LEN									50
#define NFC_MAX_LEN										64

#define NFC_I2C_SLAVE_ADDR 			(0x53 << 1)
#define TMP75_I2C_SLAVE_ADDR 		(0x48 << 1)
#define AMS5915_I2C_SLAVE_ADDR 	(0x28 << 1)
#define I2C_TIMEOUT		2000

#define	GET_DEVEUI						  "mac get deveui\r\n"	
#define	SET_APPEUI						  "mac set appeui 1122334455667788\r\n" 
#define	SET_NWKSKEY						  "mac set nwkskey 22334455667788992233445566778899\r\n" 
#define	SET_APPSKEY						  "mac set appskey 33445566778899003344556677889900\r\n" 
#define	RADIO_SET						    "radio set cr 4/7\r\n"  
#define	SET_DATARATE_SF9				"mac set dr 1\r\n"   // SF9 to comply with max 400 ms tx time 
#define	SET_DATARATE_SF10				"mac set dr 0\r\n"   // SF10 for longer distance, but may not be allowed 
#define	SET_POWERINDEX					"mac set pwridx 5\r\n"  // max power for US 18.5 dBm
#define	SET_ADR_OFF		 					"mac set adr off\r\n"  
#define	SAVE_CONF								"mac save\r\n"  
#define	JOIN_ABP								"mac join abp\r\n"  
#define	SLEEP										"sys sleep 3333333333\r\n"	
#define	SET_UPCTR								"mac set upctr 0\r\n"  
#define SET_SYNC								"mac set sync 34\r\n"
#define	SET_CLASS						    "mac set class c\r\n" 
#define	SET_AR						      "mac set ar on\r\n" 
#define	SET_RX2							    "mac set rx2 8 923300000\r\n" 
#define RESET_DOWNLCNT					"mac set dnctr 0\r\n"
#define DISABLE_RX_WATCHDOG			"radio set wdt 0\r\n"
#define MAC_PAUSE	 							"mac pause\r\n"
#define MAC_RESUME	 						"mac resume\r\n"
#define CONTINUOUS_RX	 					"radio rx 0\r\n"


#define	GET_RX2							    "mac get rx2\r\n" 
#define	GET_STATUS				      "mac get status\r\n" 
#define	RX_MESSAGE				      "mac_rx 1 " 

#define	FACTORY_RESET				    "sys factoryRESET\r\n" 
#define	GET_VERSION				      "sys get ver\r\n" 


/* define for crypto */
#define CBC 1
#define CTR 1
#define ECB 1

char	SET_DEVADDR[]	=				  	{ "mac set devaddr 00000009\r\n" };
//char	SET_DEVADDR[]	=				  	{ "mac set devaddr 00070009\r\n" };
char	SET_DEVEUI[]	=						{ "mac set deveui 2043685420436854\r\n" };
//char	SET_DEVEUI[]	=						{ "mac set deveui 2043685420170000\r\n" };
char	TX_UNCNF[]		=						{ "mac tx uncnf 3 00000000000000000000\r\n" };
char  SET_CH_STATUS_SH[] =			{ "mac set ch status 0 off\r\n" };
char  SET_CH_STATUS[] =					{ "mac set ch status 10 off\r\n" };
char  SET_CH_STATUS_ON[] =			{ "mac set ch status 40 on\r\n" };

const char * hex = "0123456789ABCDEF";

#define ITM_STIM_U32 (*(volatile unsigned int*)0xE0000000)    // Stimulus Port Register word acces
#define ITM_STIM_U8  (*(volatile         char*)0xE0000000)    // Stimulus Port Register byte acces
#define ITM_ENA      (*(volatile unsigned int*)0xE0000E00)    // Trace Enable Ports Register
#define ITM_TCR      (*(volatile unsigned int*)0xE0000E80)    // Trace control register

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

IWDG_HandleTypeDef hiwdg;

/* USER CODE BEGIN PV */

/* variables for crypto*/
uint8_t key[16] = { 0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6, 0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c };
uint8_t in[16]  = { 0x87, 0x4d, 0x61, 0x91, 0xb6, 0x20, 0xe3, 0x26, 0x1b, 0xef, 0x68, 0x64, 0x99, 0x0d, 0xb6, 0xce };
      
uint8_t out[16];
struct AES_ctx ctx;

/*         */
	uint8_t cindex =0;
__IO uint32_t  adcValue[2] = {0};
__IO uint32_t  vbat, psin, nfc_cnt;
uint8_t lora_rxbuffer[LORA_MAX_LEN], lora_rxtemp, lora_rxlen = 0;
uint8_t rx_command_and_value[5];
uint8_t nfc_rxbuffer[NFC_MAX_LEN], nfc_rxlen = 0;
uint8_t read_NFC_flag=0, valve_input, state=0, alarm=0, data_rate=0, msg_cnt=0;
uint8_t output1_status=0, output2_status=0, output3_status=0;
uint8_t bat_l_cnt=0, stopped=0, rx_hex_ok = 0, rx_command=0;
uint16_t valve_open_time_buffer[20];
uint32_t passivation_cnt=0;
uint16_t temperature, pressure_cnt=0, output_on_time=0, output_cnt, a, b;
uint8_t  psin_req = 0, vbat_req=0, send_msg_active = 0, bad_cnt=0, rx_cnt=0;
uint16_t psin_powerup_delay_cnt = 0;
uint32_t vbat_cnt=0;
uint32_t vbat_update_time = 86400;  //86400
uint16_t RTC_cali=2413, vbat_cali=5000;
uint16_t psin_cali=4500;
uint16_t pressure1_alarm_cnt=0, pressure2_alarm_cnt=0;
uint16_t pressure1_not_alarm_cnt=0, pressure2_not_alarm_cnt=0;
uint8_t pressure1_high=0, pressure2_high=0;
uint16_t msg_interval_time_cnt=0;
uint8_t NFC_loaded = 0;
uint8_t system_reset_cnt=0, NFC_byte63, supply_mode, NFC_read_OK, read_NFC_byte63_flag = 0;
uint8_t test_fail=0;
uint8_t input1_activated=1, test_cnt;
uint16_t input1_cnt=0, input1_cnt_buffer=0;
uint8_t operation_mode=0, test_LED_cnt=0, get_psin_flag=0, get_vbat_flag=0, get_pressure_flag, test_completed=0;
uint8_t string_match;
uint8_t block_rx_cnt, dummy1=0, dummy2=0, dummy3=0;

/*
RTC_cali : byte 60-61 i NFC
vbat_cali: byte 56-57 i NFC
psin_cali: byte 52-53 i NFC
*/


//state indicator flags and associated time counters
uint8_t valve_alarm=0, valve_warning=0, float_alarm=0, pressure1_alarm=0, pressure2_alarm=0, low_bat=0;
uint16_t valve_alarm_cnt=0, valve_warning_cnt=0, float_cnt=0;
uint16_t valve_open_time_measure, valve_operations_cnt, float_open_time_measure, main_time_cnt;
uint16_t NFC_update_time_cnt;
uint8_t valve_operations_rollover=0;

//digital input state flags and associated filters 
uint8_t valve_open_filtered=0, float_open_filtered=0, input3_open_filtered=0,	valve_filt_cnt=0, float_filt_cnt=0, input3_filt_cnt=0;

//analog input values
__IO uint32_t  pressure1=0,	pressure2=0, AMS5915_pressure=0;

//operational parameters read from NFC and transformed to internal format (0,25s per cnt)

uint16_t device_addr;     					//  0 ..10000
uint16_t heartbeat_time;    				//  2400 .. 61200		10 .. 255 (min)
uint16_t valve_alarm_time;    			//  0 .. 1020				0 .. 255 (s)
uint16_t valve_warning_time;	  		//	0 .. 1020				0 .. 255 (s)
uint16_t float_alarm_time;  				//  0 .. 1020				0 .. 255 (s)
uint16_t pressure_upd_time;	      	//  0 .. 60000			0 .. 15000 (s)
uint16_t pressure_alarm_time;	    	//	0 .. 1020				0 .. 255 (s)
uint16_t pressure1_limit;	    			//  0 .. 1000				(hvis 0: alarm diabled)
uint16_t pressure2_limit;	    			//  0 .. 1000				(hvis 0: alarm diabled)
uint8_t  reed_polarity;	  					//  0: NC	1: NO


extern I2C_HandleTypeDef hi2c1;

static uint8_t m_i2c_buffer[50];
uint8_t valve_open_time_ptr=0;
uint16_t valve_open_time_max;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_RTC_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */

static void modem_reset(void);
static void modem_wakeup(void);
static void Modem_save_conf(void);
static void modem_init(void);
static void save_valve_open_time(void);
static void calc_valve_open_time_max(void);
static void write_NFC(void);
static void write_NFC_type_rev(void);
static void write_NFC_no_FW_pending(void);
static void write_NFC_factory_default(void);
static void read_NFC(void);
static void read_NFC_valve_cnt(void);
static void (read_NFC_calibration)(void);
static void (read_NFC_loaded)(void);
static void (read_NFC_byte63)(void);
static void (read_NFC_supply_mode) (void);
static void send_msg(void);
static void (TMP75_sleep)(void);
static void (get_vbat)(void);
static void (get_psin)(void);
static void (read_AMS5915)(void);
static void (Delay_100)(void);
static void (Delay_1000)(void);
static void set_channels(void);
static void (start_msg_seq)(void);
static void check_operation_mode(void);
static void (calibrate_RTC)(void);
static void (test_inputs)(void);
static void (test_calibrate_psin)(void);
static void (test_onboard_pressure_sensor)(void);
static void (test_calibrate_vbat)(void);
static void (test_depassivation)(void);
static void count_input1_pulses (uint8_t);
static void reset_upctr(void);
static void (read_NFC_data_rate)(void);
static void parse_rxstring();
static void check_uart();
static void reinit_UART(void);
static void (write_TMP75_conf)(void);
static void (rx_init)(void);
/*AES-128-CTR encrypt/decrypt*/
static void AES_128_xcrypt(uint8_t* in, uint8_t* out, uint16_t length, uint8_t* key);


void pin_init(void);
void pin_deinit(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
	/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
	 set to 'Yes') calls __io_putchar() */
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
	


void lora_printf(const char *strVal)
{
	HAL_UART_Transmit_IT(&huart2, (uint8_t *)strVal, strlen(strVal));
}


void lora_write(uint8_t *buffer, uint8_t len)
{
		HAL_UART_Transmit_IT(&huart2, buffer, len);
}

//TODO: change to a boolean
void lora_autobaud()
{
	
	uint8_t txbuffer[10];
 
//	lora_rxlen = 0;
//	memset(lora_rxbuffer, 0, LORA_MAX_LEN);
		
	txbuffer[0]= 0x00;
	txbuffer[1]= 0x55;		
	txbuffer[2]= 0X0D;
	txbuffer[3]= 0X0A;
	txbuffer[4]= 0X00;

  lora_write(txbuffer, 5);	
}

HAL_StatusTypeDef Get_ADC_Input( uint8_t channel, uint32_t *ad_value )
{
	ADC_ChannelConfTypeDef sConfig = {0};
	
	switch( channel )
	{
		case VBAT_CHANNEL:
			sConfig.Channel = ADC_VBAT_CHANNEL;
			sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
			break;
		
		case PS_IN_CHANNEL:
			sConfig.Channel = ADC_PS_IN_CHANNEL;
			sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
			break;
	}
	
	HAL_ADC_ConfigChannel(&hadc, &sConfig);
	
	HAL_ADC_Start( &hadc );
		
	HAL_ADC_PollForConversion( &hadc, 10 );
		
	if ((HAL_ADC_GetState(&hadc) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
	{if (channel == VBAT_CHANNEL) *ad_value = HAL_ADC_GetValue(&hadc);}
	
	HAL_ADC_Start( &hadc );
	
	HAL_ADC_PollForConversion( &hadc, 10 );
	
	if ((HAL_ADC_GetState(&hadc) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
	{if (channel == PS_IN_CHANNEL) *ad_value = HAL_ADC_GetValue(&hadc);}

	HAL_ADC_Stop(&hadc);
}

// Get voltage value(mV unit) from every channels
int Get_VoltageValue( uint8_t ChannelId)
{
	uint32_t adc_value=0;
	int voltage_value=0;
	
	Get_ADC_Input( ChannelId, &adc_value );
	
	switch( ChannelId )
	{
		case VBAT_CHANNEL:
			if (supply_mode == 0)
			voltage_value = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE_33( adc_value ) * VBAT_RATIO;
			if (supply_mode == 1)
			voltage_value = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE_30( adc_value ) * VBAT_RATIO;	
			break;
		case PS_IN_CHANNEL:
			if (supply_mode == 0)
			voltage_value = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE_33( adc_value ) * PS_IN_RATIO;
			if (supply_mode == 1)
			voltage_value = COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE_30( adc_value ) * PS_IN_RATIO;
			break;
	}
	return voltage_value;
}

//typedef int32_t (*ST25DV_Write_Func)(uint16_t, uint16_t, const uint8_t*, uint16_t);
//typedef int32_t (*ST25DV_Read_Func) (uint16_t, uint16_t, uint8_t*, uint16_t);
static HAL_StatusTypeDef i2c_nfc_write(uint16_t dev_addr, uint16_t reg_addr, uint8_t* pData, uint16_t size)
{
	m_i2c_buffer[0] = (reg_addr >> 8) & 0x0FF;
	m_i2c_buffer[1] = reg_addr & 0x0FF;
	memcpy(m_i2c_buffer + 2, pData, size);
	
	HAL_StatusTypeDef ret_code = HAL_I2C_Master_Transmit(&hi2c1, dev_addr, m_i2c_buffer, size + 2, I2C_TIMEOUT);
	
	return ret_code;
}

static HAL_StatusTypeDef nfc_setup_GPO(void)
{
	for (uint8_t i=0; i<19; i++) m_i2c_buffer[i] = 0x00;
	m_i2c_buffer[0] = 0x09;
	m_i2c_buffer[10] = 0x09;
	HAL_I2C_Master_Transmit(&hi2c1, 0xAE, m_i2c_buffer, 19, I2C_TIMEOUT);
	Delay_1000();
	m_i2c_buffer[0] = 0x00;
	m_i2c_buffer[1] = 0x00;
	m_i2c_buffer[2] = 0x88;
	HAL_I2C_Master_Transmit(&hi2c1, 0xAE, m_i2c_buffer, 3, I2C_TIMEOUT);
  Delay_1000();
	m_i2c_buffer[0] = 0x00;
	m_i2c_buffer[1] = 0x0D;
	m_i2c_buffer[2] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, 0xAE, m_i2c_buffer, 3, I2C_TIMEOUT);
  Delay_1000();
}


static HAL_StatusTypeDef i2c_nfc_read(uint16_t dev_addr, uint16_t reg_addr, uint8_t* pData, uint16_t size)
{
	HAL_StatusTypeDef ret_code = HAL_OK;
	m_i2c_buffer[0] = (reg_addr >> 8) & 0x0FF;
	m_i2c_buffer[1] = reg_addr & 0x0FF;
	ret_code = HAL_I2C_Master_Transmit(&hi2c1, dev_addr, m_i2c_buffer, 2, I2C_TIMEOUT);
	
	if (ret_code == HAL_OK) NFC_read_OK = 1; else NFC_read_OK = 0; 
	
	if (ret_code != HAL_OK) return ret_code;
	
	return HAL_I2C_Master_Receive(&hi2c1, dev_addr, pData, size, I2C_TIMEOUT);
}


void stm32L_lowPowerSetup(void)
{
	// Configure RTC to wake up after 250ms 
	uint32_t _time = (((uint32_t)250) * RTC_cali) / 1000;
	HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, _time, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

	__HAL_RCC_PWR_CLK_ENABLE(); // Enable Power Control clock
	HAL_PWREx_EnableUltraLowPower(); // Ultra low power mode
	HAL_PWREx_EnableFastWakeUp(); // Fast wake-up for ultra low power mode
}


int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_ADC_Init();
  //AES_128_xcrypt(in,out1,16,AES128_key);
	HAL_UART_Receive_IT(&huart2, &lora_rxtemp, 1);
	
	HAL_GPIO_WritePin(GPIOA, NFC_VCC_Pin, GPIO_PIN_RESET);
	
	for (uint8_t i=0; i<8; i++) Delay_1000(); // 	NFC_VCC OFF i 5s sådan at ST25 er ude af FTM efter App download
	
	modem_wakeup();
	Delay_100();
//	reset_upctr();
	Delay_100();
//	Modem_save_conf();
	
	HAL_GPIO_WritePin(GPIOA, NFC_VCC_Pin, GPIO_PIN_SET);
	Delay_1000();	
	nfc_setup_GPO();
	Delay_1000();
	
	HAL_GPIO_WritePin(GPIOA, NFC_VCC_Pin, GPIO_PIN_RESET);
	Delay_1000();	
	
	HAL_GPIO_WritePin(GPIOA, NFC_VCC_Pin, GPIO_PIN_SET);
	Delay_1000();	
	
	read_NFC_loaded();
	if (NFC_loaded == 0) {write_NFC_factory_default(); Delay_1000();}	
	
	read_NFC(); 
	Delay_100();
	read_NFC_valve_cnt();
	Delay_100();
	read_NFC_calibration();
	Delay_100();
	read_NFC_supply_mode();
	Delay_100();
	read_NFC_data_rate();
	Delay_100();
	write_NFC_type_rev();
	Delay_100();
	write_NFC_no_FW_pending();
	Delay_100();
	write_TMP75_conf();
	Delay_100();
	
//	TMP75_sleep();
	
	operation_mode = 0;   // start-up mode
	pin_init();						// pull-ups skal være ON under start-up og test
	
	MX_RTC_Init();
	stm32L_lowPowerSetup();
	check_operation_mode();
	
	if (operation_mode == 1)  // hvis det er test_mode bliver modulet i denne if-sætning
	{
		test_inputs();
		calibrate_RTC();
		pin_deinit();	
		test_calibrate_psin();
//		test_onboard_pressure_sensor();
		test_calibrate_vbat();		
		Delay_1000();
		test_depassivation();
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
//		send_msg();
//		HAL_GPIO_WritePin(GPIOA, NFC_VCC_Pin, GPIO_PIN_RESET);
		test_completed = 1;
		while(1) HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	}		
	
	
//	stm32L_lowPowerSetup();
	
///		for (uint8_t i=0; i<5; i++)
///		{
///			HAL_GPIO_WritePin(BAT_L_GPIO_Port, BAT_L_Pin, GPIO_PIN_SET);  // depassivering
///			Delay_1000();
///			HAL_GPIO_WritePin(BAT_L_GPIO_Port, BAT_L_Pin, GPIO_PIN_RESET);
///			Delay_1000();
///		}	
	
	for (uint8_t i=0; i<5; i++) Delay_1000(); // base timer must run for a while to update status before sending data and allow NFC RF read	
	
	HAL_GPIO_WritePin(GPIOA, NFC_VCC_Pin, GPIO_PIN_RESET);
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);    // enabler NFC_busy interrupt
	
	Delay_1000();
	Delay_1000();
	
  stopped = 1;
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	stopped = 0;
	
	vbat_cnt = vbat_update_time - 4;   // sikrer at vbat bliver målt inden afsendelse
  if (pressure_upd_time > 10) pressure_cnt = pressure_upd_time - 10;  // sikrer at psin bliver målt inden afsendelse
	for (uint8_t i=0; i<20; i++) Delay_1000();
	
//	lora_printf(FACTORY_RESET);
	
//	while(1);
	
 
	for (uint8_t i=0; i<25; i++) Delay_1000(); 
	send_msg();
	for (uint8_t i=0; i<20; i++) Delay_1000();
	msg_interval_time_cnt = 120;
	
  MX_IWDG_Init();	
	
  while (1)
  {		
		if (msg_interval_time_cnt == 0)   // der kan alligevel ikke sendes ny message før den er på 0
		{
			if (msg_cnt > 0) {msg_cnt--; send_msg();}
			else
			{
				if ((main_time_cnt > heartbeat_time) && (msg_interval_time_cnt == 0)) {send_msg(); main_time_cnt = 0; msg_interval_time_cnt = 120;}
			}			
		}
		
		
		if ((psin_req == 0) && (vbat_req == 0))  // hvis der skal foretages analog måling skal STM32 ikke gå i stop
		{
			stopped = 1; 
		}
		stopped = 0;
		
		if (lora_rxlen != 0)    // der er noget i rx_bufferen i givet fald
		{	
			if ((lora_rxlen > 0) && (lora_rxlen < 19)) Delay_100();   // giver tid til at hele telegrammet er ankommet før parse
			parse_rxstring();
		}	
		
		HAL_IWDG_Refresh(&hiwdg);
//		check_uart();
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
  
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = ENABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = ENABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  HAL_ADC_Init(&hadc);
 
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
 
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);
  
	HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED);
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
//	#define IWDG_PRESCALER_4                0x00000000u                                     /*!< IWDG prescaler set to 4   */
//	#define IWDG_PRESCALER_8                IWDG_PR_PR_0                                    /*!< IWDG prescaler set to 8   */
//	#define IWDG_PRESCALER_16               IWDG_PR_PR_1                                    /*!< IWDG prescaler set to 16  */
//	#define IWDG_PRESCALER_32               (IWDG_PR_PR_1 | IWDG_PR_PR_0)                   /*!< IWDG prescaler set to 32  */
//	#define IWDG_PRESCALER_64               IWDG_PR_PR_2                                    /*!< IWDG prescaler set to 64  */
//	#define IWDG_PRESCALER_128              (IWDG_PR_PR_2 | IWDG_PR_PR_0)                   /*!< IWDG prescaler set to 128 */
//	#define IWDG_PRESCALER_256              (IWDG_PR_PR_2 | IWDG_PR_PR_1)                   /*!< IWDG prescaler set to 256 */
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;	

  hiwdg.Init.Window = 4095;				//should be same value with Reload Period
  hiwdg.Init.Reload = 4095;				//IWDG Reload period			
  HAL_IWDG_Init(&hiwdg);
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000000;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);
 
  /** Configure Analogue filter
  */
  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
 
  /** Configure Digital filter
  */
  HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 124;
  hrtc.Init.SynchPrediv = 295;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  HAL_RTC_Init(&hrtc);

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
 
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
 
  /** Enable the WakeUp
  */
  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 2314 / 4, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
 
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LORA_RST_Pin|BAT_L_Pin|OUT_3_Pin|NFC_VCC_Pin
                          |LED_2_Pin|LED_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BAT_EN_Pin|PS_EN_Pin|OUT_1_Pin|OUT_2_Pin, GPIO_PIN_RESET);
	
	  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PS1_EN_Pin|PS2_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_RST_Pin BAT_L_Pin OUT_3_Pin NFC_VCC_Pin
                           LED_2_Pin LED_1_Pin */
  GPIO_InitStruct.Pin = LORA_RST_Pin|BAT_L_Pin|OUT_3_Pin|NFC_VCC_Pin
                          |LED_2_Pin|LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_1_Pin IN_2_Pin IN_3_Pin TMP_ALERT_Pin */
  GPIO_InitStruct.Pin = IN_1_Pin|IN_2_Pin|IN_3_Pin|TMP_ALERT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BAT_EN_Pin PS_EN_Pin OUT_1_Pin OUT_2_Pin */
  GPIO_InitStruct.Pin = BAT_EN_Pin|PS_EN_Pin|OUT_1_Pin|OUT_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	 /*Configure GPIO pins : PS1_EN_Pin PS2_EN_Pin */
  GPIO_InitStruct.Pin = PS1_EN_Pin|PS2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : NFC_BUSY_Pin */
  GPIO_InitStruct.Pin = NFC_BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NFC_BUSY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

}


void pin_init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : IN_3_Pin IN_1_Pin IN_2_Pin */
  GPIO_InitStruct.Pin = IN_3_Pin|IN_1_Pin|IN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void pin_deinit(void)
{
  HAL_GPIO_DeInit(GPIOA, IN_3_Pin|IN_1_Pin|IN_2_Pin);
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin Specifies the pins connected to the EXTI line.
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		nfc_cnt=24;	//tæller ned til at slukke for NFC_VCC
	  HAL_GPIO_WritePin(GPIOA, NFC_VCC_Pin, GPIO_PIN_SET);
	  write_NFC();
		read_NFC_flag=1;
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  huart UART handle.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		lora_rxbuffer[lora_rxlen++] = lora_rxtemp;
		
		HAL_UART_Receive_IT(&huart2, &lora_rxtemp, 1);
	if(lora_rxlen >= LORA_MAX_LEN )
	{		
		lora_rxlen = 0;
	}
}
/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  UNUSED(hrtc);
	
//	HAL_GPIO_TogglePin(LED_1_GPIO_Port,  LED_1_Pin);
//	HAL_GPIO_TogglePin(LED_2_GPIO_Port,  LED_2_Pin);	
	
//	HAL_GPIO_WritePin(GPIOA, OUT_3_Pin, GPIO_PIN_SET);     // test af udgang 3
//	HAL_GPIO_WritePin(GPIOB, OUT_1_Pin, GPIO_PIN_SET);		 // test af udgang 1
//	HAL_GPIO_WritePin(GPIOB, OUT_2_Pin, GPIO_PIN_SET);		 // test af udgang 2
		
 
	if (operation_mode == 0)
	{
		if (test_cnt <= 6) test_cnt++;
		if (test_cnt == 2) input1_cnt_buffer = input1_cnt;
		input1_cnt = 0;
	}
	
	else if (operation_mode == 1) 
	{
		if (test_fail) HAL_GPIO_WritePin(GPIOB, PS_EN_Pin, GPIO_PIN_SET);
		else
		{	
			test_LED_cnt++;
			if (test_completed)
			{		
				if (test_LED_cnt > 12) {test_LED_cnt = 0; HAL_GPIO_WritePin(GPIOB, PS_EN_Pin, GPIO_PIN_SET); HAL_GPIO_WritePin(GPIOB, OUT_2_Pin, GPIO_PIN_SET);}
				else {HAL_GPIO_WritePin(GPIOB, PS_EN_Pin, GPIO_PIN_RESET); HAL_GPIO_WritePin(GPIOB, OUT_2_Pin, GPIO_PIN_RESET);}
			}
			else
			{
  				if (get_psin_flag) {HAL_GPIO_WritePin(GPIOB, PS_EN_Pin, GPIO_PIN_SET); get_psin(); get_psin_flag =0;}  // get_psin kaldes fra timer under test så det er på samme måde
					else HAL_GPIO_TogglePin(GPIOB, PS_EN_Pin);
					if (get_vbat_flag) {get_vbat(); get_vbat_flag = 0;}
					if (get_pressure_flag)
					{
						HAL_GPIO_WritePin(GPIOC, PS1_EN_Pin, GPIO_PIN_SET);
						get_psin();    // er udelukkende med for at skabe et delay der svarer til normal operation
						psin_powerup_delay_cnt = 0;
						read_AMS5915();
						pressure2 = AMS5915_pressure;
						HAL_GPIO_WritePin(GPIOC, PS1_EN_Pin, GPIO_PIN_RESET); 
						get_pressure_flag = 0;
					}
			}		
			if (test_cnt == 0) input1_cnt = 0;  // der ses bort fra første cnt_værdi da den ikke er fra fuld cyclus
			if (test_cnt == 32) input1_cnt_buffer = input1_cnt;
			if (test_cnt <= 36) test_cnt++;  // de 36 er bare for køre lidt videre fra de 32 inden der stoppes
		}
	}
	
	else
	{	
		if (nfc_cnt) nfc_cnt--; else HAL_GPIO_WritePin(GPIOA, NFC_VCC_Pin, GPIO_PIN_RESET);
		if (msg_interval_time_cnt) msg_interval_time_cnt--;
		main_time_cnt++;
		NFC_update_time_cnt++;
		passivation_cnt++;
		vbat_cnt++;
		pressure_cnt++;
		if ((pressure1_high) && (pressure1_alarm == 0)) pressure1_alarm_cnt++;
		if ((pressure2_high) && (pressure2_alarm == 0)) pressure2_alarm_cnt++;
		
		if ((pressure1_high == 0) && (pressure1_alarm == 1)) pressure1_not_alarm_cnt++;
		if ((pressure2_high == 0) && (pressure2_alarm == 1)) pressure2_not_alarm_cnt++;
	
		if (NFC_update_time_cnt > 14400) {nfc_cnt = 20; HAL_GPIO_WritePin(GPIOA, NFC_VCC_Pin, GPIO_PIN_SET); write_NFC(); NFC_update_time_cnt = 0;}  //update NFC every hour to save valve_operations_cnt
	
		if (passivation_cnt > 4838400)	// apply load when 14 days have passed
		{
			HAL_GPIO_WritePin(BAT_L_GPIO_Port, BAT_L_Pin, GPIO_PIN_SET);
			bat_l_cnt++;
			if (bat_l_cnt >= 5) {bat_l_cnt = 0; passivation_cnt=0; HAL_GPIO_WritePin(BAT_L_GPIO_Port, BAT_L_Pin, GPIO_PIN_RESET);}
		}
	
		if ((vbat_cnt > vbat_update_time) && (send_msg_active == 0))
		{
			vbat_req = 1;
			if ((send_msg_active == 0) && (stopped == 0))
			{	
				get_vbat(); 
				if (vbat < 4000) low_bat = 1; else low_bat = 0;
				vbat_cnt = 0;
				vbat_req = 0;
			}	
		}	
	
		if ((pressure_cnt > pressure_upd_time) && (send_msg_active == 0))
		{
			psin_req = 1;
			HAL_GPIO_WritePin(PS_EN_GPIO_Port, PS_EN_Pin, GPIO_PIN_SET);
			psin_powerup_delay_cnt++;
			if ((send_msg_active == 0) && (psin_powerup_delay_cnt >= 11) && (stopped == 0)) 
			{
				HAL_GPIO_WritePin(GPIOC, PS1_EN_Pin, GPIO_PIN_SET);
				get_psin();
				pressure1 = psin;
				psin_powerup_delay_cnt = 0;
				HAL_GPIO_WritePin(PS_EN_GPIO_Port, PS_EN_Pin, GPIO_PIN_RESET);
				psin_req = 0;
				read_AMS5915();
				pressure2 = AMS5915_pressure;
				HAL_GPIO_WritePin(GPIOC, PS1_EN_Pin, GPIO_PIN_RESET);
		
				if ((pressure1 < pressure1_limit) || (pressure1_limit == 0))
				{
					pressure1_high = 0; 
					pressure1_alarm_cnt = 0;
					if (pressure1_not_alarm_cnt > pressure_alarm_time)
					{
						if (pressure1_alarm) start_msg_seq();
						pressure1_alarm = 0;
					}
				}
				else 
				{
					pressure1_high = 1;
					pressure1_not_alarm_cnt = 0;
					if (pressure1_alarm_cnt > pressure_alarm_time)
					{
						if (pressure1_alarm == 0) start_msg_seq();
						pressure1_alarm = 1;
					}
				}	
				
				
				if ((pressure2 < pressure2_limit) || (pressure2_limit == 0))
				{
					pressure2_high = 0; 
					pressure2_alarm_cnt = 0;
					if (pressure2_not_alarm_cnt > pressure_alarm_time)
					{
						if (pressure2_alarm) start_msg_seq();
						pressure2_alarm = 0;
					}
				}
				else 
				{
					pressure2_high = 1;
					pressure2_not_alarm_cnt = 0;
					if (pressure2_alarm_cnt > pressure_alarm_time)
					{
						if (pressure2_alarm == 0) start_msg_seq();
						pressure2_alarm = 1;
					}
				}				
				
				if (pressure1_high || pressure2_high)
				{
					if (pressure_upd_time > 240) pressure_cnt = pressure_upd_time - 240; else pressure_cnt = 0;
				}
				else pressure_cnt = 0;
			}
		}		
		
		if (read_NFC_byte63_flag && HAL_GPIO_ReadPin(NFC_BUSY_GPIO_Port, NFC_BUSY_Pin))
		{
			read_NFC_byte63();
			if (NFC_read_OK) 
			{
				if (NFC_byte63 == 0x00) 
				{
						system_reset_cnt++;
						if (system_reset_cnt > 3)	NVIC_SystemReset();
				}	
				else
				{
					read_NFC_byte63_flag = 0;
					system_reset_cnt = 0;
				}
			}	
		}		

		if (read_NFC_flag && HAL_GPIO_ReadPin(NFC_BUSY_GPIO_Port, NFC_BUSY_Pin))
		{
			read_NFC();
			read_NFC_flag = 0;
			read_NFC_byte63_flag = 1;   // på denne måde læses byte63 først næste timer-int, dvs delay mellem reads opnået
		}
		
		pin_init();	  // switch on pullups on the 3 inputs
	
		valve_input = HAL_GPIO_ReadPin(IN_1_GPIO_Port, IN_1_Pin);
		if (reed_polarity) {if (valve_input) valve_input=0; else valve_input=1;}
	
		if (!(valve_open_filtered))
			{			
				if (valve_input) valve_filt_cnt++; else valve_filt_cnt=0;
				if (valve_filt_cnt==3) {valve_open_filtered = 1; valve_filt_cnt=0; valve_open_time_measure=0;
						valve_operations_cnt++; if (valve_operations_cnt==0) valve_operations_rollover++;} 
			}
		else 
			{			
				if (!(valve_input)) valve_filt_cnt++; else valve_filt_cnt=0;
				if (valve_filt_cnt==3) 
				{
					if (valve_alarm) start_msg_seq(); 
					valve_open_filtered = 0;
					valve_filt_cnt=0;
					valve_alarm = 0;
					save_valve_open_time(); 
				} 
				else
					{
						if (valve_open_time_measure < 65000) valve_open_time_measure++; // no reason to update time_measure if time>65000
						if (valve_open_time_measure > valve_warning_time) valve_warning = 1;
						if (valve_open_time_measure > valve_alarm_time) 
						{
							if (valve_alarm == 0) start_msg_seq(); 
							valve_alarm = 1;
						}
					}  
			}
		
		
		if (!(float_open_filtered))
			{			
				if (HAL_GPIO_ReadPin(IN_2_GPIO_Port, IN_2_Pin)) float_filt_cnt++; else float_filt_cnt=0;
				if (float_filt_cnt==3) {float_open_filtered = 1; float_filt_cnt=0; float_open_time_measure=0;} 
			}
		else
			{			
				if (!(HAL_GPIO_ReadPin(IN_2_GPIO_Port, IN_2_Pin))) float_filt_cnt++; else float_filt_cnt=0;
				if (float_filt_cnt==3) 
				{
					if (float_alarm) start_msg_seq(); 
					float_open_filtered = 0;
					float_filt_cnt=0;
					float_alarm = 0; 
				}
				else
					{
						if (float_open_time_measure < 65000) float_open_time_measure++; // no reason to update time_measure if time>65000
						if (float_open_time_measure > float_alarm_time) 
						{
							if (float_alarm == 0) start_msg_seq(); 
							float_alarm = 1;
						}
					}   	
			}
		
		if (!(input3_open_filtered))
			{			
				if (HAL_GPIO_ReadPin(IN_3_GPIO_Port, IN_3_Pin)) input3_filt_cnt++; else input3_filt_cnt=0;
				if (input3_filt_cnt==3) {input3_open_filtered = 1; input3_filt_cnt=0;} 
			}
		else
			{			
				if (!(HAL_GPIO_ReadPin(IN_3_GPIO_Port, IN_3_Pin))) input3_filt_cnt++; else input3_filt_cnt=0;
				if (input3_filt_cnt==3) {input3_open_filtered = 0; input3_filt_cnt=0;} 
			}	
		pin_deinit();	// switch off pullups on the 3 inputs		
		calc_valve_open_time_max();
		if (rx_command == 0x01)
		{
			rx_command = 0;
			if (output_on_time != 0) {HAL_GPIO_WritePin(GPIOB, OUT_2_Pin, GPIO_PIN_SET); output_cnt = output_on_time; block_rx_cnt = 140;}   // tidligere 240
		} 
		if (block_rx_cnt) block_rx_cnt--; 
		if (output_cnt) output_cnt--; else HAL_GPIO_WritePin(GPIOB, OUT_2_Pin, GPIO_PIN_RESET);
		if (HAL_GPIO_ReadPin(GPIOA, TMP_ALERT_Pin) == 0) HAL_GPIO_WritePin(GPIOB, OUT_2_Pin, GPIO_PIN_RESET);
	}
	

	
//	HAL_GPIO_TogglePin(LED_1_GPIO_Port,  LED_1_Pin);
//	HAL_GPIO_TogglePin(LED_2_GPIO_Port,  LED_2_Pin);
}

/**
  * @brief  Master Tx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
 
  UNUSED(hi2c);
}

/**
  * @brief  Master Rx Transfer completed callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  UNUSED(hi2c);
}

void save_valve_open_time(void)
{
  if (valve_open_time_measure < valve_alarm_time)	//if valve_open_time>alarm_limit not be saved because it is irrelevant in this case  
		{
			valve_open_time_buffer[valve_open_time_ptr] = valve_open_time_measure;
			valve_open_time_ptr++;
			if (valve_open_time_ptr == 20) valve_open_time_ptr = 0;
		}	
}



void calc_valve_open_time_max(void)			// result in s
{
  uint8_t n=0; 
	uint16_t temp=0;
	for(uint8_t i=0;i<20;i++)
		{
			if (valve_open_time_buffer[i] > temp) temp = valve_open_time_buffer[i];
		}	
	valve_open_time_max = temp/4;   // calculate in s
}



void modem_reset(void)
{
  HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);
	for(a=0;a<1000;a++);
	Delay_1000();
	HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
	for(a=0;a<1000;a++);
	Delay_1000();
}



void Modem_save_conf(void)
{
  lora_printf(SAVE_CONF);
	Delay_1000();
	Delay_1000();
	Delay_1000();
	Delay_1000();
}


void modem_init(void)
{
	lora_printf(GET_DEVEUI);
	Delay_100();
	lora_printf(GET_RX2);
	Delay_100();
	lora_printf(SET_SYNC);
	Delay_100();	
	lora_printf(SET_APPEUI);
	Delay_100();
	lora_printf(SET_NWKSKEY);
	Delay_100();
	lora_printf(SET_APPSKEY);
	Delay_100();
	lora_printf(RADIO_SET);
	Delay_100();
	if (data_rate == 1) lora_printf(SET_DATARATE_SF10);
	else lora_printf(SET_DATARATE_SF9);
	Delay_100();
	lora_printf(SET_POWERINDEX);
	Delay_100();
	lora_printf(SET_CLASS);
	Delay_100();
	lora_printf(SET_AR);
	Delay_100();
	lora_printf(SET_ADR_OFF);
	Delay_100();
	lora_printf(SET_RX2);
	Delay_100();
	lora_printf(RESET_DOWNLCNT);
	Delay_100();
	Delay_100();
}


void reset_upctr(void)
{
  lora_printf(SET_UPCTR);
	Delay_100();
}


void modem_wakeup(void)
{
  modem_reset();
	lora_autobaud();
	Delay_1000();
	Delay_1000();
	lora_printf(MAC_RESUME);
	Delay_1000();
	modem_init();
	Delay_1000();
	set_channels();
	Delay_1000();
}

void send_msg(void)
{
  uint8_t temp_8=0;
	uint16_t temp_16;
	uint32_t temp_32;
	send_msg_active = 1;
//	reinit_UART();
	modem_wakeup();
	temp_16 = device_addr;
	SET_DEVEUI[27] = hex[(temp_16>>12) & 0x000F];
	SET_DEVEUI[28] = hex[(temp_16>>8) & 0x000F];
	SET_DEVEUI[29] = hex[(temp_16>>4) & 0x000F];
	SET_DEVEUI[30] = hex[(temp_16) & 0x000F];
	lora_printf(SET_DEVEUI);
	Delay_100();
	SET_DEVADDR[20] = hex[(temp_16>>12) & 0x000F];
	SET_DEVADDR[21] = hex[(temp_16>>8) & 0x000F];
	SET_DEVADDR[22] = hex[(temp_16>>4) & 0x000F];
	SET_DEVADDR[23] = hex[(temp_16) & 0x000F];
	lora_printf(SET_DEVADDR);
	Delay_100();
//	Modem_save_conf();
	lora_printf(JOIN_ABP);
	Delay_1000();
	Delay_1000();

	if (float_alarm) temp_8 |= 0x01;
	if (valve_warning) temp_8 |= 0x02;
	if (valve_alarm) temp_8 |= 0x04;
	if (pressure1_alarm) temp_8 |= 0x08;
	if (pressure2_alarm) temp_8 |= 0x10;
	if (input3_open_filtered) temp_8 |= 0x20;
	if (low_bat) temp_8 |= 0x40;
	TX_UNCNF[15] = hex[(temp_8>>4) & 0x000F];
	TX_UNCNF[16] = hex[(temp_8) & 0x000F];
	temp_16 = pressure1;
	TX_UNCNF[19] = hex[(temp_16>>12) & 0x000F];
	TX_UNCNF[20] = hex[(temp_16>>8) & 0x000F];
	TX_UNCNF[17] = hex[(temp_16>>4) & 0x000F];
	TX_UNCNF[18] = hex[(temp_16) & 0x000F];
	temp_16 = pressure2;
	TX_UNCNF[23] = hex[(temp_16>>12) & 0x000F];
	TX_UNCNF[24] = hex[(temp_16>>8) & 0x000F];
	TX_UNCNF[21] = hex[(temp_16>>4) & 0x000F];
	TX_UNCNF[22] = hex[(temp_16) & 0x000F];
	temp_16 = valve_operations_cnt;
	temp_8 = valve_operations_rollover;
	TX_UNCNF[27] = hex[(temp_16>>12) & 0x000F];
	TX_UNCNF[28] = hex[(temp_16>>8) & 0x000F];
	TX_UNCNF[25] = hex[(temp_16>>4) & 0x000F];
	TX_UNCNF[26] = hex[(temp_16) & 0x000F];
	TX_UNCNF[29] = hex[(temp_8>>4) & 0x0F];
	TX_UNCNF[30] = hex[(temp_8) & 0x0F];
	temp_8 = valve_open_time_max;
	TX_UNCNF[31] = hex[(temp_8>>4) & 0x0F];
	TX_UNCNF[32] = hex[(temp_8) & 0x0F];
	temp_32 = vbat/10;
	if (temp_32 < 655)
	{	
		if (temp_32 > 400) temp_32 = temp_32 - 400;	else temp_32 = 0;
	}
	else temp_32 = 255;
	temp_8 = temp_32;
	TX_UNCNF[33] = hex[(temp_8>>4) & 0x0F];
	TX_UNCNF[34] = hex[(temp_8) & 0x0F];
	lora_printf(TX_UNCNF);
  Delay_1000();  
	Delay_1000();
	Delay_1000();
	Delay_1000();
	Delay_1000();
	lora_printf(SAVE_CONF);
	Delay_1000();
	Delay_1000();
	Delay_1000();
	Delay_1000();
	valve_warning = 0; // reset valve_warning which has now been sent (if set)
	send_msg_active = 0;
	if (msg_cnt != 8) msg_interval_time_cnt = 1200; else msg_interval_time_cnt = 100;     // hvis msg_cnt er 8 er der kommet en ny event mens msg blev sendt og der skal ikke være delay
	//	lora_rxlen = 0;
  	dummy1 = lora_rxlen;
  	lora_printf(GET_RX2);
  	Delay_1000();
  	dummy2 = lora_rxlen;
//    if (lora_rxlen == 0) NVIC_SystemReset();
	while (dummy1 == dummy2)
	{
		dummy3++;
		reinit_UART();
		Delay_1000();
		lora_autobaud();
		Delay_1000();
		Delay_1000();
		lora_printf(GET_RX2);
		Delay_1000();
		dummy2 = lora_rxlen;
	}
//	modem_wakeup();
	rx_init();
	lora_rxlen = 0;
	memset(lora_rxbuffer, 0, LORA_MAX_LEN);
}


void set_channels(void)    // Only channels 40-47 activated (sub-band 6)
{
  uint8_t i,j;
	uint8_t temp;
	
	for (i=0; i<10; i++) 
	{
		SET_CH_STATUS_SH[18] = hex[i];
		lora_printf(SET_CH_STATUS_SH);
		Delay_100();
	}
	
	for (i=1; i<4; i++) 
	{
		SET_CH_STATUS[18] = hex[i];
		for (j=0; j<10; j++) {SET_CH_STATUS[19] = hex[j]; lora_printf(SET_CH_STATUS); Delay_100();}
	}
	
	for (i=5; i<7; i++) 
	{
		SET_CH_STATUS[18] = hex[i];
		for (j=0; j<10; j++) {SET_CH_STATUS[19] = hex[j]; lora_printf(SET_CH_STATUS); Delay_100();}
	}
	
	for (i=0; i<8; i++) 
	{
		SET_CH_STATUS_ON[19] = hex[i];
		lora_printf(SET_CH_STATUS_ON); Delay_100();
	}
		
	lora_printf("mac set ch status 48 off\r\n");
	Delay_100();
	lora_printf("mac set ch status 49 off\r\n");
	Delay_100();
	lora_printf("mac set ch status 70 off\r\n");
	Delay_100();
	lora_printf("mac set ch status 71 off\r\n");
	Delay_100();	
}


void (write_NFC)(void)
{
	uint8_t buffer[8];
	buffer[0] = 0;
	if (low_bat) buffer[0] |= 0x01;
	if (output3_status) buffer[0] |= 0x02;
	if (output2_status) buffer[0] |= 0x04;
	if (output1_status) buffer[0] |= 0x08;
	if (input3_open_filtered) buffer[0] |= 0x10;
	if (valve_open_filtered) buffer[0] |= 0x40;
	if (float_open_filtered) buffer[0] |= 0x20;
	buffer[1] = valve_open_time_max;
	buffer[3] = pressure1 & 0x0FF;
	buffer[2] = (pressure1 >> 8) & 0x0FF;
	buffer[7] = pressure2 & 0x0FF;
	buffer[6] = (pressure2 >> 8) & 0x0FF;
	buffer[5] = valve_operations_cnt & 0x0FF;
	buffer[4] = (valve_operations_cnt >> 8) & 0x0FF;
	i2c_nfc_write(NFC_I2C_SLAVE_ADDR, 0x0008, buffer, 8);
	buffer[0] = valve_operations_rollover;
	i2c_nfc_write(NFC_I2C_SLAVE_ADDR, 0x0005, buffer, 1);
}

void (write_NFC_type_rev)(void)
{
	uint8_t buffer[4];
	buffer[0] = 4;
	buffer[1] = 0;
	buffer[2] = 1;
	buffer[3] = 0;
	i2c_nfc_write(NFC_I2C_SLAVE_ADDR, 0x0000, buffer, 4);
}

void (write_NFC_no_FW_pending)(void)
{
	uint8_t temp100;
	temp100 = 0;
	while (temp100 != 0x06)
	{	
		i2c_nfc_read(NFC_I2C_SLAVE_ADDR, 0x3F, nfc_rxbuffer, 1);
		temp100 = nfc_rxbuffer[0];
		if (temp100 != 0x06)
		{
			m_i2c_buffer[0] = 0x00;
	    m_i2c_buffer[1] = 0x3F;
	    m_i2c_buffer[2] = 0x06;
	    HAL_I2C_Master_Transmit(&hi2c1, 0xA6, m_i2c_buffer, 3, I2C_TIMEOUT);
			Delay_1000();
		}	
	}	
}


void (write_NFC_factory_default)(void)
{
	uint8_t buffer[16] = {0,1,0,1,60,60,15,60,0,0,0,0,0,120,0,60};
	i2c_nfc_write(NFC_I2C_SLAVE_ADDR, 0x10, buffer, 16);
	Delay_100();
	buffer[0] = 0x55;
	i2c_nfc_write(NFC_I2C_SLAVE_ADDR, 0x20, buffer, 1);				// write 0x55 to NFC addr 0x20 to trigger reset of counters (in read_NFC)
	Delay_100();
	i2c_nfc_write(NFC_I2C_SLAVE_ADDR, 0x30, buffer, 1);				// write 0x55 to NFC addr 0x30, code for NFC loaded
	Delay_100();
	buffer[0] = 0x11;
	buffer[1] = 0x94;
	i2c_nfc_write(NFC_I2C_SLAVE_ADDR, 0x34, buffer, 2);				// write default calibration for psin (4500)
	Delay_100();
	buffer[0] = 0x13;
	buffer[1] = 0x88;
	i2c_nfc_write(NFC_I2C_SLAVE_ADDR, 0x38, buffer, 2);				// write default calibration for vbat (5000)
	Delay_100();
	buffer[0] = 0x09;
	buffer[1] = 0x6D;
	i2c_nfc_write(NFC_I2C_SLAVE_ADDR, 0x3C, buffer, 2);				// write default calibration for RTC (2413)
	Delay_100();
}


void (read_NFC)(void)
{
	uint8_t buffer[8];
	HAL_NVIC_DisableIRQ(EXTI2_3_IRQn); // disable NFC_busy interrupt while reading
	i2c_nfc_read(NFC_I2C_SLAVE_ADDR, 0x10, nfc_rxbuffer, 16);
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn); 
	device_addr = (nfc_rxbuffer[0]<<8) & 0xFF00;
  device_addr |= nfc_rxbuffer[1];
	reed_polarity = nfc_rxbuffer[3];
  heartbeat_time = nfc_rxbuffer[4] * 240; // convert from minutes to 1/4s
	valve_alarm_time = nfc_rxbuffer[5] * 4;
	valve_warning_time = nfc_rxbuffer[6] * 4;
	float_alarm_time = nfc_rxbuffer[7] * 4;
	pressure1_limit = (nfc_rxbuffer[8]<<8) & 0xFF00;
  pressure1_limit |= nfc_rxbuffer[9];
	pressure2_limit = (nfc_rxbuffer[10]<<8) & 0xFF00;
  pressure2_limit |= nfc_rxbuffer[11];
	pressure_alarm_time = (nfc_rxbuffer[12]<<8) & 0xFF00;
  pressure_alarm_time |= nfc_rxbuffer[13];
	pressure_alarm_time = pressure_alarm_time * 4;
	pressure_upd_time = (nfc_rxbuffer[14]<<8) & 0xFF00;
  pressure_upd_time |= nfc_rxbuffer[15];
	pressure_upd_time = pressure_upd_time * 4;
	HAL_NVIC_DisableIRQ(EXTI2_3_IRQn); // disable NFC_busy interrupt while reading
	i2c_nfc_read(NFC_I2C_SLAVE_ADDR, 0x20, nfc_rxbuffer, 1);
	if (nfc_rxbuffer[0] == 0x55)
	{
		valve_operations_cnt = 0;
		valve_operations_rollover=0;
		buffer[0] = 0;
		buffer[1] = 0;
	  i2c_nfc_write(NFC_I2C_SLAVE_ADDR, 0x0020, buffer, 1);
		i2c_nfc_write(NFC_I2C_SLAVE_ADDR, 0x0005, buffer, 1);
		i2c_nfc_write(NFC_I2C_SLAVE_ADDR, 0x000C, buffer, 2);
	}	
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn); 
}

void (read_NFC_valve_cnt)(void)
{
	HAL_NVIC_DisableIRQ(EXTI2_3_IRQn); // disable NFC_busy interrupt while reading
	i2c_nfc_read(NFC_I2C_SLAVE_ADDR, 0x0C, nfc_rxbuffer, 2);
	valve_operations_cnt = (nfc_rxbuffer[0]<<8) & 0xFF00;
  valve_operations_cnt |= nfc_rxbuffer[1];
	i2c_nfc_read(NFC_I2C_SLAVE_ADDR, 0x05, nfc_rxbuffer, 1);
	valve_operations_rollover = nfc_rxbuffer[0];
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn); 
}

void (read_NFC_loaded)(void)
{
	HAL_NVIC_DisableIRQ(EXTI2_3_IRQn); // disable NFC_busy interrupt while reading
	i2c_nfc_read(NFC_I2C_SLAVE_ADDR, 0x30, nfc_rxbuffer, 1);
	if (nfc_rxbuffer[0] == 0x55) NFC_loaded = 1; else NFC_loaded = 0;
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn); 
}

void (read_NFC_byte63)(void)
{
	HAL_NVIC_DisableIRQ(EXTI2_3_IRQn); // disable NFC_busy interrupt while reading
	i2c_nfc_read(NFC_I2C_SLAVE_ADDR, 0x3F, nfc_rxbuffer, 1);
	NFC_byte63 = nfc_rxbuffer[0]; 
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn); 
}

void (read_NFC_supply_mode)(void)
{
	HAL_NVIC_DisableIRQ(EXTI2_3_IRQn); // disable NFC_busy interrupt while reading
	i2c_nfc_read(NFC_I2C_SLAVE_ADDR, 0x3E, nfc_rxbuffer, 1);
	supply_mode = nfc_rxbuffer[0]; 
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn); 
}

void (read_NFC_data_rate)(void)
{
	HAL_NVIC_DisableIRQ(EXTI2_3_IRQn); // disable NFC_busy interrupt while reading
	i2c_nfc_read(NFC_I2C_SLAVE_ADDR, 0x3B, nfc_rxbuffer, 1);
	data_rate = nfc_rxbuffer[0]; 
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn); 
}

void (read_NFC_calibration)(void)
{
	HAL_NVIC_DisableIRQ(EXTI2_3_IRQn); // disable NFC_busy interrupt while reading
	i2c_nfc_read(NFC_I2C_SLAVE_ADDR, 52, nfc_rxbuffer, 10);
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn); 
	
	psin_cali = (nfc_rxbuffer[0]<<8) & 0xFF00;
  psin_cali |= nfc_rxbuffer[1];
	
	vbat_cali = (nfc_rxbuffer[4]<<8) & 0xFF00;
  vbat_cali |= nfc_rxbuffer[5];
	
	RTC_cali = (nfc_rxbuffer[8]<<8) & 0xFF00;
  RTC_cali |= nfc_rxbuffer[9];
}

void (read_AMS5915)(void)
{
	nfc_rxbuffer[0] = 0;
	nfc_rxbuffer[1] = 0;
	HAL_NVIC_DisableIRQ(EXTI2_3_IRQn); // disable NFC_busy interrupt while reading
	HAL_I2C_Master_Receive(&hi2c1, AMS5915_I2C_SLAVE_ADDR, nfc_rxbuffer, 4, I2C_TIMEOUT);
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn); 
	
	AMS5915_pressure = 0;
	AMS5915_pressure = (nfc_rxbuffer[0]<<8) & 0x3F00;   // det er kun de 14 LSB's der skal anvendes
  AMS5915_pressure |= nfc_rxbuffer[1];
	AMS5915_pressure = AMS5915_pressure - 1638;
	AMS5915_pressure = AMS5915_pressure * 10000;
	AMS5915_pressure = AMS5915_pressure / 131070;
	if (AMS5915_pressure == 0x7F83) AMS5915_pressure = 0;     // Svarer til værdi man når frem til hvis der ikke er svar fra AMS5915
	AMS5915_pressure = AMS5915_pressure * 2953;
	AMS5915_pressure = AMS5915_pressure / 1000;
}

void (write_TMP75_conf)(void)
{
	uint8_t buffer[2];
	buffer[0] = 1;
	buffer[1] = 0;
	HAL_I2C_Master_Transmit(&hi2c1, TMP75_I2C_SLAVE_ADDR , buffer, 2, I2C_TIMEOUT);
}


void (TMP75_sleep)(void)
{
	HAL_NVIC_DisableIRQ(EXTI2_3_IRQn); // disable NFC_busy interrupt while reading

	m_i2c_buffer[0] = 0x01;
	m_i2c_buffer[1] = 0x61;

  HAL_I2C_Master_Transmit(&hi2c1, TMP75_I2C_SLAVE_ADDR , m_i2c_buffer, 2, I2C_TIMEOUT);
	
	HAL_NVIC_EnableIRQ(EXTI2_3_IRQn); 
}



void (Delay_100)(void)
{
	for(a=0;a<3666;a++);
}


void (Delay_1000)(void)
{
	for(a=0;a<20000;a++);
}


void (start_msg_seq)(void)
{
	msg_cnt = 8;
	msg_interval_time_cnt = 0;
}


void (get_vbat)(void)
{
	HAL_GPIO_WritePin(BAT_EN_GPIO_Port, BAT_EN_Pin, GPIO_PIN_SET);
	if (operation_mode == 2) HAL_GPIO_WritePin(BAT_L_GPIO_Port, BAT_L_Pin, GPIO_PIN_SET);  // load styres fra test-program under test
	
	
  uint32_t temp = 0;
	
	for (uint8_t i=0; i<50; i++);
	
	for (uint8_t i=0; i<5; i++)
	{
		temp = temp + Get_VoltageValue(VBAT_CHANNEL);
	}	 
	temp = temp * 59;
	temp = temp/60;  //pga spændingsdeler samt 10 målinger	v
	
	vbat = temp;
	
	if (operation_mode == 2)
	{	
		temp = temp * 4970;
		vbat = temp/vbat_cali;
	}
		
	HAL_GPIO_WritePin(BAT_L_GPIO_Port, BAT_L_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BAT_EN_GPIO_Port, BAT_EN_Pin, GPIO_PIN_RESET);
}

void (get_psin)(void)
{
  uint32_t temp = 0;
	for (uint8_t i=0; i<10; i++)
	{
		temp = temp + Get_VoltageValue(PS_IN_CHANNEL);
	}	
	if (supply_mode == 0)	{temp = temp * 80; temp = temp/470;}   // spændingsdeler ved 3,3v forsyning: 33k/47k 
	if (supply_mode == 1)	{temp = temp * 72; temp = temp/390;}	 // spændingsdeler ved 3,0v forsyning: 33k/39k
	
	psin = temp;
	
	if (operation_mode == 2)
	{	
		
		temp = temp * 4500;
		temp = temp/psin_cali;
		temp = temp * 6006;
		temp = temp/10000;
		if (temp < 2991) psin = 2991 - temp; else psin = 0;
	}	
}

void check_operation_mode(void)
{
	count_input1_pulses(4);	// der tælles pulser i 4 timer cyclus'er (timer håndterer save correct value for antal pulser i een cyclus)	
	if ((input1_cnt_buffer >= 10) && (input1_cnt_buffer <=22)) operation_mode = 1; else operation_mode = 2;
}


void test_inputs(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	uint8_t test_reg=0;
	
	GPIO_InitStruct.Pin = IN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	Delay_100();
	for(a=0;a<3666;a++)	{if (HAL_GPIO_ReadPin(IN_1_GPIO_Port, IN_1_Pin)) test_reg=1;}  //pullup is switched off, so it is error if input is high
	if (test_reg == 1) {test_fail = 1; while(1);}
	GPIO_InitStruct.Pin = IN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	Delay_100();
	GPIO_InitStruct.Pin = IN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	Delay_100();
	test_reg = 0;
	for(a=0;a<3666;a++)	{if (HAL_GPIO_ReadPin(IN_2_GPIO_Port, IN_2_Pin)) test_reg=1;}  //pullup is switched off, so it is error if input is high
	if (test_reg == 1) {test_fail = 1; while(1);}
	GPIO_InitStruct.Pin = IN_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	Delay_100();
	test_reg = 0;
	GPIO_InitStruct.Pin = IN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	Delay_100();
	for(a=0;a<3666;a++)	{if (HAL_GPIO_ReadPin(IN_3_GPIO_Port, IN_3_Pin)) test_reg=1;}  //pullup is switched off, so it is error if input is high
	if (test_reg == 1) {test_fail = 1; while(1);}
	GPIO_InitStruct.Pin = IN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	Delay_100();
	test_reg = 0;
	for(a=0;a<3666;a++)
	{
		if (HAL_GPIO_ReadPin(IN_1_GPIO_Port, IN_1_Pin)) test_reg |= 0x02; else test_reg |= 0x01;
		if (HAL_GPIO_ReadPin(IN_2_GPIO_Port, IN_2_Pin)) test_reg |= 0x08; else test_reg |= 0x04;
		if (HAL_GPIO_ReadPin(IN_3_GPIO_Port, IN_3_Pin)) test_reg |= 0x20; else test_reg |= 0x10;
	}	
	if (test_reg != 0x3F) {test_fail = 1; while(1);}
}  


void (calibrate_RTC)(void)
{
	uint32_t help_var = 0;
	uint8_t NFC_buffer[8];
	
	count_input1_pulses(34);
	help_var = RTC_cali * 512;
	help_var = help_var/input1_cnt_buffer;
	NFC_buffer[0] = (help_var >> 8) & 0x000000FF;
	NFC_buffer[1] = help_var & 0x000000FF;
	i2c_nfc_write(NFC_I2C_SLAVE_ADDR, 0x3C, NFC_buffer, 2);				// write default calibration for RTC
	Delay_100();
}

void (test_calibrate_psin)(void)
{
	uint32_t help_var = 0;
	uint8_t NFC_buffer[8];
	
	get_psin_flag = 1;
	Delay_1000();
	
	if ((psin > 4000) && (psin < 5000))
	{	
		help_var = psin;
		if (supply_mode == 0) {help_var = help_var * 9940;	help_var = help_var / 10000;}
		if (supply_mode == 1) {help_var = help_var * 10130; help_var = help_var / 10000;}		//tidligere 9932
		NFC_buffer[0] = help_var >> 8 & 0x000000FF;
		NFC_buffer[1] = help_var & 0x000000FF;
		i2c_nfc_write(NFC_I2C_SLAVE_ADDR, 0x34, NFC_buffer, 2);		
		Delay_100();
	}
	else {test_fail = 1; while(1);}
	
}


void (test_onboard_pressure_sensor)(void)
{
	get_pressure_flag = 1;
	Delay_1000();
	if (pressure2 == 0) {test_fail = 1; while(1);}
	
}

void (test_calibrate_vbat)(void)
{
	uint8_t NFC_buffer[8];
	
	get_vbat_flag = 1;
	Delay_1000();
	
	if ((vbat > 4500) && (vbat < 5500))
	{	
		NFC_buffer[0] = vbat >> 8 & 0x000000FF;
		NFC_buffer[1] = vbat & 0x000000FF;
		i2c_nfc_write(NFC_I2C_SLAVE_ADDR, 0x38, NFC_buffer, 2);		
		Delay_100();
	}
	else {test_fail = 1; while(1);}
}

void (test_depassivation)(void)
{
	uint32_t vbat_saved;
	
	Delay_100();
	get_vbat_flag = 1;
	Delay_1000();
	vbat_saved = vbat;
	HAL_GPIO_WritePin(BAT_L_GPIO_Port, BAT_L_Pin, GPIO_PIN_SET);  // hvis load virker skal vbat falde med mindst 0,2 V
	Delay_100();
	get_vbat_flag = 1;
	Delay_1000();
	
	if (vbat_saved > vbat)
	{	
		if ((vbat_saved - vbat) < 45) {test_fail = 1; while(1);}
	}
	else {test_fail = 1; while(1);}
}

void (count_input1_pulses)(uint8_t u)
{
  test_cnt=0;
	while (test_cnt != u)		// når test_cnt er nået op på u er korrekt count aflæst i timer-rutine
	{	
		if (input1_activated)
		{
			if (HAL_GPIO_ReadPin(IN_1_GPIO_Port, IN_1_Pin))
			{
				input1_activated = 0;
			}
		}
		else
		{
			if (!(HAL_GPIO_ReadPin(IN_1_GPIO_Port, IN_1_Pin)))
			{
				input1_cnt++;
				input1_activated = 1;
			}
		}
	}
}

void (check_uart)(void)
{
	lora_printf(GET_RX2);
	Delay_1000();
}

void reinit_UART(void)
 { 
	HAL_UART_DeInit(&huart2); 
	MX_USART2_UART_Init();
	HAL_UART_Receive_IT(&huart2, &lora_rxtemp, 1);
 }

void (parse_rxstring)(void)
{	  
	uint16_t temp_output_time=0, help_reg=0;	
	
	string_match = 1;   // går ud fra match indtil det modsatte er bevist
			for (uint8_t i=0; i<9; i++)
			{
				if (lora_rxbuffer[i] != RX_MESSAGE[i]) {if (i != 7) string_match = 0;}   // Plads 7 er port som kan ændre sig
			}
			if (string_match)     // Dvs. de første 9 karakterer stemmer
			{
				rx_hex_ok = 1;                     // går ud fra rx er på hex format indtil det modsatte er bevist
				for (uint8_t j=0; j<5; j++)
				{				
					switch(lora_rxbuffer[j+12])
					{
						case 0x30:
							rx_command_and_value[j] = 0x00;						
							break;
		
						case 0x31:
							rx_command_and_value[j] = 0x01;						
							break;
					
						case 0x32:
							rx_command_and_value[j] = 0x02;						
							break;
						case 0x33:
							rx_command_and_value[j] = 0x03;						
							break;
		
						case 0x34:
							rx_command_and_value[j] = 0x04;						
							break;
					
						case 0x35:
							rx_command_and_value[j] = 0x05;						
							break;
					
						case 0x36:
							rx_command_and_value[j] = 0x06;						
							break;
		
						case 0x37:
							rx_command_and_value[j] = 0x07;						
							break;
					
						case 0x38:
							rx_command_and_value[j] = 0x08;						
							break;
					
						case 0x39:
							rx_command_and_value[j] = 0x09;						
							break;
		
						case 0x41:
							rx_command_and_value[j] = 0x0A;						
							break;
					
						case 0x42:
							rx_command_and_value[j] = 0x0B;						
							break;
					
						case 0x43:
							rx_command_and_value[j] = 0x0C;						
							break;
					
						case 0x44:
							rx_command_and_value[j] = 0x0D;						
							break;
		
						case 0x45:
							rx_command_and_value[j] = 0x0E;						
							break;
					
						case 0x46:
							rx_command_and_value[j] = 0x0F;						
							break;
						
						default:
							rx_hex_ok = 0;	
				 }			  
			 }	
			 if ((rx_hex_ok) && (block_rx_cnt == 0)) 
			 {
				  help_reg = rx_command_and_value[3] & 0x000F;
				  help_reg = help_reg << 12;
				  temp_output_time |= help_reg;
				  help_reg = rx_command_and_value[4] & 0x000F;
				  help_reg = help_reg << 8;
					temp_output_time |= help_reg;
				  help_reg = rx_command_and_value[1] & 0x000F;
				  help_reg = help_reg << 4;
				  temp_output_time |= help_reg;
					help_reg = rx_command_and_value[2] & 0x000F;
					temp_output_time |= help_reg;
				  output_on_time = temp_output_time << 2;             // ganger med 4 ved at rykke to til venstre
				  rx_command = rx_command_and_value[0] & 0x0F;
			 }
   }
	 if ((rx_hex_ok) && (string_match)) rx_cnt++; else bad_cnt++;   // kun til test-formål
	 rx_init();
	 lora_rxlen = 0;    
	 memset(lora_rxbuffer, 0, LORA_MAX_LEN);		
 }			

void (rx_init)(void)
{
	lora_printf(DISABLE_RX_WATCHDOG);
	Delay_100();
	lora_printf(MAC_PAUSE);
	Delay_100();
	lora_printf(CONTINUOUS_RX);
	Delay_100();
}

/*AES-128-CTR encrypt/decrypt*/
void AES_128_xcrypt(uint8_t* in, uint8_t* out, uint16_t length, uint8_t* key)
{
	struct AES_ctx ctx;
    uint8_t iv[16]  = { 0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff };
    
    memcpy(out,in,length);
    
    AES_init_ctx_iv(&ctx, key, iv);  
    AES_CTR_xcrypt_buffer(&ctx, out, length); //encrypt 
    
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

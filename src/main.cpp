#include <Arduino.h>

#ifdef NRF52_SERIES
#include <Adafruit_TinyUSB.h>
#endif

#include <U8g2lib.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "SparkFunLIS3DH.h"
#include <SX126x-RAK4630.h> //http://librarymanager/All#SX126x
#include <SPI.h>
#include <bluefruit.h>
#include <BLEScanner.h>

// Define LoRa parameters
#define RF_FREQUENCY 868300000	// Hz
#define TX_OUTPUT_POWER 22		// dBm
#define LORA_BANDWIDTH 0		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8	// Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0	// Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 3000
#define TX_TIMEOUT_VALUE 3000

// Function declarations
void OnTxDone(void);
void OnTxTimeout(void);
void send(void);

static RadioEvents_t RadioEvents;
static uint8_t TxdBuffer[64];

#define MAX_CMD_LEN 64
#define MAX_PARAM_LEN 256
#define MAX_ARGV_SIZE 64

typedef struct
{
	char cmd[MAX_CMD_LEN + 1];
	char params[MAX_PARAM_LEN + 1];
	int argc;
	char argv[MAX_ARGV_SIZE][MAX_PARAM_LEN + 1];
} AT_Command;

static void oled(void *parameter);
static void gps(void *parameter);
static void acc(void *parameter);
static void lis3dh_read_data();
static void lora(void *parameter);
static void button(void *parameter);
static void ble(void *parameter);
static void battery(void *parameter);

void scan_callback(ble_gap_evt_adv_report_t *report);
bool getDeviceName(ble_gap_evt_adv_report_t *report, char *name, size_t nameSize);

void handleInterrupt(void);

void atCmd(void *parameter);
void process_serial_input(char c);
void process_AT_Command(const char *input);

void handle_at(const AT_Command *cmd);
void handle_gps(const AT_Command *cmd);
void handle_acc(const AT_Command *cmd);
void handle_scan(const AT_Command *cmd);
void handle_bat(const AT_Command *cmd);

#define PIN_VBAT WB_A0
uint32_t vbat_pin = PIN_VBAT;

#define VBAT_MV_PER_LSB (0.73242188F) // 3.0V ADC range and 12 - bit ADC resolution = 3000mV / 4096
#define VBAT_DIVIDER_COMP (1.73)      // Compensation factor for the VBAT divider, depend on the board

#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)


float readVBAT(void);


void startTone(const AT_Command *cmd);
void stopTone(const AT_Command *cmd);

long latitude = 0, longitude = 0, altitude = 0;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

SFE_UBLOX_GNSS g_myGNSS;

LIS3DH SensorTwo(I2C_MODE, 0x18);

// 定义互斥信号量
SemaphoreHandle_t xMutex;
SemaphoreHandle_t xAccSemaphore;
SemaphoreHandle_t xGpsSemaphore;
SemaphoreHandle_t xBinarySemaphore;
SemaphoreHandle_t xScanSemaphore;
SemaphoreHandle_t xBatSemaphore;

#define MAX_COUNT 5
// 定义计数信号量句柄
SemaphoreHandle_t xCountingSemaphore;

typedef void (*AT_Handler)(const AT_Command *);

typedef struct
{
	const char *cmd;
	AT_Handler handler;
	const char *help;
} AT_HandlerTable;

AT_HandlerTable handler_table[] = {

	{"AT", handle_at, "AT Test"},
	{"AT+GPS", handle_gps, "GPS"},
	{"AT+ACC", handle_acc, "ACC"},
	{"AT+LORASTART", startTone, "Lora start"},
	{"AT+LORASTOP", stopTone, "Lora stop"},
	{"AT+SCAN", handle_scan, "Ble scan"},
	{"AT+BAT", handle_bat, "Bat"},
};

void setup()
{

	pinMode(WB_IO2, OUTPUT);

	digitalWrite(WB_IO2, LOW);
	delay(100);
	digitalWrite(WB_IO2, HIGH);

	Serial.begin(115200);

	time_t serial_timeout = millis();
	// On nRF52840 the USB serial is not available immediately
	while (!Serial)
	{
		if ((millis() - serial_timeout) < 5000)
		{
			delay(100);
			digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
		}
		else
		{
			break;
		}
	}

	digitalWrite(PIN_LED1, HIGH);
	digitalWrite(PIN_LED2, HIGH);

	// 创建互斥信号量
	xMutex = xSemaphoreCreateMutex();

	xTaskCreate(
		oled,	  /* Task function. */
		"oled",	  /* String with name of task. */
		4 * 1024, /* Stack size in bytes. */
		NULL,	  /* Parameter passed as input of the task */
		0,		  /* Priority of the task. */
		NULL);	  /* Task handle. */

	xTaskCreate(
		gps,	  /* Task function. */
		"gps",	  /* String with name of task. */
		4 * 1024, /* Stack size in bytes. */
		NULL,	  /* Parameter passed as input of the task */
		0,		  /* Priority of the task. */
		NULL);	  /* Task handle. */

	xTaskCreate(
		acc,	  /* Task function. */
		"acc",	  /* String with name of task. */
		4 * 1024, /* Stack size in bytes. */
		NULL,	  /* Parameter passed as input of the task */
		0,		  /* Priority of the task. */
		NULL);	  /* Task handle. */

	xTaskCreate(
		atCmd,	  /* Task function. */
		"atCmd",  /* String with name of task. */
		4 * 1024, /* Stack size in bytes. */
		NULL,	  /* Parameter passed as input of the task */
		0,		  /* Priority of the task. */
		NULL);

	xTaskCreate(
		lora,	  /* Task function. */
		"lora",	  /* String with name of task. */
		4 * 1024, /* Stack size in bytes. */
		NULL,	  /* Parameter passed as input of the task */
		0,		  /* Priority of the task. */
		NULL);

	xTaskCreate(
		button,	  /* Task function. */
		"button", /* String with name of task. */
		4 * 1024, /* Stack size in bytes. */
		NULL,	  /* Parameter passed as input of the task */
		0,		  /* Priority of the task. */
		NULL);

	xTaskCreate(
		ble,	  /* Task function. */
		"ble",	  /* String with name of task. */
		4 * 1024, /* Stack size in bytes. */
		NULL,	  /* Parameter passed as input of the task */
		0,		  /* Priority of the task. */
		NULL);


	xTaskCreate(
		battery,	  /* Task function. */
		"battery",	  /* String with name of task. */
		4 * 1024, /* Stack size in bytes. */
		NULL,	  /* Parameter passed as input of the task */
		0,		  /* Priority of the task. */
		NULL);		
}

void loop()
{
	// Serial.println("rtos_delay\r\n");
	digitalWrite(PIN_LED1, !digitalRead(PIN_LED1));
	digitalWrite(PIN_LED2, !digitalRead(PIN_LED2));
	delay(100);
}

void oled(void *parameter)
{
	u8g2.begin();							 // 初始化
	u8g2.enableUTF8Print();					 // UTF8允许
	u8g2.clearBuffer();						 // 清除缓存，其实初始化里有清除，循环时一定要加上
	u8g2.clear();
	u8g2.setFont(u8g2_font_wqy12_t_gb2312b); // 选择中文gb2313b
	u8g2.setCursor(0, 12);					 // 缓存区定位
	u8g2.print("OLED TEST OK\r\n");			 // 指定缓存区需要打印的字符串
	u8g2.sendBuffer();						 // 将定位信息发送到缓冲区

	char data[32] = {0};

	while (1)
	{
		delay(1000);

		float vbat_mv = readVBAT();
		// Serial.print("BATTERY: ");
    	// Serial.print(vbat_mv);
    	// Serial.println(" mV");

		char g_dispText[256] = {0};
		snprintf(g_dispText, 255, "BATTERY %.3f V", vbat_mv / 1000);
		u8g2.setCursor(0, 60);					 // 缓存区定位
		u8g2.print(g_dispText);			 // 指定缓存区需要打印的字符串
		u8g2.sendBuffer();	

		delay(1000);	

		u8g2.setDrawColor(0); // 设置绘图颜色为背景色（0为黑色）
  		u8g2.drawBox(0, 48 , u8g2.getDisplayWidth(), 16); // 覆盖行的矩形区域（48是y坐标，16是行高）
  		u8g2.setDrawColor(1); // 恢复绘图颜色为前景色（1为白色）
  		u8g2.sendBuffer(); // 发送缓冲区内容到屏幕

	}
}

void gps(void *parameter)
{
	int baud[7] = {9600, 14400, 19200, 38400, 56000, 57600, 115200};

	size_t i ;

	for ( i = 0; i < sizeof(baud) / sizeof(int); i++)
	{
		Serial1.begin(baud[i]);
		while (!Serial1)
			; // Wait for user to open terminal
		if (g_myGNSS.begin(Serial1) == true)
		{
			Serial.printf("GNSS baund rate: %d \n", baud[i]); // GNSS baund rate
			break;
		}
		
		Serial1.end();
		delay(200);
	}
	

	if( i == 7)
	{
		vTaskDelete(NULL);
	}

	Serial.println("GNSS serial connected\n");

	g_myGNSS.setUART1Output(COM_TYPE_UBX); // Set the UART port to output UBX only
	g_myGNSS.setI2COutput(COM_TYPE_UBX);   // Set the I2C port to output UBX only (turn off NMEA noise)
	g_myGNSS.saveConfiguration();		   // Save the current settings to flash and BBR

	xGpsSemaphore = xSemaphoreCreateBinary();

	// xCountingSemaphore = xSemaphoreCreateCounting(MAX_COUNT, 0);

	// xSemaphoreGive(xGpsSemaphore);
	while (1)
	{
		xSemaphoreTake(xGpsSemaphore, portMAX_DELAY);

		for (int i = 0; i < MAX_COUNT; i++)
		{
			delay(1000);
			xSemaphoreTake(xMutex, portMAX_DELAY);

			latitude = g_myGNSS.getLatitude();
			Serial.print(F("GPS: "));
			Serial.print(F(" Lat: "));
			Serial.print(latitude);

			longitude = g_myGNSS.getLongitude();
			Serial.print(F(" Long: "));
			Serial.print(longitude);
			Serial.print(F(" (degrees * 10^-7)"));

			altitude = g_myGNSS.getAltitude();
			Serial.print(F(" Alt: "));
			Serial.print(altitude);
			Serial.print(F(" (mm)"));

			// long speed = g_myGNSS.getGroundSpeed();
			// Serial.print(F(" Speed: "));
			// Serial.print(speed);
			// Serial.print(F(" (mm/s)"));

			// long heading = g_myGNSS.getHeading();
			// Serial.print(F(" Heading: "));
			// Serial.print(heading);
			// Serial.print(F(" (degrees * 10^-5)"));

			byte SIV = g_myGNSS.getSIV();
			Serial.print(F(" SIV: "));
			Serial.println(SIV);
			xSemaphoreGive(xMutex);

			if (latitude != 0 && longitude != 0)
			{
				u8g2.setCursor(0, 48);		   // 缓存区定位
				u8g2.print("GPS TEST OK\r\n"); // 指定缓存区需要打印的字符串
				u8g2.sendBuffer();
				Serial.print("GPS TEST OK\r\n");
				//vTaskSuspend(NULL);
				//break;
			}
		}
	}
}

void acc(void *parameter)
{
	if (SensorTwo.begin() != 0)
	{
		Serial.println("Problem starting the sensor at 0x18.");
	}
	else
	{
		Serial.println("Sensor at 0x18 started.");
		// Set low power mode
		uint8_t data_to_write = 0;
		SensorTwo.readRegister(&data_to_write, LIS3DH_CTRL_REG1);
		data_to_write |= 0x08;
		SensorTwo.writeRegister(LIS3DH_CTRL_REG1, data_to_write);
		delay(100);

		data_to_write = 0;
		SensorTwo.readRegister(&data_to_write, 0x1E);
		data_to_write |= 0x90;
		SensorTwo.writeRegister(0x1E, data_to_write);
		delay(100);
	}

	xAccSemaphore = xSemaphoreCreateBinary();

	while (1)
	{
		xSemaphoreTake(xAccSemaphore, portMAX_DELAY);
		lis3dh_read_data();
		// delay(1000);
	}
}

void lis3dh_read_data()
{
	// read the sensor value
	xSemaphoreTake(xMutex, portMAX_DELAY);

	Serial.print(F("ACC: "));
	Serial.print(" X(g) = ");
	Serial.print(SensorTwo.readFloatAccelX(), 4);
	Serial.print(" Y(g) = ");
	Serial.print(SensorTwo.readFloatAccelY(), 4);
	Serial.print(" Z(g)= ");
	Serial.println(SensorTwo.readFloatAccelZ(), 4);

	if (SensorTwo.readFloatAccelX() != 0 || SensorTwo.readFloatAccelY() != 0 || SensorTwo.readFloatAccelZ() != 0)
	{
		u8g2.setCursor(0, 24);		   // 缓存区定位
		u8g2.print("ACC TEST OK\r\n"); // 指定缓存区需要打印的字符串
		u8g2.sendBuffer();
		Serial.print("ACC TEST OK\r\n");
	}

	xSemaphoreGive(xMutex);
}

void atCmd(void *parameter)
{
	while (1)
	{
		if (Serial.available() > 0)
		{
			// 读取一个字节的数据
			char received = Serial.read();

			// 将接收到的字节写回串口
			Serial.write(received);
			process_serial_input(received);
		}
	}
}

void get_all_commands()
{
	Serial.print("Available AT commands:\r\n");
	int num_handlers = sizeof(handler_table) / sizeof(handler_table[0]);
	for (int i = 0; i < num_handlers; i++)
	{
		Serial.printf("%s - %s\r\n", handler_table[i].cmd, handler_table[i].help);
	}
}

void process_serial_input(char c)
{
	static char input[MAX_CMD_LEN + MAX_PARAM_LEN + 3];
	static int i = 0;

	/* backspace */
	if (c == '\b')
	{
		if (i > 0)
		{
			i--;
			Serial.print(" \b");
		}
		return;
	}

	if (i >= MAX_CMD_LEN + MAX_PARAM_LEN + 2)
	{

		i = 0;
		Serial.print("ERROR: Input buffer overflow\r\n");
		return;
	}

	if (c == '\n' || c == '\r')
	{
		input[i] = '\0';
		if (strcasecmp(input, "AT?") == 0 || strcasecmp(input, "AT+HELP") == 0)
		{
			get_all_commands();
		}
		else
		{
			process_AT_Command(input);
		}
		i = 0;
	}
	else
	{
		input[i] = c;
		i++;
	}
}

AT_Command parse_AT_Command(const char *input)
{
	AT_Command cmd;
	const char *eq_pos = strchr(input, '=');
	if (eq_pos != NULL)
	{
		size_t cmd_len = eq_pos - input;
		/* Gets the length of the argument */
		size_t params_len = strlen(eq_pos + 1);
		memcpy(cmd.cmd, input, cmd_len);
		memcpy(cmd.params, eq_pos + 1, params_len);
		cmd.cmd[cmd_len] = '\0';
		cmd.params[params_len] = '\0';
	}
	else
	{
		/* Without the = sign, the whole string is copied */
		strcpy(cmd.cmd, input);
		cmd.params[0] = '\0';
	}

	return cmd;
}

void process_AT_Command(const char *input)
{
	AT_Command cmd = parse_AT_Command(input);

	//  start fix Hit Enter to return AT_ERROR
	if (strlen(cmd.cmd) == 0) // When the length of the AT command is 0, no processing is performed.
	{
		Serial.print("\r\n");
		return;
	}
	//  end fix Hit Enter to return AT_ERROR

	int num_handlers = sizeof(handler_table) / sizeof(handler_table[0]);
	for (int i = 0; i < num_handlers; i++)
	{
		if (strcasecmp(cmd.cmd, handler_table[i].cmd) == 0)
		{
			Serial.print("\r\n"); // Add \r\n before returning the result of the AT command.
			handler_table[i].handler(&cmd);
			return;
		}
	}
	Serial.print("AT_ERROR\r\n");
}

void handle_at(const AT_Command *cmd)
{
	Serial.print("AT\r\n");
	Serial.print("OK\r\n");
}

/************************************************************************************/

static void lora(void *parameter)
{
	lora_rak4630_init();
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = NULL;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = NULL;
	RadioEvents.RxError = NULL;
	RadioEvents.CadDone = NULL;

	// Initialize the Radio
	Radio.Init(&RadioEvents);

	// Set Radio channel
	Radio.SetChannel(RF_FREQUENCY);

	// Set Radio TX configuration
	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
					  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
					  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
					  true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

	while (1)
	{
		delay(1000);
	}
}

/**@brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void)
{
	xSemaphoreTake(xMutex, portMAX_DELAY);
	Serial.println("LORA: OnTxDone");
	xSemaphoreGive(xMutex);
}

/**@brief Function to be executed on Radio Tx Timeout event
 */
void OnTxTimeout(void)
{
	Serial.println("OnTxTimeout");
}

void send()
{
	TxdBuffer[0] = 'H';
	TxdBuffer[1] = 'e';
	TxdBuffer[2] = 'l';
	TxdBuffer[3] = 'l';
	TxdBuffer[4] = 'o';
	taskENTER_CRITICAL();
	Radio.Send(TxdBuffer, 5);
	taskEXIT_CRITICAL();
}

void startTone(const AT_Command *cmd)
{
	Serial.printf("OK\r\n");
	Radio.SetTxContinuousWave(RF_FREQUENCY, TX_OUTPUT_POWER, 0xFFFF);
}

void stopTone(const AT_Command *cmd)
{
	Serial.printf("OK\r\n");
	Radio.Sleep();
}

static void button(void *parameter)
{
	// 配置数字引脚2为输入模式，并启用内部上拉电阻
	pinMode(WB_IO5, INPUT_PULLUP);

	// 配置外部中断
	attachInterrupt(WB_IO5, handleInterrupt, FALLING);

	xBinarySemaphore = xSemaphoreCreateBinary();

	while (1)
	{
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
		Serial.println("BUTTON TEST OK");

		u8g2.setCursor(0, 36);			  // 缓存区定位
		u8g2.print("BUTTON TEST OK\r\n"); // 指定缓存区需要打印的字符串
		u8g2.sendBuffer();
	}
}

void handleInterrupt()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// 从ISR中发出信号量
	xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);

	// 如果需要，执行上下文切换
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void handle_gps(const AT_Command *cmd)
{
	Serial.printf("OK\r\n");
	xSemaphoreGive(xGpsSemaphore);
}

void handle_acc(const AT_Command *cmd)
{
	Serial.printf("OK\r\n");
	xSemaphoreGive(xAccSemaphore);
}


BLEUuid testuuid1(0x1234);
BLEUuid testuuid2(0x3802);
TimerHandle_t xScanTimer;
void scanTimeoutCallback(TimerHandle_t xTimer);

static void ble(void *parameter)
{
	// Initialize Bluefruit with maximum connections as Peripheral = 0, Central = 1
	// SRAM usage required by SoftDevice will increase dramatically with number of connections
	Bluefruit.begin(0, 1);
	Bluefruit.setTxPower(4); // Check bluefruit.h for supported values
	Bluefruit.setName("Bluefruit52");

	// Start Central Scan
	// Bluefruit.setConnLedInterval(250);

	Bluefruit.Scanner.setRxCallback(scan_callback);
	Bluefruit.Scanner.filterUuid(testuuid1,testuuid2);

	xScanSemaphore = xSemaphoreCreateBinary();
	xScanTimer = xTimerCreate("ScanTimer", pdMS_TO_TICKS(2000), pdFALSE, (void *)0, scanTimeoutCallback);

	while (1)
	{
		xSemaphoreTake(xScanSemaphore, portMAX_DELAY);
		Bluefruit.Scanner.start(0);
		xTimerStart(xScanTimer, 0);
		// delay(1000);
	}
}

void scan_callback(ble_gap_evt_adv_report_t *report)
{
	xTimerStop(xScanTimer, 0);

	// Serial.println("Timestamp Addr              Rssi Data");

	// Serial.printf("%09d ", millis());

	// // MAC is in little endian --> print reverse
	// Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
	// Serial.print(" ");
	

	// Serial.printBuffer(report->data.p_data, report->data.len, '-');
	// Serial.println();

	// Extract and print the device name if present
	Serial.printf("rssi %d ",report->rssi);

	char deviceName[32] = {0}; // Buffer to hold the device name
	if (getDeviceName(report, deviceName, sizeof(deviceName)))
	{
		Serial.print("Device Name: ");
		Serial.println(deviceName);
	}

	// Serial.println();
	// Bluefruit.Scanner.resume();
	// xSemaphoreGive(xScanSemaphore);
	Bluefruit.Scanner.stop();	
	// For Softdevice v6: after received a report, scanner will be paused
	// We need to call Scanner resume() to continue scanning
	// Bluefruit.Scanner.resume();
}

void scanTimeoutCallback(TimerHandle_t xTimer)
{
    Serial.println("BLE Test failed");
	Bluefruit.Scanner.stop();
    // Restart the scan timer
    // xTimerStart(xScanTimer, 0);
}


void handle_scan(const AT_Command *cmd)
{
	Serial.printf("OK\r\n");
	xSemaphoreGive(xScanSemaphore);
}


void handle_bat(const AT_Command *cmd)
{
	Serial.printf("OK\r\n");
	xSemaphoreGive(xBatSemaphore);
}

bool getDeviceName(ble_gap_evt_adv_report_t *report, char *name, size_t nameSize)
{
	uint8_t len = report->data.len;
	uint8_t *data = report->data.p_data;

	while (len > 0)
	{
		uint8_t fieldLength = data[0];
		if (fieldLength == 0)
			break;
		uint8_t fieldType = data[1];

		if (fieldType == BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME || fieldType == BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME)
		{
			uint8_t nameLength = fieldLength - 1;
			if (nameLength > nameSize - 1)
			{
				nameLength = nameSize - 1;
			}
			memcpy(name, &data[2], nameLength);
			name[nameLength] = '\0'; // Null-terminate the string
			return true;
		}

		len -= fieldLength + 1;
		data += fieldLength + 1;
	}
	return false;
}






static void battery(void *parameter)
{
	// Set the analog reference to 3.0V (default = 3.6V)
    analogReference(AR_INTERNAL_3_0);
	// Set the resolution to 12-bit (0..4095)
    analogReadResolution(12); // Can be 8, 10, 12 or 14

    // Let the ADC settle
    delay(1);

	xBatSemaphore = xSemaphoreCreateBinary();

	while(1)
	{
		xSemaphoreTake(xBatSemaphore, portMAX_DELAY);
		// Get a raw ADC reading
    	float vbat_mv = readVBAT();
		Serial.print("BATTERY: ");
    	Serial.print(vbat_mv);
    	Serial.println(" mV");
		delay(1000);
	}
}



float readVBAT(void)
{
    float raw;

    // Get the raw 12-bit, 0..3000mV ADC value
    raw = analogRead(vbat_pin);

    return raw * REAL_VBAT_MV_PER_LSB;
}



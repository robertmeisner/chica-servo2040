#pragma once

#include <stdio.h>
#include <cstring>
#include "pico/stdlib.h"
#include "servo2040.hpp"
#include "analogmux.hpp"
#include "analog.hpp"
#include "button.hpp"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Used to log on a serial terminal, not Chica compatible
 * Enabled (Do not comment out)
 * Disabled (Comment out)													*/
//#define SERIAL_LOGGING

/* LED bar can be used to debug, but will interfere with serial flow
 * Enabled (Do not comment out)
 * Disabled (Comment out)													*/
#define LED_DEBUGGING

/* Commands */
#define SET_CMD	0xD3 // 0x53 & 0x80
#define GET_CMD	0xC7 // 0x47 & 0x80
	
/* Constraints */
#define MAX_PULSE_VALUE 2500
#define MIN_PULSE_VALUE	500

/* Short-name defines */
#define SERVO1		servo::servo2040::SERVO_1
#define SERVO2		servo::servo2040::SERVO_2
#define SERVO3		servo::servo2040::SERVO_3
#define SERVO4		servo::servo2040::SERVO_4
#define SERVO5		servo::servo2040::SERVO_5
#define SERVO6		servo::servo2040::SERVO_6
#define SERVO7		servo::servo2040::SERVO_7
#define SERVO8		servo::servo2040::SERVO_8
#define SERVO9		servo::servo2040::SERVO_9
#define SERVO10		servo::servo2040::SERVO_10
#define SERVO11		servo::servo2040::SERVO_11
#define SERVO12		servo::servo2040::SERVO_12
#define SERVO13		servo::servo2040::SERVO_13
#define SERVO14		servo::servo2040::SERVO_14
#define SERVO15		servo::servo2040::SERVO_15
#define SERVO16		servo::servo2040::SERVO_16
#define SERVO17		servo::servo2040::SERVO_17
#define SERVO18		servo::servo2040::SERVO_18

/* A0/A1/A2 Mapping */
#define A0_GPIO_PIN			26
#define A1_GPIO_PIN			27
#define A2_GPIO_PIN			28	
#define A0_GPIO_MASK		(1<<A0_GPIO_PIN)
#define A1_GPIO_MASK		(1<<A1_GPIO_PIN)
#define A3_GPIO_MASK		(1<<A2_GPIO_PIN)
#define GPIO_OUTPUT_MASK	0xFFFFFFFF
#define GPIO_INPUT_MASK		0x00
#define GPIO_HIGH_MASK		0xFFFFFFFF
#define GPIO_LOW_MASK		0x00

/* Miscellaneous */
#define MAX_COUNT_VALUE		127
#define MAX_DATA_VALUE		MAX_COUNT_VALUE
#define CMDPKT_BUFF_SIZE	50

/*******************************************************************************
 * Constants
 ******************************************************************************/
/* ISR */
constexpr uint TIMER_PERIOD_MS	= 100;

/* LED */
constexpr float BRIGHTNESS		= 0.3f;		// Normalized
constexpr float RED_HUE			= 0.0f;		// From hsv model, normalized
constexpr float GREEN_HUE		= 0.375f;	// From hsv model, normlaized

/* Volage/Current */
constexpr float MAX_VOLTAGE		= 8.2f; // Actual is 8.4V
constexpr float NOMINAL_VOLTAGE	= 7.4f;
constexpr float MIN_VOLTAGE		= 6.8f; // Actual is 6.2V
constexpr float MAX_CURRENT		= 10.0f;
constexpr float OVERCURRENT		= 8.5f;
constexpr uint NUM_SAMPLES		= 10;

/* Ratios */
constexpr float b1024_3_3V_RATIO	= 310.3f;
constexpr float b1024_5V_RATIO		= 204.8f;
constexpr float curr_LSb			= 0.0814f;

/* Lookup table */
constexpr uint RP_hardwarePins_table[] = 
{
	SERVO16,	SERVO17,	SERVO18,		// L11, L12, L13
	SERVO10,	SERVO11,	SERVO12,		// L21, L22, L23
	SERVO4,		SERVO5,		SERVO6, 		// L31, L32, L33
	SERVO13,	SERVO14,	SERVO15,		// R11, R12, R13
	SERVO7,		SERVO8,		SERVO9, 		// R21, R22, R23
	SERVO1,		SERVO2,		SERVO3,			// R31, R32, R33
	servo::servo2040::SENSOR_1_ADDR,		// TS_L1
	servo::servo2040::SENSOR_2_ADDR,		// TS_L2
	servo::servo2040::SENSOR_3_ADDR,		// TS_L3
	servo::servo2040::SENSOR_4_ADDR,		// TS_R1
	servo::servo2040::SENSOR_5_ADDR,		// TS_R2
	servo::servo2040::SENSOR_6_ADDR,		// TS_R3
	servo::servo2040::CURRENT_SENSE_ADDR,	// CURR
	servo::servo2040::VOLTAGE_SENSE_ADDR,	// VOL
	A0_GPIO_PIN,							// RELAY
	A1_GPIO_PIN,							// A1
	A2_GPIO_PIN								// A2
};

/*******************************************************************************
 * Enumerations
 ******************************************************************************/
typedef enum {
	L11, L12, L13, L21, L22, L23, L31, L32, L33,
	R11, R12, R13, R21, R22, R23, R31, R32, R33,
	TS_L1, TS_L2, TS_L3, TS_R1, TS_R2, TS_R3, 
	CURR, VOLT, RELAY, A1, A2, cmdPin_num
} cmdPins;

typedef enum {
	current,
	battSOC,
	touchSensing
} ledMapping;

typedef enum {
	none = 0,
	cmd_unrecognized, 
	startIdx_max_exceeded,
	countIdx_max_exceeded, 
	cmdPins_range_exceeded, 
	setCmd_cmdPin_incompatible,
	getCmd_cmdPin_incompatible, 
	overcurrent_detected
} errorCode;

typedef enum {
	set,
	get
} hexapodCmds;

/*******************************************************************************
 * Structures
 ******************************************************************************/
typedef struct {
	float voltage;
	float current;
	float touchSens[servo::servo2040::NUM_SENSORS];
} sensorReadings;

typedef struct {
	hexapodCmds cmd;
	uint startIdx;
	uint count;
	uint valueBuff[MAX_COUNT_VALUE];
} cmdPkt;

typedef struct {
	cmdPkt *buffer;
	uint head;
	uint tail;
	uint max;
	uint full;
} circular_buf_cmdPkt;

/* Handle type, the way users interact with the API */
typedef circular_buf_cmdPkt *cbuf_handle_cmdPkt;

typedef struct {
	ledMapping led_mapType;
	sensorReadings sysReadings;
	bool servoEnabled;
	volatile bool run_periodicTasks;
} sys_operData;


/*******************************************************************************
 * Function Forward Declarations
 ******************************************************************************/
/*******************************************************************************
 * ISR Callbacks
 ******************************************************************************/
void gpio_callback(uint gpio, uint32_t events);
bool periodicTimer_callback(struct repeating_timer *t);

/*******************************************************************************
 * Core Functions
 ******************************************************************************/
void serialStream_task(
cbuf_handle_cmdPkt cmd_cBuff
);

void run_hexapodCmds_task(
sys_operData *operData,
cbuf_handle_cmdPkt cmd_cBuff
);

void periodic_tasks(
sys_operData *operData
);

void protection_tasks(
sys_operData *operData
);

/*******************************************************************************
 * VCP/Parsing Support Functions
 ******************************************************************************/
uint cmdPin_to_hardwarePin(
cmdPins cmdPin
);

void vcp_transmit(
uint *txbuff,
uint size
);

/*******************************************************************************
 * LED Support Functions
 ******************************************************************************/
void pendingVCP_ledSequence(
void
);

void currMeasure_ledMapping(
float measuredCurrent
);

void batterySOC_ledMapping(
float measuredVolatage
);

void touchSensing_ledMapping(
float *touchSens
);

void errorCode_ledMapping(
errorCode error
);

/*******************************************************************************
 * Sensing Support Functions
 ******************************************************************************/
float read_current(
void
);

float read_voltage(
void
);

void read_analogPins(
float *touchSens
);

/*******************************************************************************
 * Circular Buffer
 ******************************************************************************/
void circular_buf_put(cbuf_handle_cmdPkt cbuf, cmdPkt *data);
void advance_pointer(cbuf_handle_cmdPkt cbuf);
int  circular_buf_get(cbuf_handle_cmdPkt cbuf, cmdPkt *data);
void retreat_pointer(cbuf_handle_cmdPkt cbuf);
uint circular_buf_empty(cbuf_handle_cmdPkt cbuf);
void circular_buf_reset(cbuf_handle_cmdPkt cbuf);
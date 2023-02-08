/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "main.h"
using namespace plasma;
using namespace servo;

/////////////* Global Variables */////////////
struct repeating_timer periodic_timer;

/* Create an array of servo pointers */
const int START_PIN = servo2040::SERVO_1; 	
const int END_PIN = servo2040::SERVO_18;	
const int NUM_SERVOS = (END_PIN - START_PIN) + 1;
ServoCluster servos = ServoCluster(pio0, 0, START_PIN, NUM_SERVOS);

/* Set up the shared analog inputs */
Analog sen_adc = Analog(servo2040::SHARED_ADC);
Analog vol_adc = Analog(servo2040::SHARED_ADC, servo2040::VOLTAGE_GAIN);
Analog cur_adc = Analog(servo2040::SHARED_ADC, servo2040::CURRENT_GAIN,
                        servo2040::SHUNT_RESISTOR, servo2040::CURRENT_OFFSET);

/* Set up the analog multiplexer, including the pin for controlling pull-up/pull-down */
AnalogMux mux = AnalogMux(servo2040::ADC_ADDR_0, servo2040::ADC_ADDR_1, servo2040::ADC_ADDR_2,
                          PIN_UNUSED, servo2040::SHARED_ADC);

/* Create the LED bar, using PIO 1 and State Machine 0 */
WS2812 led_bar(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);

/* Create the user button */
Button user_sw(servo2040::USER_SW);

sys_operData operData = { current, {0}, false, false };
cmdPkt cmdBuff[CMDPKT_BUFF_SIZE];
circular_buf_cmdPkt cmd_cBuff = { cmdBuff, 0, 0, CMDPKT_BUFF_SIZE, 0 };

int main() 
{
/*******************************************************************************
 * Initializations
 ******************************************************************************/
	/* Initialize the servo cluster */
	servos.init();
	
	/* Initialize analog inputs with pull downs */
	for (auto i = 0u; i < servo2040::NUM_SENSORS; i++) {
		mux.configure_pulls(servo2040::SENSOR_1_ADDR + i, false, true);
	}
	
	/* Initialize A0,A1,A2 */
	gpio_init_mask(A0_GPIO_MASK|A1_GPIO_MASK|A3_GPIO_MASK);		
	gpio_set_dir_masked(A0_GPIO_MASK|A1_GPIO_MASK|A3_GPIO_MASK, 
						GPIO_OUTPUT_MASK);	// Set output
	gpio_put_masked(A0_GPIO_MASK|A1_GPIO_MASK|A3_GPIO_MASK,		
					GPIO_LOW_MASK);			// Set LOW
	
	stdio_init_all();
	/* Wait for VCP/CDC connection */
	led_bar.start();
	while (!stdio_usb_connected()){pendingVCP_ledSequence();}
	led_bar.clear();
	
	/* Enable user button ISR */
	gpio_set_irq_enabled_with_callback(servo2040::USER_SW, GPIO_IRQ_EDGE_FALL, 
									   true, &gpio_callback);
	/* Enable periodic timer */
	add_repeating_timer_ms(TIMER_PERIOD_MS, periodicTimer_callback, NULL, &periodic_timer);

/*******************************************************************************
 * Application
 ******************************************************************************/
	while (1)
	{
		/* Monitor and parse serial data */
		serialStream_task(&cmd_cBuff);
		
		/* Run cmds given by chica */
		run_hexapodCmds_task(&operData, &cmd_cBuff);
		
		/* Update readings and LED bar periodically */
		periodic_tasks(&operData);
		
		/* Protect against catastrophic failure */
		protection_tasks(&operData);
		
	} // while(1)
}
	
/*******************************************************************************
 * Function Definitions
 ******************************************************************************/
/*******************************************************************************
 * ISR Callbacks
 ******************************************************************************/
void gpio_callback(uint gpio, uint32_t events) 
{
	if (gpio == servo2040::USER_SW) {
		if (operData.led_mapType == current){operData.led_mapType = battSOC;}
		else if (operData.led_mapType == battSOC){operData.led_mapType = touchSensing;}
		else if (operData.led_mapType == touchSensing){operData.led_mapType = current;}
	}
}
/*******************************************************************************
 ******************************************************************************/
bool periodicTimer_callback(struct repeating_timer *t) 
{
	operData.run_periodicTasks = true;
	return (1);
}
/*******************************************************************************
 * Core Functions
 ******************************************************************************/
void serialStream_task(cbuf_handle_cmdPkt cmd_cBuff)
{
	int input;
	
	input = getchar_timeout_us(100);
	
	while (input != PICO_ERROR_TIMEOUT)
	{	
		// Check if command
		if (input & 0x80) 
		{
			errorCode error = none;
			cmdPkt curr_cmdPkt;
			uint value = 0;
			
			curr_cmdPkt.startIdx = getchar_timeout_us(10);
			curr_cmdPkt.count = getchar_timeout_us(10);
			
			if (input == SET_CMD){curr_cmdPkt.cmd = set;}
			else if (input == GET_CMD){curr_cmdPkt.cmd = get;}
			/* Handle errors */
			else {
				error = cmd_unrecognized;
				errorCode_ledMapping(error);
			} 
			if (curr_cmdPkt.startIdx & 0x80){
				error = startIdx_max_exceeded;
				errorCode_ledMapping(error);
			}
			if (curr_cmdPkt.count & 0x80){
				error = countIdx_max_exceeded;
				errorCode_ledMapping(error);
			}
			if (curr_cmdPkt.startIdx + curr_cmdPkt.count >= cmdPin_num){
				error = cmdPins_range_exceeded;
				errorCode_ledMapping(error);
			}

			/* Stop parsing if error occured */
			if (error != none){break;}
			
			if (curr_cmdPkt.cmd == set)
			{
				for (uint idx = 0; idx < curr_cmdPkt.count; idx++)
				{
					value = 0;
					value = (getchar_timeout_us(10) & 0x7F) << 7;
					value |= getchar_timeout_us(10) & 0x7F;
					
					// Limit values if startIdx is a servo pin
					if (curr_cmdPkt.startIdx + idx <= R33) {
						if (value > MAX_PULSE_VALUE){value = MAX_PULSE_VALUE;}
						else if (value < MIN_PULSE_VALUE){value = MIN_PULSE_VALUE;}	
					}
					
					curr_cmdPkt.valueBuff[idx] = value;
				}
			}
				
			circular_buf_put(cmd_cBuff, &curr_cmdPkt);
		} // if (input & 0x80) 
			
		input = getchar_timeout_us(100);
	} // while (input != PICO_ERROR_TIMEOUT)
}
/*******************************************************************************
 ******************************************************************************/
void run_hexapodCmds_task(sys_operData *operData, cbuf_handle_cmdPkt cmd_cBuff)
{
	cmdPkt currCmd;
	
	if (!circular_buf_get(cmd_cBuff, &currCmd))
	{
		if (currCmd.cmd == set)
		{
			for (uint idx = 0; idx < currCmd.count; idx++, currCmd.startIdx++)
			{
				// startIdx is servo
				if (currCmd.startIdx <= R33)
				{
					servos.pulse(cmdPin_to_hardwarePin((cmdPins)currCmd.startIdx), 
								 currCmd.valueBuff[idx], operData->servoEnabled);
#ifdef SERIAL_LOGGING
					printf("Setting pin %d to %d\r\n", currCmd.startIdx, 
													   currCmd.valueBuff[idx]);
#endif 
				} 
				// startIdx is A0/A1/A2
				else if (currCmd.startIdx >= RELAY)
				{
					bool enableState = currCmd.valueBuff[idx] ? true : false;
					
					// Set physical pins 
					gpio_put(cmdPin_to_hardwarePin((cmdPins)currCmd.startIdx), enableState) ;
					
					// Enable/disable PWM outputs
					if (currCmd.startIdx == RELAY)
					{
						operData->servoEnabled = enableState;
						if (enableState) {
							servos.enable_all();						
#ifdef SERIAL_LOGGING
							printf("Enabling all servos\r\n");
#endif 
						}
						else {
							servos.disable_all();								
#ifdef SERIAL_LOGGING
							printf("Disabling all servos\r\n");
#endif 							
						}
					}
				}
				// startIdx is not servo or A0/A1/A2
				else 
				{	
					errorCode_ledMapping(setCmd_cmdPin_incompatible);
				}
			} // for (auto idx = 0; idx < currCmd.count; idx++, currCmd.startIdx++)
		} // if (currCmd.cmd == set)
		else if (currCmd.cmd == get)
		{		
			uint tx[3] = {GET_CMD, currCmd.startIdx, currCmd.count};
			vcp_transmit(tx, 3);
			
			for (uint idx = 0; idx < currCmd.count; idx++, currCmd.startIdx++)
			{
				// startIdx is servo
				if (currCmd.startIdx <= R33)
				{
					uint pwmValue = 0;
					uint mappedPin = cmdPin_to_hardwarePin((cmdPins)currCmd.startIdx);
					pwmValue = servos.pulse(mappedPin);
					tx[0] = (pwmValue >> 7) & 0x7F;
					tx[1] = pwmValue & 0x7F;
					vcp_transmit(tx, 2);
				}
				// startIdx is touch sensor
				else if (currCmd.startIdx <= TS_R3)
				{
					uint mappedPin = cmdPin_to_hardwarePin((cmdPins)currCmd.startIdx);
					uint voltage_binary = round(operData->sysReadings.touchSens[mappedPin] 
												* b1024_3_3V_RATIO);
					tx[0] = (voltage_binary >> 7) & 0x7F;
					tx[1] = voltage_binary & 0x7F;
					vcp_transmit(tx, 2);
				}
				else if (currCmd.startIdx == CURR)
				{
					uint current_binary = round(operData->sysReadings.current/curr_LSb) + 512;
					tx[0] = (current_binary >> 7) & 0x7F;
					tx[1] = current_binary & 0x7F;
					vcp_transmit(tx, 2);
				}
				else if (currCmd.startIdx == VOLT)
				{
					uint voltage_binary = round(operData->sysReadings.voltage * b1024_3_3V_RATIO);
					tx[0] = (voltage_binary >> 7) & 0x7F;
					tx[1] = voltage_binary & 0x7F;
					vcp_transmit(tx, 2);
				}
				else
				{
					errorCode_ledMapping(getCmd_cmdPin_incompatible);
				}
			} // for (auto idx = 0; idx < currCmd.count; idx++, currCmd.startIdx++)
		} // else if (currCmd.cmd == get)
	}
}
/*******************************************************************************
 ******************************************************************************/
void periodic_tasks(sys_operData *operData)
{
	if (!operData->run_periodicTasks){return;}
	operData->run_periodicTasks = false;
	
	/* Update readings */
	operData->sysReadings.current = read_current();
	operData->sysReadings.voltage = read_voltage();
	read_analogPins(operData->sysReadings.touchSens);
	
	/* Update LED bar */
	switch (operData->led_mapType)
	{
	case current:
		{
			currMeasure_ledMapping(operData->sysReadings.current);
			break;
		}
	case battSOC:
		{
			batterySOC_ledMapping(operData->sysReadings.voltage);
			break;
		}
	case touchSensing:
		{
			touchSensing_ledMapping(operData->sysReadings.touchSens);
			break;
		}
	}
}
/*******************************************************************************
 ******************************************************************************/
void protection_tasks(sys_operData *operData)
{
	if (operData->sysReadings.current > OVERCURRENT) {
		operData->servoEnabled = false;
		gpio_put(A0_GPIO_PIN, false);				// Disable relay
		servos.disable_all();						// Disable servo torque
		errorCode_ledMapping(overcurrent_detected); // Report error
	}
	// todo: What condition to re-enable servos?
}

/*******************************************************************************
 * VCP/Parsing Support Functions
 ******************************************************************************/
uint cmdPin_to_hardwarePin(cmdPins cmdPin)
{
	return RP_hardwarePins_table[cmdPin];
}
/*******************************************************************************
 ******************************************************************************/
void vcp_transmit(uint *txbuff, uint size)
{
	for (uint byte = 0; byte < size; byte++) {
		putchar(txbuff[byte]);
	}
}

/*******************************************************************************
 * LED Support Functions
 ******************************************************************************/
void pendingVCP_ledSequence(void)
{
	static float offset = 0.0;
	const uint updates = 50;
	
	offset += 0.005;
	
	// Update all the LEDs
	for (auto i = 0u; i < servo2040::NUM_LEDS; i++) {
		float hue = (float)i / (float)servo2040::NUM_LEDS;
		led_bar.set_hsv(i, hue + offset, 1.0f, BRIGHTNESS);
	}

	sleep_ms(1000 / updates);
}
/*******************************************************************************
 ******************************************************************************/
void currMeasure_ledMapping(float measuredCurrent)
{
	// Convert the current to a percentage of the maximum we want to show
	float percent = (measuredCurrent / MAX_CURRENT);

	// Update all the LEDs
	for (auto i = 0u; i < servo2040::NUM_LEDS; i++) {
		// Calculate the LED's hue, with Red for high currents and Green for low
		float hue = (1.0f - (float)i / (float)(servo2040::NUM_LEDS - 1)) * 0.333f;

		// Calculate the current level the LED represents
		float level = (i + 0.5f) / servo2040::NUM_LEDS;
		
		// If the percent is above the level, light the LED, otherwise turn it off
		if (percent >= level)
			led_bar.set_hsv(i, hue, 1.0f, BRIGHTNESS);
		else
			led_bar.set_hsv(i, hue, 1.0f, 0.0f);
	}
}
/*******************************************************************************
 ******************************************************************************/
void batterySOC_ledMapping(float measuredVolatage)
{
	// Convert the voltage to a percentage of the maximum we want to show
	float percent = round(((measuredVolatage - MIN_VOLTAGE) / 
							(float)(MAX_VOLTAGE - MIN_VOLTAGE)) * 100);
		
	if (percent > 100){percent = 100;}
	else if (percent < 0){percent = 1;}
	
	// Update all the LEDs
	for (auto i = 0u; i < servo2040::NUM_LEDS; i++) {
		// Calculate the LED's hue, with Red for high currents and Green for low
//		float hue = (1.0f - (float)i / (float)(servo2040::NUM_LEDS - 1)) * 0.333f;

		// Calculate the current level the LED represents
		float level = ((i + 0.5f) / servo2040::NUM_LEDS)*100;
		
		// If the percent is above the level, light the LED, otherwise turn it off
		if (percent >= level)
			led_bar.set_hsv(i, GREEN_HUE, 1.0f, BRIGHTNESS);
		else
			led_bar.set_hsv(i, GREEN_HUE, 1.0f, 0.0f);
	}
}
/*******************************************************************************
 ******************************************************************************/
void touchSensing_ledMapping(float *touchSens)
{
	for (auto i = 0u; i < servo2040::NUM_LEDS; i++)	
	{
		if (touchSens[i] > 2) {
			led_bar.set_hsv(i, GREEN_HUE, 1.0f, BRIGHTNESS);
		}
		else {
			led_bar.set_hsv(i, GREEN_HUE, 1.0f, 0.0f);
		}
	}
}
/*******************************************************************************
 ******************************************************************************/
void errorCode_ledMapping(errorCode error)
{
#ifdef LED_DEBUGGING
	if (error == none){return;}
	
	/* Binary representation of error will be displayed via LEDs */
	for (auto i = 0u; i < servo2040::NUM_LEDS; i++)	
	{
		if (error & (1 << i)) {
			led_bar.set_hsv(i, RED_HUE, 1.0f, BRIGHTNESS);
		}
		else {
			led_bar.set_hsv(i, RED_HUE, 1.0f, 0.0);
		}
	}
	sleep_ms(10000); // Give user time to check error
	led_bar.clear();
#endif
}

/*******************************************************************************
 * Sensing Support Functions
 ******************************************************************************/
float read_current(void)
{
	// Select the current sense
	mux.select(servo2040::CURRENT_SENSE_ADDR);
	
	// Read the current sense several times and average the result
	float current = 0.0f;
	for (auto i = 0u; i < NUM_SAMPLES; i++) {
		current += cur_adc.read_current();
		sleep_ms(1);
	}
	current /= NUM_SAMPLES;
#ifdef SERIAL_LOGGING
	printf("Current: %f\r\n", current);
#endif
	return (current);
}
/*******************************************************************************
 ******************************************************************************/
float read_voltage(void)
{
	// Select the current sense
	mux.select(servo2040::VOLTAGE_SENSE_ADDR);
	
	// Read the current sense several times and average the result
	float voltage = 0.0f;
	for (auto i = 0u; i < NUM_SAMPLES; i++) {
		voltage += vol_adc.read_voltage();
		sleep_ms(1);
	}
	voltage /= NUM_SAMPLES;
#ifdef SERIAL_LOGGING
	printf("Voltage: %f\r\n", voltage);
#endif	
	return (voltage);
}
/*******************************************************************************
 ******************************************************************************/
void read_analogPins(float *touchSens)
{
	// Read each sensor in turn and print its voltage
	for (auto i = 0u; i < servo2040::NUM_SENSORS; i++) {
		mux.select(servo2040::SENSOR_1_ADDR + i);
		touchSens[i] = sen_adc.read_voltage();
	}
}
/*******************************************************************************
 * Circular Buffer
 ******************************************************************************/
inline void circular_buf_put(cbuf_handle_cmdPkt cbuf, cmdPkt *data)
{
	cbuf->buffer[cbuf->head] = *data;
	advance_pointer(cbuf);
}
/*******************************************************************************
 ******************************************************************************/
inline void advance_pointer(cbuf_handle_cmdPkt cbuf)
{
	if (cbuf->full) {
		cbuf->tail = (cbuf->tail + 1) % cbuf->max;
	}

	cbuf->head = (cbuf->head + 1) % cbuf->max;
	cbuf->full = (cbuf->head == cbuf->tail);
}
/*******************************************************************************
 ******************************************************************************/
inline int circular_buf_get(cbuf_handle_cmdPkt cbuf, cmdPkt *data)
{
	int r = -1;

	if (!circular_buf_empty(cbuf))
	{
		*data = cbuf->buffer[cbuf->tail];
		retreat_pointer(cbuf);

		r = 0;
	}

	return r;
}
/*******************************************************************************
 ******************************************************************************/
inline void retreat_pointer(cbuf_handle_cmdPkt cbuf)
{
	cbuf->full = 0;
	cbuf->tail = (cbuf->tail + 1) % cbuf->max;
}
/*******************************************************************************
 ******************************************************************************/
inline uint circular_buf_empty(cbuf_handle_cmdPkt cbuf)
{
	return (!cbuf->full && (cbuf->head == cbuf->tail));
}
/*******************************************************************************
 ******************************************************************************/
inline void circular_buf_reset(cbuf_handle_cmdPkt cbuf)
{
	cbuf->head = 0;
	cbuf->tail = 0;
	cbuf->full = 0;
}
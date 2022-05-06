#include "rtos_tasks.h"

extern QueueHandle_t modeQ;

void state_machine_transition(uint8_t mode)
{
	uint8_t curr_mode = CMD_STANDBY; // standby by default
	// get the current state to compare against
	xQueuePeek(modeQ, &curr_mode, 0);
	// make sure we are entering a new state

#if DEBUG
	char debug_str[16];
#endif

	if (mode == curr_mode) // if not, exit
	{
		return;
	}

	flywhl.stop();

	bool command_is_valid = true;

	switch (mode)
	{
	case CMD_HEARTBEAT:
		// do test command stuff
#if DEBUG
		SERCOM_USB.print("[mode switch]\tEntering HEARTBEAT mode\r\n");
#endif
		break;

	// WARNING: if you switch out of a test mode the test will still be on the stack
	// 	on the plus side, the task won't run though
	case CMD_TST_BASIC_MOTION:
	case CMD_TST_BASIC_AD:
	case CMD_TST_BASIC_AC:
	case CMD_TST_SIMPLE_DETUMBLE:
	case CMD_TST_SIMPLE_ORIENT:
	case CMD_TST_PHOTODIODES:
#if DEBUG
		SERCOM_USB.print("[mode switch]\tEntering TEST mode\r\n");
#endif

// #if RTOS_TEST_SUITE
		// create_test_tasks();
// #endif
		break;

	case CMD_STANDBY:
#if DEBUG
		SERCOM_USB.print("[mode switch]\tEntering STANDBY mode\r\n");
#endif
		// print heartbeat regularly turn off actuators
		// if(magnetorquer_on){
		// 	// TODO: SM turn magnetorquer off
		// }
		// if(flywhl_is_spinning){
		// 	// TODO: SM turn off the flywheel
		// }
		break;

	// TODO: SM fill out the other modes with functional code
	case CMD_ORIENT_DEFAULT: // should be orienting to something like X+
	case CMD_ORIENT_X_POS:
	case CMD_ORIENT_Y_POS:
	case CMD_ORIENT_X_NEG:
	case CMD_ORIENT_Y_NEG:
#if DEBUG
		SERCOM_USB.print("[mode switch]\tEntering ORIENT mode\r\n");
#endif
		break;

	default: // do nothing
		command_is_valid = false;
#if DEBUG
		// convert int to string for USB monitoring
		sprintf(debug_str, "0x%02x", mode);
		SERCOM_USB.print("[mode switch]\tUnknown command: ");
		SERCOM_USB.print(debug_str);
		SERCOM_USB.print("\r\n");
#endif
	}

	if (command_is_valid)
	{
		xQueueOverwrite(modeQ, (void *)&mode); // enter specified mode

		// TODO: SM init the new mode, maybe turn off an unneeded actuator? clear some data?
	}
}

void create_test_tasks(void)
{
#if DEBUG
	SERCOM_USB.print("[rtos]\t\tInitializing RTOS test suite\r\n");
#endif

	xTaskCreate(photodiode_test, "PHOTODIODE TEST", 256, NULL, 1, NULL);
#if DEBUG
	SERCOM_USB.print("[rtos]\t\tCreated photodiode test task\r\n");
#endif

	xTaskCreate(basic_motion, "BASIC MOTION", 256, NULL, 1, NULL);
#if DEBUG
	SERCOM_USB.print("[rtos]\t\tCreated basic motion task\r\n");
#endif

// 	xTaskCreate(basic_attitude_determination, "BASIC AD", 256, NULL, 1, NULL);
// #if DEBUG
// 	SERCOM_USB.print("[rtos]\t\tCreated basic attitude determination task\r\n");
// #endif

// 	xTaskCreate(basic_attitude_control, "BASIC AC", 256, NULL, 1, NULL);
// #if DEBUG
// 	SERCOM_USB.print("[rtos]\t\tCreated basic attitude control task\r\n");
// #endif

	xTaskCreate(simple_detumble, "SIMPLE DETUMBLE", 256, NULL, 1, NULL);
#if DEBUG
	SERCOM_USB.print("[rtos]\t\tCreated simple detumble task\r\n");
#endif

	xTaskCreate(simple_orient, "SIMPLE ORIENT", 256, NULL, 1, NULL);
#if DEBUG
	SERCOM_USB.print("[rtos]\t\tCreated simple orient task\r\n");
#endif

#if DEBUG
	SERCOM_USB.print("[rtos]\t\tInitialized RTOS test suite\r\n");
#endif
}

// get direction the adcs should turn to align X+ with the light source
//
// @param[in]  vals  The sensor values, unscaled
//
// @return     The direction to turn the adcs (clockwise or counter-clockwise)
//
MotorDirection getDirection(PDdata vals){
	uint8_t max=0; // channel receiving the most light
	float max_val=-1;
	// calculate side/channel receiving most light, ignore +/-Z so only look at first 4 values
	for(int i=0; i < 4; i++){
		if(vals.data[i] >= max_val){
			// save channel and value
			max_val = vals.data[i];
			max = i;
		}
	}

	#if DEBUG 
		SERCOM_USB.print("getDirection found max on channel = ");
		SERCOM_USB.println(max);
	#endif

	// translate to coordinates and calculate direction to spin
	switch(max){
		case X_POS:
			return IDLE;
		case X_NEG:
			return CW;
		case Y_POS:
			return CCW;
		case Y_NEG:
			return CW;
		case Z_POS:
		case Z_NEG:
			return IDLE;
	}
} 
/**
 * @brief
 * Polls the UART module for data. Processes data one byte at a time if the
 * module reports that data is ready to be received.
 *
 * @param[in] pvParameters  Unused but required by FreeRTOS. Program will not
 * compile without this parameter. When a task is instantiated from this
 * function, a set of initialization arguments or NULL is passed in as
 * pvParameters, so pvParameters must be declared even if it is not used.
 *
 * @return None
 *
 * TODO: Remove polling and invoke this task using an interrupt instead.
 */
void receiveCommand(void *pvParameters)
{
	TEScommand cmd_packet;
	ADCSdata response;
	uint8_t mode;

	int rx_len;
	int rx_bytes;
	uint8_t rx_buf[COMMAND_LEN];

#if DEBUG
	char debug_str[8]; // used to print command value to serial monitor
	SERCOM_USB.print("[command rx]\tTask started\r\n");
#endif

	while (1)
	{
		rx_len = SERCOM_UART.available();

		if (rx_len > 0) // at least one byte is in the UART
		{				// receive buffer
#if DEBUG
			sprintf(debug_str, "%d", rx_len);
			SERCOM_USB.print("[command rx]\tDetected ");
			SERCOM_USB.print(debug_str);
			SERCOM_USB.print(" bytes in UART rx buffer\r\n");
#endif

			rx_bytes = SERCOM_UART.readBytes(rx_buf, COMMAND_LEN);

#if DEBUG
			sprintf(debug_str, "%d", rx_bytes);
			SERCOM_USB.print("[command rx]\tReceived ");
			SERCOM_USB.print(debug_str);
			SERCOM_USB.print(" bytes:  [");

			for (int i = 0; i < rx_bytes; i++)
			{
				sprintf(debug_str, " %02x", rx_buf[i]);
				SERCOM_USB.print(debug_str);
			}

			SERCOM_USB.print(" ]\r\n");
#endif

			if (rx_bytes == COMMAND_LEN) // full command packet received
			{
				cmd_packet.loadBytes(rx_buf);

				// if (cmd_packet.checkCRC())
				// {
				state_machine_transition(cmd_packet.getCommand());
				// 				}
				// 				else
				// 				{
				// 					// send error message if CRC is not valid
				// 					response.setStatus(STATUS_COMM_ERROR);
				// 					response.computeCRC();
				// 					response.send();
				// #if DEBUG
				// 					SERCOM_USB.print("[command rx]\tCRC check failed - transmitting error message\r\n");
				// #endif
				// 				}
			}
			else
			{
				// send error message if data length is incorrect
				response.setStatus(STATUS_COMM_ERROR);
				response.send();
#if DEBUG
				SERCOM_USB.print("[command rx]\tReceived incorrect number of bytes - transmitting error message\r\n");
#endif
			}
		}

		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

/**
 * @brief
 * Reads magnetometer and gyroscope values from IMU and writes them to UART
 * every 0.5 seconds while ADCS is in test mode.
 *
 * @param[in] pvParameters  Unused but required by FreeRTOS. Program will not
 * compile without this parameter. When a task is instantiated from this
 * function, a set of initialization arguments or NULL is passed in as
 * pvParameters, so pvParameters must be declared even if it is not used.
 *
 * @return None
 */
void heartbeat(void *pvParameters)
{
	uint8_t mode;
	ADCSdata data_packet;
	INAdata ina;
	IMUdata imu;
	PDdata pd;

	uint8_t *tx_buf;

#if DEBUG
	char debug_str[16];
	SERCOM_USB.print("[heartbeat]\tTask started\r\n");
#endif

	while (1)
	{
		xSemaphoreTake(IMUsemphr, portMAX_DELAY);
		xQueuePeek(modeQ, (void *)&mode, (TickType_t)0);

		if (mode != CMD_STANDBY && mode != CMD_TST_PHOTODIODES)
		{
			data_packet.setStatus(STATUS_OK);

#if NUM_IMUS > 0
			xQueuePeek(IMUq, (void *)&imu, (TickType_t)10);
			data_packet.setIMUdata(imu);
#endif

#if INA
			ina = readINA();
			data_packet.setINAdata(ina);
#endif

			pd = readPD();

			data_packet.send(); // send to TES

#if DEBUG
			tx_buf = data_packet.getBytes();
			sprintf(debug_str, "%d", PACKET_LEN);
			SERCOM_USB.print("[heartbeat]\tTransmitted ");
			SERCOM_USB.print(debug_str);
			SERCOM_USB.print(" bytes:  [");

			for (int i = 0; i < PACKET_LEN; i++)
			{
				sprintf(debug_str, " %02x", tx_buf[i]);
				SERCOM_USB.print(debug_str);
			}

			SERCOM_USB.print(" ]\r\n");
#endif

			data_packet.clear();
		}

		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

void photodiode_test(void *pvParameters)
{
	uint8_t mode;
	PDdata pd;

#if DEBUG
	SERCOM_USB.print("[sun test]\tTask started\r\n");
#endif

	while (1)
	{
		xQueuePeek(modeQ, (void *)&mode, (TickType_t)0);

		if (mode == CMD_TST_PHOTODIODES)
		{
			pd = readPD();

			for (int channel = 0; channel < 6; channel++)
			{
#if DEBUG
				SERCOM_USB.print(pd.data[channel]);
				SERCOM_USB.print(", ");
#endif
			}
#if DEBUG
			SERCOM_USB.print("\r\n");
#endif
		}

		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

/*
 * Assume start from start, then begin increasing flywheel RPM until the system begins to spin.
 *  RULES:
 *      1. check the IMU every loop and if the rotation rate is above MAX_TEST_SPD, stop revving
 *      2. RPM only go up if some input is received...
 */
void basic_motion(void *pvParameters)
{
#if DEBUG
	char debug_str[16];
	SERCOM_USB.print("[basic motion]\tTask started\r\n");
#endif

	uint8_t *tx_buf;
	uint8_t mode;
	float multiplier = 0.00;
	int pwm_sig = 255 * multiplier; // 0%
	const int MAX_TEST_SPD = 10;	// upper limit is 10 degrees per second/1.667 rpm

	const TickType_t FREQ = 2000 / portTICK_PERIOD_MS;

	IMUdata imu;

	// notify the TES by sending an empty packet with status set
	ADCSdata data;
	data.setStatus(STATUS_TEST_START);

	while (true)
	{
// #if DEBUG
// 		SERCOM_USB.print("[basic motion]\tChecked mode\r\n");
// #endif

		xQueuePeek(modeQ, (void *)&mode, (TickType_t)0);

		if (mode == CMD_TST_BASIC_MOTION)
		{
			if (multiplier == 0.0)
			{
				data.send();
#if DEBUG
				tx_buf = data.getBytes();
				sprintf(debug_str, "%d", PACKET_LEN);
				SERCOM_USB.print("[basic motion]\tTransmitted ");
				SERCOM_USB.print(debug_str);
				SERCOM_USB.print(" bytes:  [");

				for (int i = 0; i < PACKET_LEN; i++)
				{
					sprintf(debug_str, " %02x", tx_buf[i]);
					SERCOM_USB.print(debug_str);
				}

				SERCOM_USB.print(" ]\r\n");
#endif
			}

			flywhl.run(CW, 2);

//			xSemaphoreTake(IMUsemphr, portMAX_DELAY);
// 			xQueuePeek(IMUq, (void *)&imu, (TickType_t)10);
// 			float rot_vel_z = imu.gyrZ;

// 			if (rot_vel_z < MAX_TEST_SPD && multiplier < 1.0)
// 			{ // as long as we are spinning slower than our goal, continue
// 				pwm_sig = 255 * multiplier;
// 				flywhl.run(CW, pwm_sig);

// 				if (multiplier < 1.0)
// 				{
// 					multiplier += 0.01;
// 				}
// #if DEBUG
// 				else
// 				{
// 					SERCOM_USB.print("[basic motion]\tMultiplier hit ceiling\r\n");
// 				}
// #endif
// 			}
// 			else
// 			{ // stop the test
// 				flywhl.stop();

// 				// notify the TES by sending an empty packet with status set
// 				data.setStatus(STATUS_TEST_END);
// 				data.computeCRC();
// 				data.send();
// #if DEBUG
// 				tx_buf = data.getBytes();
// 				sprintf(debug_str, "%d", PACKET_LEN);
// 				SERCOM_USB.print("[basic motion]\tTransmitted ");
// 				SERCOM_USB.print(debug_str);
// 				SERCOM_USB.print(" bytes:  [");

// 				for (int i = 0; i < PACKET_LEN; i++)
// 				{
// 					sprintf(debug_str, " %02x", tx_buf[i]);
// 					SERCOM_USB.print(debug_str);
// 				}

// 				SERCOM_USB.print(" ]\r\n");
// #endif

// #if DEBUG
// 				if (rot_vel_z > MAX_TEST_SPD)
// 				{
// 					SERCOM_USB.print("[basic motion]\tRotational velocity greater than MAX_TEST_SPD ( ");
// 					SERCOM_USB.print(MAX_TEST_SPD);
// 					SERCOM_USB.print(" deg/s )\r\n");
// 				}
// 				else if (multiplier >= 1)
// 				{
// 					SERCOM_USB.print("[basic motion]\tMultiplier hit ceiling but velocity still ");
// 					SERCOM_USB.print(rot_vel_z);
// 					SERCOM_USB.print(" deg/s\r\n");
// 				}
// #endif

// 				// change mode to standby
// 				uint8_t mode = MODE_STANDBY;
// 				xQueueOverwrite(modeQ, (void *)&mode); // enter specified mode
// 			}
		}

		vTaskDelay(FREQ);
	}
}

/**
 * @brief      calculate and output result of attitude determination, MODE_TEST_AD
 *
 * @param      pvParameters  The pv parameters
 */
void basic_attitude_determination(void *pvParameters)
{
	uint8_t mode;

#if DEBUG
	char debug_str[16];
	SERCOM_USB.print("[basic AD]\tTask started\r\n");
#endif

	while (true)
	{
// #if DEBUG
// 		SERCOM_USB.print("[basic AD]\tChecked mode\r\n");
// #endif
		xQueuePeek(modeQ, &mode, 0);

		if (mode == CMD_TST_BASIC_AD)
		{
			// TODO: write the attitude determination test in validation_tests.cpp
			// TODO: read IMU
			// TODO: use geographic lib to get ideal
			// TODO: calculate the vector...
		}
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

/**
 * @brief      attempt to spin the satellite a pre-determined amount, MODE_TEST_AC
 *
 * @param      pvParameters  The pv parameters
 */
void basic_attitude_control(void *pvParameters)
{
	uint8_t mode;

#if DEBUG
	char debug_str[16];
	SERCOM_USB.print("[basic AC]\tTask started\r\n");
#endif

	while (true)
	{
// #if DEBUG
// 		SERCOM_USB.print("[basic AC]\tChecked mode\r\n");
// #endif
		xQueuePeek(modeQ, &mode, 0);

		if (mode == CMD_TST_BASIC_AC)
		{
			// TODO: write the attitude control test in validation_tests.cpp
		}

		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

/**
 * @brief      test basic ability to stop system from spinning, MODE_TEST_SMPLTUMBLE

 *
 * @param      pvParameters  The pv parameters
 */
void simple_detumble(void *pvParameters)
{
#if DEBUG
	SERCOM_USB.print("[basic detumbl]\tTask started\r\n");
#endif

	uint8_t mode; // last received ADCS mode

	const int target_rot_vel = 0; // rotational velocity we want to maintain
/***********************************************************************************************************************/
	// int num_steps = 0; // causes divide by zero error on first iteration
	int num_steps = 1;
/***********************************************************************************************************************/

	const float P = 1; // proportional component
	int I = 0;			  // integral component
	int D = 0;			  // derivative component

	int prev_error = 0; // error from last loop iteration
	int error = 0;		// assumed stationary at start

	int pwm_output = 0; // init at zero, signed to represent direction

	IMUdata imu;

	while (true)
	{
// #if DEBUG
// 		SERCOM_USB.print("[basic detumbl]\tChecked mode\r\n");
// #endif
		xQueuePeek(modeQ, &mode, 0);

		if (mode == CMD_TST_SIMPLE_DETUMBLE)
		{
			xSemaphoreTake(IMUsemphr, portMAX_DELAY);
			// calculate error
			xQueuePeek(IMUq, &imu, 0);
			float rot_vel_z = imu.gyrZ;

			error = rot_vel_z - target_rot_vel; // difference between current state and target state
			// proportional term calculation
			float p_term = error * P;
			// integral term accumulation
			I += error;
			int i_term = I;
			// derivative term
			int d_term = I / num_steps;
			// output
			pwm_output = p_term; //+ i_term + d_term;
			if (pwm_output > 255)
			{ // cap output
				pwm_output = 255;
			}

			prev_error = error;
			num_steps++;

			// unsigned pwm to motor with direction
			if (error > 0)
			{
				flywhl.run(CW, abs(pwm_output));
			}
			else if (error < 0)
			{
				flywhl.run(CCW, abs(pwm_output));
			}
			else
			{
				// TODO: desaturate with magnetorquers if |pwm_output| == 255 or in state of equilibrium
				// for the meantime, just keep doing the same thing
			}

#if DEBUG
				SERCOM_USB.print("[basic detumbl]\t====== PID LOOP ======\r\n");
				SERCOM_USB.print("\t\tIMU VELOCITY = ");
				SERCOM_USB.print(rot_vel_z);
				SERCOM_USB.print(" degrees/sec\r\n");

				SERCOM_USB.print("\t\tERROR = ");
				SERCOM_USB.print(error);
				SERCOM_USB.print("\r\n");

				SERCOM_USB.print("\t\tPWM OUTPUT = ");
				SERCOM_USB.print(abs(pwm_output));
				SERCOM_USB.print("\r\n\t\t======================\r\n");
#endif

			prev_error = error;
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

/**
 * @brief      test ability to orient the system, MODE_TEST_ORIENT
 *
 * @param      pvParameters  The pv parameters
 */
void simple_orient(void *pvParameters)
{
	uint8_t mode;

#if DEBUG
	char debug_str[16];
	SERCOM_USB.print("[simple orient]\tTask started\r\n");
#endif

	while (true)
	{
// #if DEBUG
// 		SERCOM_USB.print("[simple orient]\tChecked mode\r\n");
// #endif

		xQueuePeek(modeQ, &mode, 0);

		if (mode == CMD_TST_SIMPLE_ORIENT)
		{
			// read photodiodes
			PDdata pdata = readPD();
			// read the direction the satellite needs to move to align
			MotorDirection md = getDirection(pdata);

			// spin motor so that X+ is pointed at light
			if(md != IDLE){
				#if DEBUG
					SERCOM_USB.println("[simple orient]\t motor set to run");
				#endif
				flywhl.run(md, 3);
			}else{	// aligned so don't move
				#if DEBUG
					SERCOM_USB.println("[simple orient]\t motor stopped");
				#endif
				flywhl.stop();
			}
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
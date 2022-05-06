#include "sensors.h"

ICM_20948_I2C IMU1;
ICM_20948_I2C IMU2;

INA209 ina209;

ADCSPhotodiodeArray sunSensors(A0, 13, 12, 11);

QueueHandle_t IMUq;
QueueHandle_t INAq;
QueueHandle_t PDq;

SemaphoreHandle_t IMUsemphr;
SemaphoreHandle_t INAsemphr;
SemaphoreHandle_t PDsemphr;

/* HARDWARE INIT FUNCTIONS ================================================== */

void initIMU(void)
{
	/**
	 * Initialize first IMU
	 * Address: 0x69 or 0x68
	 */
    IMU1.begin(SERCOM_I2C, AD0_VAL);
    while (IMU1.status != ICM_20948_Stat_Ok);  // wait for initialization to
                                               // complete
#if DEBUG
    SERCOM_USB.print("[system init]\tIMU1 initialized\r\n");
#endif

#if NUM_IMUS >= 2
	/**
	 * Initialize second IMU
	 * Address: 0x68 or 0x69
	 */
    IMU2.begin(SERCOM_I2C, AD0_VAL^1);  // initialize other IMU with opposite
										// value for bit 0
    while (IMU2.status != ICM_20948_Stat_Ok);  // wait for initialization to
                                               // complete
	#if DEBUG
    SERCOM_USB.print("[system init]\tIMU2 initialized\r\n");
	#endif
#endif

	IMUq = xQueueCreate(1, sizeof(IMUdata));
	IMUdata dummy_init;
	dummy_init.magX = 0.0f;
	dummy_init.magY = 0.0f;
	dummy_init.magZ = 0.0f;
	dummy_init.gyrZ = 0.0f;
	xQueueSend(IMUq, (void *)&dummy_init, (TickType_t)0);

	IMUsemphr = xSemaphoreCreateBinary();
	xSemaphoreGive(IMUsemphr);

	xTaskCreate(readIMU, "IMU read", 256, NULL, 1, NULL);
#if DEBUG
	SERCOM_USB.print("[rtos]\t\tCreated IMU read task\r\n");
#endif
}

void initINA(void)
{
	/**
	 * Write default settings to INA209
	 * Reset: no
	 * Bus voltage range: 32V
	 * PGA gain: /8
	 * PGA range: +-320mV
	 * ADC resolution: 12 bits
	 * ADC conversion time: 532us
	 * Mode: shunt and bus, continuous
	 */
    ina209.writeCfgReg(0x399f);

	/**
	 * Calibrate INA209
	 * Current LSB: 100uA
	 * 
	 * Can also use 0x6aaa to prevent overflow
	 * 7fff seems to be more accurate though
	 */
    ina209.writeCal(0x7fff);
    
#if DEBUG
    SERCOM_USB.print("[system init]\tINA209 initialized\r\n");
#endif

	INAq = xQueueCreate(1, sizeof(INAdata));
	INAdata dummy_init;
	dummy_init.voltage = 0.0f;
	dummy_init.current = 0.0f;
	xQueueSend(INAq, (void *)&dummy_init, (TickType_t)0);

	INAsemphr = xSemaphoreCreateBinary();
	xSemaphoreGive(INAsemphr);

	// xTaskCreate(readINA_rtos, "INA read", 256, NULL, 1, NULL);
}

void initSunSensors(void)
{
	sunSensors.init();
#if DEBUG
	SERCOM_USB.print("[system init]\tSun sensors initialized\r\n");
#endif
}

/* SENSOR READING FUNCTIONS ================================================= */

INAdata readINA(void)
{
	INAdata data;

	int v_raw;
	int i_raw;

	v_raw = ina209.busVol();
	i_raw = ina209.current();

	data.voltage = v_raw / 1000.0f;
	data.current = i_raw / 10.0f;

#if DEBUG
	SERCOM_USB.print("[readINA]\tBus voltage: ");
	SERCOM_USB.print(data.voltage);
	SERCOM_USB.print("V\r\n");

	SERCOM_USB.print("[readINA]\tCurrent: ");
	SERCOM_USB.print(data.current);
	SERCOM_USB.print("mA\r\n");
#endif

	return data;
}

PDdata readPD(void)
{
	PDdata data;
	uint8_t channel;

	for (channel = 0; channel < 6; channel++)
	{
		data.data[channel] = sunSensors.read(channel);
	}

// #if DEBUG
// 	SERCOM_USB.print("[readPD]\tSun sensors: [ [");
// 	SERCOM_USB.print(data.x_pos);
// 	SERCOM_USB.print(", ");
// 	SERCOM_USB.print(data.x_neg);
// 	SERCOM_USB.print("], [");
// 	SERCOM_USB.print(data.y_pos);
// 	SERCOM_USB.print(", ");
// 	SERCOM_USB.print(data.y_neg);
// 	SERCOM_USB.print("], [");
// 	SERCOM_USB.print(data.z_pos);
// 	SERCOM_USB.print(", ");
// 	SERCOM_USB.print(data.z_neg);
// 	SERCOM_USB.print("] ]\r\n");
// #endif

	return data;
}

/* SENSOR RTOS TASKS ======================================================== */

void readIMU(void *pvParameters)
{
	ICM_20948_I2C *sensor_ptr1 = &IMU1;
#if NUM_IMUS >= 2
	ICM_20948_I2C *sensor_ptr2 = &IMU2;
#endif

	IMUdata result;

	const int DECIMATION = 4;
	const int NUM_DECIMATIONS = 8;

	float gyrXavgs[NUM_DECIMATIONS];
	float gyrYavgs[NUM_DECIMATIONS];
	float gyrZavgs[NUM_DECIMATIONS];

	float gyrXresult = 0.0f;
	float gyrYresult = 0.0f;
	float gyrZresult = 0.0f;

	int readcntr = 0;
	int avgcntr = 0;

	result.magX = 0.0f;
	result.magY = 0.0f;
	result.magZ = 0.0f;
	result.gyrX = 0.0f;
	result.gyrY = 0.0f;
	result.gyrZ = 0.0f;

	for (int i = 0; i < NUM_DECIMATIONS; i++)
	{
		gyrXavgs[i] = 0.0f;
		gyrYavgs[i] = 0.0f;
		gyrZavgs[i] = 0.0f;
	}
	
	while (1)
	{
#if NUM_IMUS >= 2
		if (IMU1.dataReady() && IMU2.dataReady())
		{
#else
		if (IMU1.dataReady())
		{
#endif
			xSemaphoreTake(IMUsemphr, 0);
			IMU1.getAGMT();
#if NUM_IMUS >= 2
			IMU2.getAGMT();
#endif
			xSemaphoreGive(IMUsemphr);

			result.magX = sensor_ptr1->magX();
			result.magY = sensor_ptr1->magY();
			result.magZ = sensor_ptr1->magZ();

#if NUM_IMUS >= 2
			gyrXavgs[avgcntr] += ((sensor_ptr1->gyrX() + sensor_ptr2->gyrX()) / 2);
			gyrYavgs[avgcntr] += ((sensor_ptr1->gyrY() + sensor_ptr2->gyrY()) / 2);
			gyrZavgs[avgcntr] += ((sensor_ptr1->gyrZ() + sensor_ptr2->gyrZ()) / 2);
#else
			gyrXavgs[avgcntr] += sensor_ptr1->gyrX();
			gyrYavgs[avgcntr] += sensor_ptr1->gyrY();
			gyrZavgs[avgcntr] += sensor_ptr1->gyrZ();
#endif

			readcntr++;

			if (readcntr >= DECIMATION)
			{
				gyrXavgs[avgcntr] /= (float)DECIMATION;
				gyrYavgs[avgcntr] /= (float)DECIMATION;
				gyrZavgs[avgcntr] /= (float)DECIMATION;

				for (int i = 0; i < NUM_DECIMATIONS; i++)
				{
					gyrXresult += gyrXavgs[i];
					gyrYresult += gyrYavgs[i];
					gyrZresult += gyrZavgs[i];
				}

				gyrXresult /= (float)NUM_DECIMATIONS;
				gyrYresult /= (float)NUM_DECIMATIONS;
				gyrZresult /= (float)NUM_DECIMATIONS;

				result.gyrX = gyrXresult;
				result.gyrY = gyrYresult;
				result.gyrZ = gyrZresult;

				avgcntr = (avgcntr + 1) % NUM_DECIMATIONS;

				gyrXavgs[avgcntr] = 0.0f;
				gyrYavgs[avgcntr] = 0.0f;
				gyrZavgs[avgcntr] = 0.0f;

				readcntr = 0;
			}
		}

		xQueueOverwrite(IMUq, &result);

		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
}

void readINA_rtos(void *pvParameters)
{

}

/* PRINTING FUNCTIONS ======================================================= */

void printPaddedInt16b(int16_t val)
{
	if (val > 0)
	{
		SERCOM_USB.print(" ");
		if (val < 10000)
		{
			SERCOM_USB.print("0");
		}
		if (val < 1000)
		{
			SERCOM_USB.print("0");
		}
		if (val < 100)
		{
			SERCOM_USB.print("0");
		}
		if (val < 10)
		{
			SERCOM_USB.print("0");
		}
	}
	else
	{
		SERCOM_USB.print("-");
		if (abs(val) < 10000)
		{
			SERCOM_USB.print("0");
		}
		if (abs(val) < 1000)
		{
			SERCOM_USB.print("0");
		}
		if (abs(val) < 100)
		{
			SERCOM_USB.print("0");
		}
		if (abs(val) < 10)
		{
			SERCOM_USB.print("0");
		}
	}
	SERCOM_USB.print(abs(val));
}

/**
 * @brief
 * This function has something to do with printing IMU data over USB to the
 * serial monitor. Other than that, I have no idea what it does. It came as part
 * of the IMU demo code, and printScaledAGMT, which is used to validate data
 * transmissions, relies on this function.
 * 
 * @param[in] agmt  An instance of the IMU data object
 * 
 * @return None
 */
void printRawAGMT(ICM_20948_AGMT_t agmt)
{
	SERCOM_USB.print("RAW. Acc [ ");
	printPaddedInt16b(agmt.acc.axes.x);
	SERCOM_USB.print(", ");
	printPaddedInt16b(agmt.acc.axes.y);
	SERCOM_USB.print(", ");
	printPaddedInt16b(agmt.acc.axes.z);
	SERCOM_USB.print(" ], Gyr [ ");
	printPaddedInt16b(agmt.gyr.axes.x);
	SERCOM_USB.print(", ");
	printPaddedInt16b(agmt.gyr.axes.y);
	SERCOM_USB.print(", ");
	printPaddedInt16b(agmt.gyr.axes.z);
	SERCOM_USB.print(" ], Mag [ ");
	printPaddedInt16b(agmt.mag.axes.x);
	SERCOM_USB.print(", ");
	printPaddedInt16b(agmt.mag.axes.y);
	SERCOM_USB.print(", ");
	printPaddedInt16b(agmt.mag.axes.z);
	SERCOM_USB.print(" ], Tmp [ ");
	printPaddedInt16b(agmt.tmp.val);
	SERCOM_USB.print(" ]");
	SERCOM_USB.println();
}

/**
 * @brief
 * This function has something to do with printing IMU data over USB to the
 * serial monitor. Other than that, I have no idea what it does. It came as part
 * of the IMU demo code, and printScaledAGMT, which is used to validate data
 * transmissions, relies on this function.
 * 
 * @param[in] val       Value to print
 * @param[in] leading   Number of digits left of the decimal
 * @param[in] decimals  Number of digits right of the decimal
 * 
 * @return None
 */
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
	float aval = abs(val);
	if (val < 0)
	{
		SERCOM_USB.print("-");
	}
	else
	{
		SERCOM_USB.print(" ");
	}
	for (uint8_t indi = 0; indi < leading; indi++)
	{
		uint32_t tenpow = 0;
		if (indi < (leading - 1))
		{
			tenpow = 1;
		}
		for (uint8_t c = 0; c < (leading - 1 - indi); c++)
		{
			tenpow *= 10;
		}
		if (aval < tenpow)
		{
			SERCOM_USB.print("0");
		}
		else
		{
			break;
		}
	}
	if (val < 0)
	{
		SERCOM_USB.print(-val, decimals);
	}
	else
	{
		SERCOM_USB.print(val, decimals);
	}
}

/**
 * @brief
 * Prints IMU data over USB to the serial monitor. Converts raw data to a form
 * that is readable by humans. Came as part of the IMU demo code.
 * 
 * @param[in] sensor  Pointer to IMU object
 * 
 * @return None
 */
void printScaledAGMT(ICM_20948_I2C *sensor)
{
	SERCOM_USB.print("Scaled. Acc (mg) [ ");
	printFormattedFloat(sensor->accX(), 5, 2);
	SERCOM_USB.print(", ");
	printFormattedFloat(sensor->accY(), 5, 2);
	SERCOM_USB.print(", ");
	printFormattedFloat(sensor->accZ(), 5, 2);
	SERCOM_USB.print(" ], Gyr (DPS) [ ");
	printFormattedFloat(sensor->gyrX(), 5, 2);
	SERCOM_USB.print(", ");
	printFormattedFloat(sensor->gyrY(), 5, 2);
	SERCOM_USB.print(", ");
	printFormattedFloat(sensor->gyrZ(), 5, 2);
	SERCOM_USB.print(" ], Mag (uT) [ ");
	printFormattedFloat(sensor->magX(), 5, 2);
	SERCOM_USB.print(", ");
	printFormattedFloat(sensor->magY(), 5, 2);
	SERCOM_USB.print(", ");
	printFormattedFloat(sensor->magZ(), 5, 2);
	SERCOM_USB.print(" ], Tmp (C) [ ");
	printFormattedFloat(sensor->temp(), 5, 2);
	SERCOM_USB.print(" ]");
	SERCOM_USB.println();
}
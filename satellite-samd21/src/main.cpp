#include "comm.h"
#include "CRC16.h"
#include "FreeRTOS_SAMD21.h"
#include <stdint.h>

// if defined, enables debug print statements over USB to the serial monitor
#define DEBUG

// create more descriptive names for serial interfaces
#define SERCOM_USB   SerialUSB
#define SERCOM_UART  Serial1

/* GLOBAL VARIABLES ========================================================= */

TEScommand cmd_packet;
ADCSdata data_packet;

/* FUNCTION DECLARATIONS ==================================================== */

// task function declarations
static void readUART(void *pvParameters);
static void writeUART(void *pvParameters);

/* FUNCTION DEFINITIONS ===================================================== */

void setup()
{
	clearTEScommand(&cmd_packet);
	clearADCSdata(&data_packet);

#ifdef DEBUG
    /**
     * Initialize UART connection to satellite
     * Baud rate: 115200
     * Data bits: 8
     * Parity: none
     */
    SERCOM_USB.begin(115200);
    while (!SERCOM_USB);  // wait for initialization to complete
    SERCOM_USB.write("USB interface initialized\r\n");
#endif

    /**
     * Initialize UART connection to satellite
     * Baud rate: 115200
     * Data bits: 8
     * Parity: odd (1 bit)
     */
    SERCOM_UART.begin(115200, SERIAL_8O1);
    while (!SERCOM_UART);  // wait for initialization to complete
#ifdef DEBUG
    SERCOM_USB.write("UART interface initialized\r\n");
#endif

    // instantiate tasks and start scheduler
    xTaskCreate(readUART, "Read UART", 1024, NULL, 1, NULL);
    xTaskCreate(writeUART, "Write UART", 1024, NULL, 1, NULL);

#ifdef DEBUG
    SERCOM_USB.write("Tasks created\r\n");
#endif

    vTaskStartScheduler();

    // should never be reached if everything goes right
    while (1);
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
 */
static void readUART(void *pvParameters)
{
    uint8_t bytes_received = 0;  // number of consecutive bytes received from
                                 // satellite - used as index for cmd packet
                                 // char array
	CRC16 crcGen;

#ifdef DEBUG
    char data_str[256];  // used to print command value to serial monitor
#endif

    while (1)
    {
        if (SERCOM_UART.available())  // at least one byte is in the UART
        {							  // receive buffer

            // copy one byte out of UART receive buffer
            data_packet.data[bytes_received] = SERCOM_UART.read();
            bytes_received++;

            if (bytes_received >= PACKET_LEN)  // full command packet received
            {
                // TODO: verify CRC
				crcGen.reset();
				crcGen.add((uint8_t*)(data_packet.data), PACKET_LEN-2);

#ifdef DEBUG
				if (crcGen.getCRC() == data_packet.crc)
					SERCOM_USB.print("VALID - ");
				else
					SERCOM_USB.print("INVALID - ");
					
                // print data value to USB
                SERCOM_USB.print("V: ");
                SERCOM_USB.print(data_packet.voltage);

				SERCOM_USB.print(" V; I: ");
                SERCOM_USB.print(data_packet.current * 10);

				SERCOM_USB.print(" mA; Speed: ");
                SERCOM_USB.print(fixedToFloat(data_packet.speed));

				SERCOM_USB.print(" RPS; Mag: [");
                SERCOM_USB.print(data_packet.magX);

				SERCOM_USB.print(", ");
                SERCOM_USB.print(data_packet.magY);

				SERCOM_USB.print(", ");
                SERCOM_USB.print(data_packet.magZ);

				SERCOM_USB.print("] uT; Gyro: [");
                SERCOM_USB.print(fixedToFloat(data_packet.gyroX));

				SERCOM_USB.print(", ");
                SERCOM_USB.print(fixedToFloat(data_packet.gyroY));

				SERCOM_USB.print(", ");
                SERCOM_USB.print(fixedToFloat(data_packet.gyroZ));
				SERCOM_USB.print("] DPS\r\n");
#endif

                // reset index counter to zero for next command
                bytes_received = 0;
            }
        }
    }
}

/**
 * @brief
 * Sends a command to the ADCS every 10 seconds, switching between test mode and
 * standby mode.
 * 
 * @param[in] pvParameters  Unused but required by FreeRTOS. Program will not
 * compile without this parameter. When a task is instantiated from this
 * function, a set of initialization arguments or NULL is passed in as
 * pvParameters, so pvParameters must be declared even if it is not used.
 * 
 * @return None
 */
static void writeUART(void *pvParameters)
{
	CRC16 crcGen;

	while (1)
    {
        cmd_packet.command = (uint8_t)COMMAND_TEST;
		crcGen.reset();
		crcGen.add((uint8_t*)cmd_packet.data, COMMAND_LEN-2);
		cmd_packet.crc = crcGen.getCRC();
		
		// clearADCSdata(&data_packet);

		SERCOM_UART.write((char*)(cmd_packet.data), COMMAND_LEN);
		SERCOM_USB.write("Command sent: test mode\r\n");
        vTaskDelay(10000 / portTICK_PERIOD_MS);

		cmd_packet.command = (uint8_t)COMMAND_STANDBY;
		crcGen.reset();
		crcGen.add((uint8_t*)cmd_packet.data, COMMAND_LEN-2);
		cmd_packet.crc = crcGen.getCRC();
		
		// clearADCSdata(&data_packet);

		SERCOM_UART.write((char*)(cmd_packet.data), COMMAND_LEN);
		SERCOM_USB.write("Command sent: standby mode\r\n");
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief
 * Does nothing. Since we are using FreeRTOS, we will not use Arduino's loop
 * function. However, the project will fail to compile if loop is not defined.
 * Therefore, we define loop to do nothing.
 */
void loop()
{
  	// do nothing
}
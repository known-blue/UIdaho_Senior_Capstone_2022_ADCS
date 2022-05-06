#ifndef __RTOS_TASKS_H__
#define __RTOS_TASKS_H__

#include <global_definitions.h>
#include <comm.h>
#include <actuators.h>
#include <FreeRTOS_SAMD51.h>

extern DRV10970 flywhl;
extern ICM_20948_I2C IMU1;
extern QueueHandle_t modeQ;
extern QueueHandle_t IMUq;
extern SemaphoreHandle_t IMUsemphr;

void state_machine_transition(uint8_t cmd);
void create_test_tasks(void);

MotorDirection getDirection(PDdata); // get direction the adcs should turn to align X+ with the light source

// RTOS TASKS /////////////////////////////////////////////////////
void receiveCommand(void *pvParameters);
void heartbeat(void *pvParameters);
void photodiode_test(void *pvParameters);

void basic_motion(void* pvParameters);
void basic_attitude_determination(void *pvParameters);
void basic_attitude_control(void *pvParameters);
void simple_detumble(void *pvParameters);
void simple_orient(void *pvParameters);

#endif
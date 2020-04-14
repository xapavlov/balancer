#include "stdint.h"
#include <stdlib.h>
#include "string.h"
#include "MPU6000.h"

#define RECEIVE_PACKET_LENGTH 5
#define TRANSMIT_PACKET_LENGTH 20
#define M_PI 3.14159265f

#define __ENABLE_LED1     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define __DISABLE_LED1      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define __ENABLE_LED2     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET)
#define __DISABLE_LED2      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET)

void SendToMatlab(void);
void SendToMatlabShort(void);
int ParsePacket(void);
void SetMotorsPWM(void);
void GetPosition(void);
uint32_t MakeDecision(void);

void EXTI2Callback(void);
void TIM2Callback(void);
void GetVelocity(void);
void ComputeTheta(void);
void CalculatePID(void);
void BufferPush(uint8_t);
void EstimateState1ms();
void EstimateState10ms();
void PrepareLP(uint32_t, int);

float ComputeControl();
void ApplyControl(float control);
float ComputeLQR();

void test_lp();
uint8_t BufferPull(uint8_t);


extern float control;
extern float control_pwm;
extern float theta;
extern uint8_t pData[];
extern uint8_t receive_buffer[256];
extern uint8_t receive_buffer_head;

extern uint8_t flag_received_packet;
extern uint8_t flag_ready_packet;
extern uint16_t time_updates;

extern int16_t changes_of_sign[2];

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim5;
extern uint32_t cnt_at_last_pulse[2];
extern uint16_t time_updates_at_last_pulse[2];
extern int16_t pulse_sign[2];

enum ConnnectionState { 
  NO_CONNECTION,
  GOOD_CONNECTION
};

enum SystemState { 
  WAITING,
  STARTING,
  WORKING,
  STOPPING
};

#include "user.h"
#include "math.h"
#include "linprog.h"

#include "control_matrix.h"
#include "cost_matrix.h"
#include "location_matrix.h"


const int8_t vertices[4][16]=
{{0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1},
 {0,0,0,0,1,1,1,1,0,0,0,0,1,1,1,1},
 {0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1},
 {0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1}};

//float lqr[4]={2.9605, 8.1369, 749.8926, 172.7059}; //749.8926, 172.7059
float lqr[4]={6.37640732479400,13.8646931502208,713.145165132753,177.915021144309};
uint8_t receive_buffer[256]; //cyclic buffer
uint8_t receive_buffer_head=0;
uint8_t pData[]="\0";

uint8_t motor_packet_1[4];
uint8_t motor_packet_2[4];

uint8_t flag_received_packet=0;
uint8_t flag_ready_packet=0;

uint8_t send_buffer[TRANSMIT_PACKET_LENGTH];

uint8_t ID=0;

int32_t position[2];
float velocity[2];
float velocity_pred;
int16_t changes_of_sign[2]={-1,-1};
uint32_t cnt_at_last_pulse[2];
uint16_t time_updates_at_last_pulse[2];
int16_t pulse_sign[2];
float asd;
float delta_n[2];
float theta;
float theta_acc;
float theta_bias=0;

uint32_t time_start=0;
uint32_t time_end1=0;
uint32_t time_end2=0;

float motor1, motor2;
int8_t motor[2];

float control;
float control_pwm;
float sysState[4];
float stateInBox[4]={0,0,0,0};

void BufferPush(uint8_t byte)
{
  receive_buffer[receive_buffer_head++]=byte;
}

uint8_t BufferPull(uint8_t depth)
{
  return receive_buffer[receive_buffer_head-depth];
}


//BuffToSend
//0 - [ - start byte 
//1 - ID
//2-7 - accel_8[0-5]
//8-13 - gyro_8[0-5]
//14-15 - distance
//16-17 - distance
//18 - hash - xor(2 - packet end byte) 
//19 - ] - stop byte


void SendToMatlab(void)
{
  send_buffer[0]='[';
  send_buffer[1]=ID;
  
  /*
  for (int i=0; i<6; i++){
    send_buffer[i+2]=accel_8[i];
    send_buffer[i+8]=gyro_8[i];
  }
  */
  
  send_buffer[2]=(uint8_t)((((int16_t)(sysState[0]*10000))>>8)&0xFF);
  send_buffer[3]=(uint8_t)((((int16_t)(sysState[0]*10000)))&0xFF);  
  
  send_buffer[4]=(uint8_t)((((int16_t)(sysState[1]*10000))>>8)&0xFF);
  send_buffer[5]=(uint8_t)((((int16_t)(sysState[1]*10000)))&0xFF);  
  
  send_buffer[6]=(uint8_t)((((int16_t)(sysState[2]*10000))>>8)&0xFF);
  send_buffer[7]=(uint8_t)((((int16_t)(sysState[2]*10000)))&0xFF);  
  
  send_buffer[8]=(uint8_t)((((int16_t)(sysState[3]*10000))>>8)&0xFF);
  send_buffer[9]=(uint8_t)((((int16_t)(sysState[3]*10000)))&0xFF);  
  
  send_buffer[10]=(uint8_t)((((int16_t)(control*100))>>8)&0xFF);
  send_buffer[11]=(uint8_t)((((int16_t)(control*100)))&0xFF);  
  
  send_buffer[12]=(uint8_t)(((time_end1)>>8)&0xFF);
  send_buffer[13]=(uint8_t)(((time_end1))&0xFF);  
  
  send_buffer[14]=(uint8_t)(((time_end2)>>8)&0xFF);
  send_buffer[15]=(uint8_t)(((time_end2))&0xFF);  
  
  send_buffer[16]=']';

  ID++;
  HAL_UART_Transmit_IT(&huart1, send_buffer, 17);
}

void SendToMatlabShort(void)
{
  send_buffer[0]='[';
  send_buffer[1]=ID;
  send_buffer[2]=(uint8_t)((((uint16_t)control)>>8)&0xFF);
  send_buffer[3]=(uint8_t)((((uint16_t)control))&0xFF);  
  send_buffer[4]=(uint8_t)((((uint16_t)control_pwm)>>8)&0xFF);
  send_buffer[5]=(uint8_t)((((uint16_t)control_pwm))&0xFF);  
  send_buffer[6]=(uint8_t)((((int16_t)accel_16[2])>>8)&0xFF);
  send_buffer[7]=(uint8_t)((((int16_t)accel_16[2]))&0xFF);  
  send_buffer[8]=']';
  ID++;
  HAL_UART_Transmit_IT(&huart1, send_buffer, 9);
}

void GetPosition(void)
{
  position[0]=changes_of_sign[0]*0xFFFF + TIM3->CNT;
  position[1]=changes_of_sign[1]*0xFFFF + TIM2->CNT;
}

void GetVelocity(void)
{
  
  static uint32_t time_prev;
  static int32_t delta_t[2]={0,0};
  static int32_t delta_t_prev[2]={0,0};
  static int32_t position_prev[2]={0,0};
  static int32_t acc_time[2]={0,0};
  uint32_t time;
  
  time=__HAL_TIM_GET_COUNTER(&htim5);
  
  position[0]=changes_of_sign[0]*0xFFFF + TIM3->CNT;
  position[1]=changes_of_sign[1]*0xFFFF + TIM2->CNT;

  delta_t[0]=time-cnt_at_last_pulse[0];
  delta_t[1]=time-cnt_at_last_pulse[1];
    
  if ((position[0]-position_prev[0])==0){
    acc_time[0]=acc_time[0]+(time-time_prev);
  }else{
    acc_time[0]=time-time_prev;
  }
  
  if ((position[1]-position_prev[1])==0){
    acc_time[1]=acc_time[1]+(time-time_prev);
  }else{
    acc_time[1]=time-time_prev;
  }
  
  velocity[0]=((position[0]-position_prev[0])*1e6f)/((float)(acc_time[0]+delta_t_prev[0]-delta_t[0]));
  velocity[1]=((position[1]-position_prev[1])*1e6f)/((float)(acc_time[1]+delta_t_prev[1]-delta_t[1]));
  
  if (isnan(velocity[0])||isinf(velocity[0])) velocity[0]=0;
  if (isnan(velocity[1])||isinf(velocity[1])) velocity[1]=0;

  time_prev=time;
  delta_t_prev[0]=delta_t[0];
  delta_t_prev[1]=delta_t[1];
  position_prev[0]=position[0];
  position_prev[1]=position[1];
}

void SetMotorsPWM(void)
{
  //memcpy(&motor1, &motor_packet_1, sizeof(motor1));
  //memcpy(&motor2, &motor_packet_2, sizeof(motor2));
  
  memcpy(&motor[0], &motor_packet_1[0], sizeof(motor_packet_1[0]));
  memcpy(&motor[1], &motor_packet_2[0], sizeof(motor_packet_2[0]));
  
  int8_t delta=1;
  if (motor[0]>delta){
    TIM4->CCR3=(uint16_t)(0xFFFF*0.01f*(motor[0]+25)); 
    TIM4->CCR4=0;
  }
  else if (motor[0]<-delta){
    TIM4->CCR3=0; 
    TIM4->CCR4=(uint16_t)(0xFFFF*0.01f*(-motor[0]+25)); 
  }
  else {
    TIM4->CCR3=0; 
    TIM4->CCR4=0; 
  }
  
  if (motor[1]>delta){
    TIM9->CCR1=(uint16_t)(0xFFFF*0.01f*(motor[1]+25)); 
    TIM9->CCR2=0;
  }
  else if (motor[1]<-delta){
    TIM9->CCR1=0; 
    TIM9->CCR2=(uint16_t)(0xFFFF*0.01f*(-motor[1]+25)); 
  }
  else {
    TIM9->CCR1=0; 
    TIM9->CCR2=0; 
  }
}

void ApplyControl(float control)
{
  //memcpy(&motor1, &motor_packet_1, sizeof(motor1));
  //memcpy(&motor2, &motor_packet_2, sizeof(motor2));
  static float old_control=0;
  
  float new_control;
  float add;
  float delta;
  int diff;
  add=25;
  delta=1;
  //new_control=0.2f*control+0.8f*old_control;
  new_control=0.2f*control+0.8f*old_control;
  diff=(position[0]-position[1]);
  if (diff>1) diff=1;
  else if (diff<-1) diff=-1;
  
  //if (new_control>99.5f-add) new_control=99.5f-add;
  //else if (new_control<-99.5f+add) new_control=-99.5f+add;
  //if (new_control*old_control<0.0f){
  //  new_control=0;
  //}
  
  if (new_control>delta){
    
    TIM4->CCR3=(uint16_t)(0xFFFF*0.01f*(new_control+add-diff)); 
    TIM4->CCR4=0;
    TIM9->CCR1=(uint16_t)(0xFFFF*0.01f*(1.05f*new_control+add+diff)); 
    TIM9->CCR2=0;
  }
  else if (new_control<-delta){
    TIM4->CCR3=0; 
    TIM4->CCR4=(uint16_t)(0xFFFF*0.01f*(-new_control+add+diff)); 
    TIM9->CCR1=0; 
    TIM9->CCR2=(uint16_t)(0xFFFF*0.01f*(-1.05f*new_control+add-diff)); 
  }
  else {
    TIM4->CCR3=0; 
    TIM4->CCR4=0; 
    TIM9->CCR1=0; 
    TIM9->CCR2=0; 
  }

  old_control=new_control;
}

int ParsePacket(void)
{
  uint8_t hash=0;
  for (uint8_t i=3; i<RECEIVE_PACKET_LENGTH; i++) hash^=BufferPull(i);
  uint8_t asd= BufferPull(2);
  if (hash==BufferPull(2)) 
  {
    motor_packet_1[0]=BufferPull(RECEIVE_PACKET_LENGTH-1);
    motor_packet_2[0]=BufferPull(RECEIVE_PACKET_LENGTH-2);
    /*
    for (uint8_t i=0; i<4;i++)
    {
      motor_packet_1[i]=BufferPull(RECEIVE_PACKET_LENGTH-1-i);
      motor_packet_2[i]=BufferPull(RECEIVE_PACKET_LENGTH-5-i);
    }
    */
    flag_received_packet=1;
    return 1;
  }
  return 0;
}

float ComputeControl()
{
  float center[4]={0,0,0,0};
  float radius[4]={M_PI, M_PI, M_PI/6.0f, M_PI/2.0f};

  /*
  sysState[0]=0;
  sysState[1]=0;
  sysState[2]=-1.5547784044E-2;
  sysState[3]=3.1203120011E-3;
  */
  int max_depth=30;
  int axis=0;
  uint32_t element=0;
  uint32_t address=0;
  
  time_end1=0;
  time_start=__HAL_TIM_GET_COUNTER(&htim5);
  
  for (int depth=0; depth<max_depth; depth++){
    element=locationMatrix[address];
    axis = (element>>28)-1;
    address = (element & 0x0FFFFFFF)-1; 
    if (axis>=0){
      radius[axis]=radius[axis]/2.0f;
      if (sysState[axis]>center[axis]){
        address=address+1;
        center[axis]=center[axis]+radius[axis];
      }else{
        address=address;
        center[axis]=center[axis]-radius[axis];
      }
    }else break;
  }
  
  time_end1=__HAL_TIM_GET_COUNTER(&htim5)-time_start;
  time_end2=0;
  
  for(int i=0; i<dim_x; i++){
    stateInBox[i]=(sysState[i]-center[i]+radius[i])/(2.0f*radius[i]);
  }
  
  int sign = 1-2*((element & 0x2000)>>13);
  int flip = ((element & 0x8000000)>>27);
  
  uint32_t controlAddress=(element & 0x1FFF)-1;
  uint32_t costAddress=((element>>14) & 0x1FFF)-1;
  
  float control=0;
  
  if (costAddress==0){
    for (int i=0; i<dim_x; i++){
      control+=lqr[i]*sysState[i];
    }
  }
  else{
    time_start=__HAL_TIM_GET_COUNTER(&htim5);
    
    PrepareLP(costAddress,flip);
    linprog_solve();
    if (flip){
      for (int i=0; i<dim_lambda; i++){
        control+=y[i]*(float)controlMatrix[controlAddress][dim_lambda-i-1];
      }
    }else{
      for (int i=0; i<dim_lambda; i++){
        control+=y[i]*(float)controlMatrix[controlAddress][i];
      }
    }
    control=sign*control*0.2745f; //from 255 to 70
    time_end2=__HAL_TIM_GET_COUNTER(&htim5)-time_start;
  }
  if (control>70.0f) {control=70.0f;}
  else if (control<-70.0f) {control=-70.0f;}
  
  return control;
}

void PrepareLP(uint32_t costAddress, int flip)
{
  for (int j=0; j<dim_y;j++){
    if (j<dim_lambda){
      if (flip) cost[j]=costMatrix[costAddress][dim_lambda-j-1];
      else cost[j]=costMatrix[costAddress][j];
      A[0][j]=1.0f;
      N[j]=j;
    }else{
      cost[j]=2000.0f;
      A[0][j]=0.0f;
      B[j-dim_lambda]=j;
      N[j]=-1;
    }
  }
  
  cost[dim_y]=0.0f;
  A[0][dim_y]=1.0f;
  A[0][dim_lambda]=1.0f;
 
  for (int i=1; i<num_con;i++){
    for (int j=0; j<dim_y; j++){
      if (j<dim_lambda){
        A[i][j]=(float)vertices[i-1][j];
      }else{
        if (i==j-dim_lambda){
          A[i][j]=1.0f;
        }else{
          A[i][j]=0.0f;
        }
      }
    }
    A[i][dim_y]=stateInBox[i-1];
  }
}

void EstimateState10ms()
{
  static float pos_prev=0;
  static float velo=0;
  static float velo_pred=0;
  float pos=(position[0]+position[1])*3.1416f/520;
  sysState[0]=sysState[0]+(pos-pos_prev);
  pos_prev=pos;
  if (sysState[0]>3.14f){
    sysState[0]=3.14f;
  }else if (sysState[0]<-3.14f){
    sysState[0]=-3.14f;
  }
  //sysState[0]=(position[0]+position[1])*3.1416f/520;

  //sysState[1]=0.94f*sysState[1]+0.023227f*control;
  //velo_pred = 0.9214f*sysState[1]-0.6f*sysState[2]-0.003f*sysState[3]+0.0245f*control;
  velo = 0.5f*velo+ 0.5f*(velocity[0]+velocity[1])*3.1416f/520;
  //velo=(velocity[0]+velocity[1])*3.1416f/520;
  sysState[1]=velo;
  if (sysState[1]>3.14f){
    sysState[1]=3.14f;
  }else if (sysState[1]<-3.14f){
    sysState[1]=-3.14f;
  }
}

void EstimateState1ms()
{  
  //sysState[3]=0.022f*sysState[2] + 1.5e-04f*gyro_16[0];
  float gyro=0;
  gyro=1.331601e-04f*(gyro_16[0]+gyro_16[1])*0.707107f;
  theta=theta+gyro*0.001f;//1.3316e-04f
  float acc=0;
  
  acc=(accel_16[0]+accel_16[1])*0.707107f;
  theta_acc=(atan2f(acc,accel_16[2]))+2.6E-2f;
  theta=0.999f*theta+0.001f*theta_acc;

    
  //theta_bias=0.999f*theta_bias+0.001f*theta_acc;
  
  
  //sysState[2]=1.0011f*theta + 0.01f*gyro;
  //sysState[3]=0.2219f*theta + 1.0011f*gyro;
  //sysState[2]=1.0001f*theta + 0.003f*gyro;
  //sysState[3]=0.0672f*theta + 1.0003f*gyro;
  
  sysState[2]=theta;
  sysState[3]=gyro;
}


float ComputeLQR()
{
  float control=0;
  for (int i=0; i<dim_x; i++){
    control+=lqr[i]*sysState[i];
  }
  if (control>70) {control=70;}
  else if (control<-70) {control=-70;}
  
  return control;
}



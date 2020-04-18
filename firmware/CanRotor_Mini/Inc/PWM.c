//#include "PWM.h"
#include "Board.h"

uint32_t time_manual_motor;

void PwmWriteMotor(void)
{
  if(micros() - time_manual_motor >= 500000){
    Manual_Motor_flag = false;
  }
  if(Manual_Motor_flag == true){
    TIM4->CCR1 = M_motor[0];  // Actual : REAR_L
    TIM4->CCR2 = M_motor[1];  // Actual : FRONT_R
    TIM4->CCR3 = M_motor[2];  // Actual : FRONT_L
    TIM4->CCR4 = M_motor[3];  // Actual : REAR_R
  }else{
    TIM4->CCR1 = motor[0];  // Actual : REAR_L
    TIM4->CCR2 = motor[1];  // Actual : FRONT_R
    TIM4->CCR3 = motor[2];  // Actual : FRONT_L
    TIM4->CCR4 = motor[3];  // Actual : REAR_R
  }
}

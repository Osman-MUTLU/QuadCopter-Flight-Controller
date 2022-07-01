#include "Wire.h"

/* ARDUINO NANO QUADCOPTER SCHEMA https://electronoobs.com/images/Robotica/tut_5/flight_controller_schematic.png */

/* RECEIVER values*/
unsigned long timer_1, timer_2, timer_3, timer_4,timer_5,timer_6, current_time;
int receiver_input[7];
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4,last_channel_5, last_channel_6;
int receiver_throttle,receiver_yaw,receiver_pitch,receiver_roll,receiver_aux1,receiver_aux2;

/* Timers */ 
unsigned long loop_timer;
unsigned long timer_motor_1, timer_motor_2, timer_motor_3, timer_motor_4, esc_timer, esc_loop_timer;
int timer;
/* Motors*/
int motor_1_pow =0;
int motor_2_pow =0;
int motor_3_pow =0;
int motor_4_pow =0;
int throttle;
int minThrottle = 1200;
boolean start = false;
/*  GYRO VARIABLES MPU6050  */
long gyroX , gyroY, gyroZ;
int cal_int;
float rotX, rotY, rotZ;
float calX,calY,calZ;
float gyro_pitch_input,gyro_yaw_input,gyro_roll_input;
int counter = 0;

/* ROLL PID */

float pid_roll_kp=1.3; //DEFAULT 1.3
float pid_roll_ki=0.05; //DEFAULT 0.05
float pid_roll_kd=15;  //DEFAULT 15

/* PITCH PID */

float pid_pitch_kp=1.3; //DEFAULT 1.3
float pid_pitch_ki=0.05; //DEFAULT 0.05
float pid_pitch_kd=15;  //DEFAULT 15

/* YAW PID */

float pid_yaw_kp=4.0; //DEFAULT 4.0
float pid_yaw_ki=0.02; //DEFAULT 0.02
float pid_yaw_kd=0;  //DEFAULT 0

/* PID VARIABLES */
float pid_roll_error,pid_roll_error_prev,pid_roll_error_mem;
float pid_roll_output;
float pid_roll_setpoint=0;
float pid_pitch_error,pid_pitch_error_prev,pid_pitch_error_mem;
float pid_pitch_output;
float pid_pitch_setpoint=0;
float pid_yaw_error,pid_yaw_error_prev,pid_yaw_error_mem;
float pid_yaw_output;
float pid_yaw_setpoint=0;


void setup() {
  /*SENSOR SIGNAL*/
  Wire.begin();
  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz.
  /*SERIAL MONITOR*/
  Serial.begin(9600);
  /* RECEIVER SIGNALS */
  PCICR |= (1 << PCIE2);                                                    //Set PCIE0 to enable PCMSK2 scan.
  PCMSK2 |= (1 << PCINT21);                                                  //Set PCINT0 (digital input 5) to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT20);                                                  //Set PCINT1 (digital input 4)to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT18);                                                  //Set PCINT2 (digital input 2)to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT22);                                                  //Set PCINT3 (digital input 6)to trigger an interrupt on state change.
  PCMSK2 |= (1 << PCINT23);                                                  //Set PCINT0 (digital input 7) to trigger an interrupt on state change.
  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                   //Set PCINT0 (digital input 8) to trigger an interrupt on state change.

  /* ARDUINO MOTOR CONTROL WITH ESC https://www.youtube.com/watch?v=uOQk8SJso6Q */
  DDRD |= B00001000;//Configure digital poort 3 as output.
  DDRB |= B00001110;   //Configure digital poort 9 , 10 and 11 as output.
  
  for (timer = 0; timer < 1250 ; timer ++){                           //Wait 5 seconds before continuing.
    PORTD |= B00001000;                                                     //Set digital poort 3 high.
    PORTB |= B00001110;                                                     //Set digital poort 9, 10 and 11 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B11110111;                                                     //Set digital poort 3 low.
    PORTB &= B11110001;                                                     //Set digital poort 9, 10 and 11 low.
    delayMicroseconds(3000);                                                //Wait 3000us.
  }
  setupMPU();
  read_pwm();
  //Wait until the receiver is active and the throtle is set to the lower position.
  while(!(receiver_throttle > 990 && receiver_throttle < 1070)){
    PORTD |= B00001000;                                                     //Set digital poort 3 high.
    PORTB |= B00001110;                                                     //Set digital poort 9, 10 and 11 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B11110111;                                                     //Set digital poort 3 low.
    PORTB &= B11110001;                                                     //Set digital poort 9, 10 and 11 low.
    delay(3);
  }
  start=true;
  loop_timer = micros(); 
}

void loop() {
  recordGyroRegisters();
  read_pwm();
  // printMPU();
  // printReceiver();
  if(receiver_throttle<1100 && receiver_yaw<1100) start=true;
  if(start == true && receiver_throttle < 1100 && receiver_yaw > 1900)start = false;
  if(start){
    pid_roll_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if(receiver_roll > 1508)pid_roll_setpoint = receiver_roll - 1508;
    else if(receiver_roll < 1492)pid_roll_setpoint = receiver_roll - 1492;
    pid_roll_setpoint /= 3.0;  //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.
    
    pid_pitch_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if(receiver_pitch > 1508)pid_pitch_setpoint = receiver_pitch - 1508;
    else if(receiver_pitch < 1492)pid_pitch_setpoint = receiver_pitch - 1492;
    pid_pitch_setpoint /= 3.0;  //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.
    
    pid_yaw_setpoint = 0;
    //We need a little dead band of 16us for better results.
    if(receiver_throttle > 1100){ //Do not yaw when turning off the motors.
      if(receiver_yaw > 1508)pid_yaw_setpoint = (receiver_yaw - 1508)/3.0;
      else if(receiver_yaw < 1492)pid_yaw_setpoint = (receiver_yaw - 1492)/3.0;
    }
    PID();
    throttle = receiver_throttle;
  }
  Motor_Control_Algorithm();
}


/* RECEIVER READING */
/* Pin Change Interruptins ISR for reading PWM signals https://www.youtube.com/watch?v=ZDtRWmBMCmw */
void printReceiver(){
  Serial.print(" PITCH = ");
  Serial.print(receiver_input[1]); // CH1 Roll
  Serial.print("    ROLL = ");
  Serial.print(receiver_input[2]); // CH2 Pitch
  Serial.print("    Throttle = "); 
  Serial.print(receiver_input[3]); // CH3 Throttle
  Serial.print("    YAW = ");
  Serial.print(receiver_input[4]); // CH4 Yaw
  Serial.print("    AUX1 = ");
  Serial.print(receiver_input[5]); // CH5 Aux1
  Serial.print("    AUX2 = ");
  Serial.print(receiver_input[6]); // CH6 Aux2
  Serial.println();
}
void read_pwm(){
  if(receiver_input[1]<1000) receiver_input[1] = 1000;
  else if(receiver_input[1]>2000) receiver_input[1] = 2000;
  if(receiver_input[2]<1000) receiver_input[2] = 1000;
  else if(receiver_input[2]>2000) receiver_input[2] = 2000;
  if(receiver_input[3]<1000) receiver_input[3] = 1000;
  else if(receiver_input[3]>2000) receiver_input[3] = 2000;
  if(receiver_input[4]<1000) receiver_input[4] = 1000;
  else if(receiver_input[4]>2000) receiver_input[4] = 2000;
  if(receiver_input[5]<1000) receiver_input[5] = 1000;
  else if(receiver_input[5]>2000) receiver_input[5] = 2000;
  if(receiver_input[6]<1000) receiver_input[6] = 1000;
  else if(receiver_input[6]>2000) receiver_input[6] = 2000;
  receiver_roll=receiver_input[1]; //Roll ch1
  receiver_pitch=receiver_input[2]; //Pitch ch2
  receiver_throttle=receiver_input[3]; //Throttle ch3
  receiver_yaw=receiver_input[4]; //Yaw ch4
  receiver_aux1=receiver_input[5]; //Aux1 ch5
  receiver_aux2=receiver_input[6]; //Aux2 ch6
}
ISR(PCINT2_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PIND & B00010000){                                                     //Is input 4 high? Roll channel 1
    if(last_channel_1 == 0){                                                //Input 4 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 4 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(PIND & B00100000 ){                                                    //Is input 5 high?  Pitch channel 2
    if(last_channel_2 == 0){                                                //Input 5 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                             //Input 5 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if(PIND & B00000100 ){                                                    //Is input 2 high?  Throttle channel 3
    if(last_channel_3 == 0){                                                //Input 2 changed from 0 to 1. 
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             //Input 2 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if(PIND & B01000000 ){                                                    //Is input 6 high? Yaw channel 4
    if(last_channel_4 == 0){                                                //Input 6 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             //Input 6 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[4] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
  //Channel 5=========================================
  if(PIND & B10000000 ){                                                    //Is input 7 high? Aux1 channel 5
    if(last_channel_5 == 0){                                                //Input 7 changed from 0 to 1.
      last_channel_5 = 1;                                                   //Remember current input state.
      timer_5 = current_time;                                               //Set timer_5 to current_time.
    }
  }
  else if(last_channel_5 == 1){                                             //Input 7 is not high and changed from 1 to 0.
    last_channel_5 = 0;                                                     //Remember current input state.
    receiver_input[5] = current_time - timer_5;                             //Channel 5 is current_time - timer_5.
  }
}
ISR(PCINT0_vect){
  //Channel 6=========================================
  if(PINB & B00000001 ){                                                    //Is input 8 high? Aux2 channel 6
    if(last_channel_6 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_6 = 1;                                                   //Remember current input state.
      timer_6 = current_time;                                               //Set timer_6 to current_time.
    }
  }
  else if(last_channel_6 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_6 = 0;                                                     //Remember current input state.
    receiver_input[6] = current_time - timer_6;                             //Channel 6 is current_time - timer_6.
  }
}

/* MOTOR MIXING ALGORTIHM https://www.youtube.com/watch?v=hGcGPUqB67Q&list=PLPNM6NzYyzYqMYNc5e4_xip-yEu1jiVrr&index=1 */ 
void Motor_Control_Algorithm(){
  if (throttle > 1800) throttle = 1800;
  // Arka poz m3,m4 + pitch
  // Sağ poz 2,3 + roll
  // Saatin tersi poz 1,3 + yaw
  if(start){
    motor_1_pow = throttle - pid_roll_output - pid_pitch_output + pid_yaw_output;
    motor_2_pow = throttle + pid_roll_output - pid_pitch_output - pid_yaw_output;
    motor_3_pow = throttle + pid_roll_output + pid_pitch_output + pid_yaw_output;
    motor_4_pow = throttle - pid_roll_output + pid_pitch_output - pid_yaw_output;
     if(motor_1_pow>2000){
      motor_1_pow=2000;
    }
    else if(motor_1_pow<minThrottle){
      motor_1_pow=minThrottle;
    }
    
    if(motor_2_pow>2000){
      motor_2_pow=2000;
    }
    else if(motor_2_pow<minThrottle){
      motor_2_pow=minThrottle;
    }
    
    if(motor_3_pow>2000){
      motor_3_pow=2000;
    }
    else if(motor_3_pow<minThrottle){
      motor_3_pow=minThrottle;
    }
    
    if(motor_4_pow>2000){
      motor_4_pow=2000;
    }
    else if(motor_4_pow<minThrottle){
      motor_4_pow=minThrottle;
    }
  }
  else{
    motor_1_pow=1000;
    motor_2_pow=1000;
    motor_3_pow=1000;
    motor_4_pow=1000;
  }
  //Creating the pulses for the ESC's is explained in this video:
  //https://youtu.be/fqEkVcqxtU8
  while(micros() - loop_timer < 4000); 
  loop_timer = micros(); 

  PORTD |= B00001000;                                                     //Set digital poort 3 high.
  PORTB |= B00001110;                                                     //Set digital poort 9, 10 and 11 high.
  timer_motor_1 = motor_1_pow + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_motor_2 = motor_2_pow + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_motor_3 = motor_3_pow + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_motor_4 = motor_4_pow + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  recordGyroRegisters();
  boolean esc_1=false,esc_2=false,esc_3=false,esc_4=false;
  while(true){
    //Stay in this loop until output 3,9,10 and 11 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_motor_1 <= esc_loop_timer){
      PORTD &= B11110111;                //Set digital output 3 to low if the time is expired.
      esc_1=true;
    }
    if(timer_motor_2 <= esc_loop_timer){
      PORTB &= B11110111;                //Set digital output 9 to low if the time is expired.
      esc_2=true;
    }
    if(timer_motor_3 <= esc_loop_timer){
      PORTB &= B11111011;                //Set digital output 10 to low if the time is expired.
      esc_3=true;
    }
    if(timer_motor_4 <= esc_loop_timer){
      PORTB &= B11111101;                //Set digital output 11 to low if the time is expired.
      esc_4=true;
    }
    if(esc_1 && esc_2 && esc_3 && esc_4) break;
  }  
}

/* PID Control video: (YMFC-3D by Joop Brokking) https://www.youtube.com/watch?v=JBvnB0279-Q */
void PID(){
  /* ROLL PID */
  pid_roll_error = gyro_roll_input - pid_roll_setpoint;
  pid_roll_error_mem += pid_roll_error;
  if(pid_roll_error_mem >400) pid_roll_error_mem = 400;
  if(pid_roll_error_mem <-400) pid_roll_error_mem = -400;
  pid_roll_output = pid_roll_error*pid_roll_kp + pid_roll_error_mem * pid_roll_ki + (pid_roll_error - pid_roll_error_prev)*pid_roll_kd;
  if(pid_roll_output >400) pid_roll_output=400;
  if(pid_roll_output <-400) pid_roll_output=-400;
  pid_roll_error_prev = pid_roll_error;

  /* PITCH PID */
  pid_pitch_error = gyro_pitch_input - pid_pitch_setpoint;
  pid_pitch_error_mem += pid_pitch_error;
  if(pid_pitch_error_mem >400) pid_pitch_error_mem = 400;
  if(pid_pitch_error_mem <-400) pid_pitch_error_mem = -400;
  pid_pitch_output = pid_pitch_error*pid_pitch_kp + pid_pitch_error_mem * pid_pitch_ki + (pid_pitch_error - pid_pitch_error_prev)*pid_pitch_kd;
  if(pid_pitch_output >400) pid_pitch_output=400;
  if(pid_pitch_output <-400) pid_pitch_output=-400;
  pid_pitch_error_prev = pid_pitch_error;

  /* YAW PID */
  pid_yaw_error = gyro_yaw_input - pid_yaw_setpoint;
  pid_yaw_error_mem += pid_yaw_error;
  if(pid_yaw_error_mem >400) pid_yaw_error_mem = 400;
  if(pid_yaw_error_mem <-400) pid_yaw_error_mem = -400;
  pid_yaw_output = pid_yaw_error*pid_yaw_kp + pid_yaw_error_mem * pid_yaw_ki + (pid_yaw_error - pid_yaw_error_prev)*pid_yaw_kd;
  if(pid_yaw_output >400) pid_yaw_output=400;
  if(pid_yaw_output <-400) pid_yaw_output=-400;
  pid_yaw_error_prev = pid_yaw_error;

}
  
void printMPU(){
  counter++;
  if(counter%100==0){
    Serial.print("PITCH =   ");
    Serial.print(gyro_pitch_input);// Ön negatif(-), Arka pozitif(+)
    Serial.print("    Roll =   ");
    Serial.print(gyro_roll_input); // Sol negatif(-), Sağ pozitif(+)
    Serial.print("    Z =   ");
    Serial.print(gyro_yaw_input); // Saat yönü negatif(-), Saatin tersi (+)
    Serial.println();
    counter=0;
  }
}

/* MPU650 READING DATA AND CALCULATING ERRORS https://www.youtube.com/watch?v=UxABxSADZ6U  */
void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
  Serial.print("Reading MPU 6050 .");
  for(cal_int = 0;cal_int<2000;cal_int++){
    recordGyroRegisters();
    if(cal_int %200 == 0)Serial.print(".");
    calX += rotX;
    calY += rotY; 
    calZ += rotZ;
    PORTD |= B00001000;                                                     //Set digital poort 3 high.
    PORTB |= B00001110;                                                     //Set digital poort 9, 10 and 11 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B11110111;                                                     //Set digital poort 3 low.
    PORTB &= B11110001;                                                     //Set digital poort 9, 10 and 11 low.
    delay(3);
  }
  Serial.println();
  calX /= 2000;
  calY /= 2000;
  calZ /= 2000;
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
  if(cal_int == 2000){
    rotX -= calX;
    rotY -= calY; 
    rotZ -= calZ;
  }
}
void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((rotX / 65.5) * 0.3);
  gyro_roll_input = (gyro_roll_input * 0.7) + ((rotY / 65.5) * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((rotZ / 65.5) * 0.3);
}

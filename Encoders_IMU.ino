
//#include "stm32f4xx.h"
//#include "stm32f4xx_hal.h"


/*
#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
std_msgs::Float64MultiArray speed_msg;
sensor_msgs::Imu imu_msg;
ros::Publisher Wheels("/autonomous_steer_bot/wheel_speed", &speed_msg);
ros::Publisher Imu("/autonomous_steer_bot/imu/data", &imu_msg);
ros::NodeHandle nh;
*/


#include "variabili.h"

void fusion_update(void)
{
  fusion_flag = 1;
}


void count1() {
  counter1++;
}
void count2(){
  counter2++;
}
void count3() {
  counter3++;
}
void count4(){
  counter4++;
}

void WheelsSpeed(){
  double number_of_rounds1 = (double)counter1/fori;
  double number_of_rounds2 = (double)counter2/fori;
  double number_of_rounds3 = (double)counter3/fori;
  double number_of_rounds4 = (double)counter4/fori;

  double timeNow_sec = timer.getCount()/timerOverflow;
  
  double angular_speed1 = (number_of_rounds1)/(sample_time_sec+timeNow_sec);
  double angular_speed2 = (number_of_rounds2)/(sample_time_sec+timeNow_sec);
  double angular_speed3 = (number_of_rounds3)/(sample_time_sec+timeNow_sec);
  double angular_speed4 = (number_of_rounds4)/(sample_time_sec+timeNow_sec);
 
  speed_measurement[0] = angular_speed1 * PI * diameter;
  speed_measurement[1] = angular_speed2 * PI * diameter;
  speed_measurement[2] = angular_speed3 * PI * diameter;
  speed_measurement[3] = angular_speed4 * PI * diameter;

  test_wheel1.push(speed_measurement[0]);
  test_wheel2.push(speed_measurement[1]);
  test_wheel3.push(speed_measurement[2]);
  test_wheel4.push(speed_measurement[3]);

  speed_measurement[0]=test_wheel1.get();
  speed_measurement[1]=test_wheel2.get();
  speed_measurement[2]=test_wheel3.get();
  speed_measurement[3]=test_wheel4.get();

  
  counter1 = 0;
  counter2 = 0;
  counter3 = 0;
  counter4 = 0;
  
  timer.setCount(1);
  
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}







void setup() {
  /* Initialize Serial */
  Serial.begin(115200);
  pinMode(PB13,INPUT);
  pinMode(PB14,INPUT);
  pinMode(PB1,INPUT);
  pinMode(PB2,INPUT);
  
 
  
  while (!Serial) yield();

  /* Initialize LED */
  pinMode(LED_BUILTIN, OUTPUT);

  /* Initialize I2C bus */
  Wire.begin();
  Wire.setClock(400000);

  /* Start communication with IMU */
  AccGyr.begin();
  AccGyr.Set_X_ODR((float)ALGO_FREQ);
  AccGyr.Set_X_FS(4);
  AccGyr.Set_G_ODR((float)ALGO_FREQ);
  AccGyr.Set_G_FS(2000);
  AccGyr.Enable_X();
  AccGyr.Enable_G();
  delay(10);
  
  MotionFX_initialize((MFXState_t *)mfxstate);

  MotionFX_getKnobs(mfxstate, ipKnobs);

  ipKnobs->acc_orientation[0] = 'n';
  ipKnobs->acc_orientation[1] = 'w';
  ipKnobs->acc_orientation[2] = 'u';
  ipKnobs->gyro_orientation[0] = 'n';
  ipKnobs->gyro_orientation[1] = 'w';
  ipKnobs->gyro_orientation[2] = 'u';

  ipKnobs->gbias_acc_th_sc = GBIAS_ACC_TH_SC;
  ipKnobs->gbias_gyro_th_sc = GBIAS_GYRO_TH_SC;

  ipKnobs->output_type = MFX_ENGINE_OUTPUT_ENU;
  ipKnobs->LMode = 1;
  ipKnobs->modx = DECIMATION;

  MotionFX_setKnobs(mfxstate, ipKnobs);
  MotionFX_enable_6X(mfxstate, MFX_ENGINE_ENABLE);
  MotionFX_enable_9X(mfxstate, MFX_ENGINE_DISABLE);


  /* OPTIONAL */
  /* Get library version */
  LibVersionLen = (int)MotionFX_GetLibVersion(LibVersion);  

 
/*nh.initNode();
  speed_msg.data_length = 4; 
  nh.advertise(Wheels);
  nh.advertise(Imu);
  */
  MyTim = new HardwareTimer(TIM3);
  MyTim->setOverflow(ALGO_FREQ, HERTZ_FORMAT);
  MyTim->attachInterrupt(fusion_update);
  MyTim->resume();

  timer.setOverflow(Interrupt_FREQ, HERTZ_FORMAT);
  timer.attachInterrupt(WheelsSpeed);
  timer.refresh();
  timer.resume();
  
  attachInterrupt(PB13,count1,RISING);
  attachInterrupt(PB14,count2,RISING);
  attachInterrupt(PB1,count3,RISING);
  attachInterrupt(PB2,count4,RISING);

  AccGyr.Get_X_Axes(accelerometer);
  AccGyr.Get_G_Axes(gyroscope);

  //Valore di giroscopio iniziale
  angolo_prec=data_out.rotation[0];
  offset_x=data_out.rotation[0];
  offset_y=data_out.rotation[1];
  offset_z=data_out.rotation[2];
}








void loop() {
    if(fusion_flag)
    {
      fusion_flag = 0;
      deltat=millis();
      AccGyr.Get_X_Axes(accelerometer);
      AccGyr.Get_G_Axes(gyroscope);

      /* Convert angular velocity from [mdps] to [dps] */
      data_in.gyro[0] = (float)gyroscope[0] * FROM_MDPS_TO_DPS;
      data_in.gyro[1] = (float)gyroscope[1] * FROM_MDPS_TO_DPS;
      data_in.gyro[2] = (float)gyroscope[2] * FROM_MDPS_TO_DPS;

      /* Convert acceleration from [mg] to [g] */
      data_in.acc[0] = (float)accelerometer[0] * FROM_MG_TO_G;
      data_in.acc[1] = (float)accelerometer[1] * FROM_MG_TO_G;
      data_in.acc[2] = (float)accelerometer[2] * FROM_MG_TO_G;

      /* Don't set mag values because we use only acc and gyro */
      data_in.mag[0] = 0.0f;
      data_in.mag[1] = 0.0f;
      data_in.mag[2] = 0.0f; 
       
      if (discardedCount == sampleToDiscard)
      {
        MotionFX_propagate(mfxstate, &data_out, &data_in, &delta_time);
        MotionFX_update(mfxstate, &data_out, &data_in, &delta_time, NULL);
      }
      else
      {
        discardedCount++;
      }

      
     
      

      
    }
    /*
       data_out.rotation[0]= data_out.rotation[0]-offset_x;
       data_out.rotation[1]= data_out.rotation[1]-offset_y;
       data_out.rotation[2]= data_out.rotation[2]-offset_z;
  */
    
        
       //Variazione di angolo in modulo e convertita in radianti
       deltaangolo=(abs(angolo_prec - data_out.rotation[0]))* DEG_TO_RAD;
       //l'angolo precedente diventa quello appena letto
       angolo_prec=data_out.rotation[0];
       //variazione di tempo
       deltat=millis()-deltat;
      //pubblica i dati sulla velocità
      /*
      speed_msg.data=speed_measurement;
      Wheels.publish( &speed_msg );  

      //imposta il messaggio con i dati dell'IMU
      imu_msg.orientation.z=data_out.rotation[0];
      imu_msg.orientation.x=data_out.rotation[1];
      imu_msg.orientation.y=data_out.rotation[2];
      imu_msg.orientation.w=0;
      imu_msg.linear_acceleration.x=data_out.linear_acceleration[0];
      imu_msg.linear_acceleration.y=data_out.linear_acceleration[1];
      imu_msg.linear_acceleration.z=data_out.linear_acceleration[2];

      Imu.publish(&imu_msg);
      
      nh.spinOnce();
*/
      
      Serial.print(speed_measurement[0]); //motor_speed (m/s)
      Serial.print(" , ");
      Serial.print(speed_measurement[1]); //motor_speed (m/s)
      Serial.print(" , ");
      Serial.print(speed_measurement[2]); //motor_speed (m/s)
      Serial.print(" , ");
      Serial.print(speed_measurement[3]); //motor_speed (m/s)
      Serial.print(" , ");  
      Serial.print(data_out.rotation[0] * DEG_TO_RAD); //yaw (rad)
      Serial.print(" , ");
      Serial.print(data_out.rotation[1] *DEG_TO_RAD); //pitch (rad)
      Serial.print(" , ");
      Serial.print(data_out.rotation[2] *DEG_TO_RAD); //roll (rad)
      Serial.print(" , ");
      Serial.print(deltaangolo/(deltat/1000)); //angular speed (rad/s)
      Serial.print(" , ");
      Serial.print(data_out.linear_acceleration[0] * G_TO_ACCEL); //acc_x (m/s^2)
      Serial.print(" , ");
      Serial.print(data_out.linear_acceleration[1] * G_TO_ACCEL); //acc_y (m/s^2)
      Serial.print(" , ");
      Serial.println(data_out.linear_acceleration[2] * G_TO_ACCEL); //acc_z (m/s^2)
  
 
}

/*
   rosserial Subscriber Example
   Blinks an LED on callback
*/

#include <ros.h>
#include <std_msgs/String.h>
#include<std_msgs/UInt16.h>
#include<std_msgs/UInt8.h>
#include <VL53L0X.h>

VL53L0X sensor;

#define motor2_CW 5
#define motor2_CCW 6

#define motor1_CW 7
#define motor1_CCW 8

#define motor3_CW 9
#define motor3_CCW 10


#define db1_cw 20
#define db1_ccw 21 // yg dipake

#define db2_cw 22 //yg dipake
#define db2_ccw 23


float radian30 = 0.523599; // alpha 1
float radian90 = 1.5708; //   phi/2
float radian150 = 2.61799; // alpha 2
float radian270 = 4.71239; // alpha 3

float a = sin (radian30 + radian90);  //m11
float b = sin (radian150 + radian90); //m12
float c = sin (radian270 + radian90); //m13
float d = cos (radian30 + radian90);  //m21
float e = cos (radian150 + radian90); //m22
float f = cos (radian270 + radian90); //m23

int g = 1; //m31
int h = 1; //m32
int i = 1; //m33

int f1CW;
int f1CCW;

int f2CW;
int f2CCW;

int f3CW;
int f3CCW;

int db1CW;
int db1CCW;

int db2CW;
int db2CCW;

int TOF_Read;

float ax;
float ay;
float w;

double dett = (b*f*g) + (e*a*i) + (c*d*h) - (a*f*h) - (b*d*i) - (e*c*g);
double mult = 1 / dett;
double force1;
double force2;
double force3;

ros::NodeHandle  nh;
float dataax = 0;
float dataay = 0;
float dataaw = 0;
String isi_multi_data_c;
void baca_multi_data( const std_msgs::String& isi_multi_data) {

  isi_multi_data_c = (isi_multi_data.data);
  dataax = (getValue(isi_multi_data_c, ' ', 0)).toFloat();
  dataay = (getValue(isi_multi_data_c, ' ', 1)).toFloat();
  dataaw = (getValue(isi_multi_data_c, ' ', 2)).toFloat();

  //  digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN)); // blink the led
}

uint8_t isi;
void baca_grip( const std_msgs::UInt8& isi_grip){
isi = (isi_grip.data);
 //tangkap bola
  if(isi == 1){
    db1CCW = 255 * 0.7;
    db2CW = 255 * 0.7 ;
  }
  //berhenti
  if(isi == 0){
    db1CCW = 0;
    db2CW = 0;
  }
}

void mathematical() {
  ax = dataax;
  ay = dataay;
  w = dataaw;

  force1 = ((((e - f) * mult) * ax) + (((c - b) * mult) * ay) + ((((b * f) - (e * c)) * mult) * w));
  force2 = ((((-d + f) * mult) * ax) + (((a - c) * mult) * ay) + (((-(a * f) + (c * d)) * mult) * w));
  force3 = ((((d - e) * mult) * ax) + (((b - a) * mult) * ay) + ((((e * a) - (b * d)) * mult) * w));

  if (force1 >= 0) {
    f1CW = map(force1, 0, 0.91, 0, 255);
    f1CCW = 0;
  }

  if (force1 < 0) {
    f1CW = 0;
    f1CCW = map(force1, 0, -0.91, 0, 255);
  }

  if (force2 >= 0) {
    f2CW = map(force2, 0, 0.91, 0, 255);
    f2CCW = 0;
  }

  if (force2 < 0) {
    f2CW = 0;
    f2CCW = map(force2, 0, -0.91, 0, 255);
  }

  if (force3 >= 0) {
    f3CW = map(force3, 0, 0.91, 0, 255);
    f3CCW = 0;
  }

  if (force3 < 0) {
    f3CW = 0;
    f3CCW = map(force3, 0, -0.91, 0, 255);
  }
  //f1CW = map(force1, 0, 0.91, 0, 255);
  //f1CCW = map(force1, 0, -0.91, 0, 255);

  //f2CW = map(force2, 0, 0.91, 0, 255);
  //f2CCW = map(force2, 0, -0.91, 0, 255);

  //f3CW = map(force3, 0, 0.91, 0, 255);
  //f3CCW = map(force3, 0, -0.91, 0, 255);
}

std_msgs::UInt16 tof_data;

ros::Subscriber<std_msgs::String> sub("topic_multi_data", &baca_multi_data);
ros::Subscriber<std_msgs::UInt8> sub_grip("topic_grip", &baca_grip);
ros::Publisher pub_tof("topic_sensor_tof", &tof_data);

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void setup()
{
   //Serial.begin(57600);
  pinMode(LED_BUILTIN, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(sub_grip);
  nh.advertise(pub_tof);
  db1CW = 0;
  db1CCW = 0;
  db2CW = 0;
  db2CCW = 0;
  Wire.begin();

  pinMode(motor1_CW, OUTPUT);
  pinMode(motor1_CCW, OUTPUT);

  pinMode(motor2_CW, OUTPUT);
  pinMode(motor2_CCW, OUTPUT);

  pinMode(motor3_CW, OUTPUT);
  pinMode(motor3_CCW, OUTPUT);

  pinMode(db1_cw, OUTPUT);
  pinMode(db2_ccw, OUTPUT);

  pinMode(db1_cw, OUTPUT);
  pinMode(db2_ccw, OUTPUT);
  sensor.setTimeout(500);
  sensor.init();

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif
}

void loop()
{
  //mathematical();
  analogWrite(motor1_CCW, f1CCW);
  analogWrite(motor1_CW, f1CW);
  analogWrite(motor2_CCW, f2CCW);
  analogWrite(motor2_CW, f2CW);
  analogWrite(motor3_CCW, f3CCW);
  analogWrite(motor3_CW, f3CW);
  analogWrite(db1_cw, db1CW);
  analogWrite(db1_ccw, db1CCW);
  analogWrite(db2_cw, db2CW);
  analogWrite(db2_ccw, db2CCW);
  
  nh.spinOnce();
  delay(1);
  
//  TOF_Read = sensor.readRangeSingleMillimeters();
//  tof_data.data = TOF_Read;
//  pub_tof.publish(&tof_data);
}

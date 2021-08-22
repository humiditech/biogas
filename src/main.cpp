#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "math.h"

char auth[] = "9ApOBzPB7cdq64dKnlK-9wpspKWOAZiQ";

const char * WIFI_SSID = "BISA";
const char * WIFI_PASS = "bayu1234";

#define MQ8_PIN     34
#define MG811_PIN   35
#define MQ4_PIN     26
#define RELAY_1_PIN 32
#define RELAY_2_PIN 33
#define RELAY_3_PIN 25

void pinInit();
int readHydrogen();
int readCO2(float volts, float *pcurve);
int readMethane();
void sendHydrogenVal(int value);
void sendCO2Val(int value);
void sendMethaneVal(int value);
float MGRead();
void sendCurrentValveControl();
float MQ8Calibration();
float MQ8ResistanceCalculation(int raw_adc);
float MQ8Read();
int MQ8GetGasPercentage(float rs_ro_ratio, int gas_id);
int  MQ8GetPercentage(float rs_ro_ratio, float *pcurve);

#define TURN_RELAY_1_ON   digitalWrite(RELAY_1_PIN, LOW)
#define TURN_RELAY_1_OFF  digitalWrite(RELAY_1_PIN, HIGH)
#define TURN_RELAY_2_ON   digitalWrite(RELAY_2_PIN, LOW)
#define TURN_RELAY_2_OFF  digitalWrite(RELAY_2_PIN, HIGH)
#define TURN_RELAY_3_ON   digitalWrite(RELAY_3_PIN, LOW)
#define TURN_RELAY_3_OFF  digitalWrite(RELAY_3_PIN, HIGH)

#define SEND_INTERVAL   2000
unsigned long prevMillis = 0;

#define MG811_DC_GAIN             (8.5)
#define MG811_ZERO_POINT_VOLTAGE  (0.220)
#define MG811_REACTION_VOLTAGE    (0.030)
float CO2Curve[3] = {2.602,MG811_ZERO_POINT_VOLTAGE,(MG811_REACTION_VOLTAGE/(2.602-3))};

#define MG811_READ_SAMPLE_TIMES     5
#define MG811_READ_SAMPLE_INTERVAL  50

#define MQ8_RL_VALUE                    (10)
#define MQ8_RO_CLEAN_AIR_FACTOR         (9.21)
#define MQ8_CALIBRATION_SAMPLE_TIMES    (50)
#define MQ8_CALIBRATION_SAMPLE_INTERVAL (500)
#define MQ8_READ_SAMPLE_TIMES           (5)
#define MQ8_READ_SAMPLE_INTERVAL        (50)
#define MQ8_GAS_H2                      (0)

float H2Curve[3] = {2.3, 0.93, -1.44};
float MQ8_Ro = 10;

bool automaticValveControl = false;

void setup() {
  Serial.begin(115200);

  pinInit();

  MQ8_Ro = MQ8Calibration();  

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting");
  while(WiFi.status() != WL_CONNECTED){
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println("Connected to WiFi");

  Blynk.begin(auth, WIFI_SSID, WIFI_PASS);
}

void loop() {
  Blynk.run();

  if(millis() - prevMillis >= SEND_INTERVAL){
    sendHydrogenVal(readHydrogen());
    sendCO2Val(readCO2(MGRead(), CO2Curve));
    sendMethaneVal(readMethane());
    sendCurrentValveControl();
  }

  if(!automaticValveControl){
    TURN_RELAY_3_OFF;
  }
  else {
    // TODO : Automatic valve control algorithm
  }


}

void pinInit(){
  pinMode(RELAY_1_PIN, OUTPUT);
  pinMode(RELAY_2_PIN, OUTPUT);
  pinMode(RELAY_3_PIN, OUTPUT);
}

int readHydrogen(){
  int hydrogenVal = MQ8GetGasPercentage(MQ8Read()/MQ8_Ro,MQ8_GAS_H2);

  return hydrogenVal;
}

int readCO2(float volts, float *pcurve){
  if((volts/MG811_DC_GAIN) >= MG811_ZERO_POINT_VOLTAGE){
    return -1;
  }
  else{
    return pow(10, ((volts/MG811_DC_GAIN) - pcurve[1]) / pcurve[2] + pcurve[0]);
  }
}

int readMethane(){
  int adcValue = analogRead(MQ4_PIN);
  float mvValue = (adcValue*3.3)/4096;
  int methaneVal = 10.938*exp(1.7742*mvValue);

  return methaneVal;
}

void sendHydrogenVal(int value){
  char sendBuf[10];
  sprintf(sendBuf, "%d ppm",value);

  Blynk.virtualWrite(V1, sendBuf);
}

void sendCO2Val(int value){
  char sendBuf[10];
  sprintf(sendBuf, "%d ppm",value);

  Blynk.virtualWrite(V2, sendBuf);
}

void sendMethaneVal(int value){
  char sendBuf[10];
  sprintf(sendBuf, "%d ppm",value);

  Blynk.virtualWrite(V3, sendBuf);
}

float MGRead(){
  float mvValue = 0.0;
  for(int i=0; i<MG811_READ_SAMPLE_TIMES; i++){
    mvValue += analogRead(MG811_PIN);
    delay(MG811_READ_SAMPLE_INTERVAL);
  }
  mvValue = (mvValue/MG811_READ_SAMPLE_TIMES) * 3.3 / 4096;

  return mvValue;
}

float MQ8Calibration()
{
  int i;
  float val=0;
 
  for (i=0;i<MQ8_CALIBRATION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQ8ResistanceCalculation(analogRead(MQ8_PIN));
    delay(MQ8_CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/MQ8_CALIBRATION_SAMPLE_TIMES;                   //calculate the average value
 
  val = val/MQ8_RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro 
                                                        //according to the chart in the datasheet 
 
  return val; 
}

float MQ8ResistanceCalculation(int raw_adc)
{
  return ( ((float)MQ8_RL_VALUE*(4096-raw_adc)/raw_adc));
}

float MQ8Read()
{
  int i;
  float rs=0;
 
  for (i=0;i<MQ8_READ_SAMPLE_TIMES;i++) {
    rs += MQ8ResistanceCalculation(analogRead(MQ8_PIN));
    delay(MQ8_READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/MQ8_READ_SAMPLE_TIMES;
 
  return rs;  
}
 
int MQ8GetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == MQ8_GAS_H2) {
     return MQ8GetPercentage(rs_ro_ratio,H2Curve);
  }  
  return 0;
}
 
int  MQ8GetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

BLYNK_WRITE(V4){
  if(param.asInt()) TURN_RELAY_1_ON;
  else TURN_RELAY_1_OFF;
}

BLYNK_WRITE(V5){
  if(param.asInt()) TURN_RELAY_2_ON;
  else TURN_RELAY_2_OFF;
}

BLYNK_WRITE(V6){
  if(param.asInt()) automaticValveControl = true;
  else automaticValveControl = false;
}

void sendCurrentValveControl(){
  char sendBuf[20];
  if(automaticValveControl) sprintf(sendBuf, "AUTOMATIC ON");
  else sprintf(sendBuf, "AUTOMATIC OFF");

  Blynk.virtualWrite(V7, sendBuf);
}





//download library "DallasTemperature" 在安裝時出現的OneWire也要一起安裝，不然找不到OneWire
//search "28BYJ-48" install "Unistep2"
#include <PubSubClient.h>
#include <LWiFi.h>
#include "Ultrasonic.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Unistep2.h>
/*===============================腳位規劃============================

//數位腳位D0-D13 (p0-p13)
//類比腳位A0-A3 (p13-p16)

// 馬達：D8 D9 D10 D11 (IN1, IN2, IN3, IN4), 總step數, 每步的延遲(in micros)
// 馬達旋轉一圈是360度，需要360/(5.635/64)=4096個脈衝信號，也就是說轉一圈總共是4096個step。
// 4聯繼電器：D4 D5 D6(IN1, IN2, IN3)
// 水溫：D1
// 超音波：D2 D3
// 水濁：A?D? A0
=====================================================================*/

Unistep2 stepper(8, 9, 10, 11, 4096, 1000);
Ultrasonic ultrasonic(2);
//Ultrasonic ultrasonic1(2); //D2
//Ultrasonic ultrasonic2(3); //D3
OneWire ds(1);

//水濁
//#define WaterDirty 480 //換水的濁度 //ad
#define WNTUpin A0 //水濁度的接腳  //add

//===============================MQTT============================

//topic
char Level_topic[] = "team11/level/data";
char LevelSetRange_topic[] = "team11/level/set_range";

char TempTurb_topic[] = "team11/temp_turb/data";
char TempSetMinRange_topic[] = "team11/temp/set_min_range";
char TempSetMaxRange_topic[] = "team11/temp/set_max_range";
char TurbSetRange_topic[] = "team11/turb/set_range";

char Switch_topic[] =  "team11/light_feed_pump/switch";
char State_topic[] =  "team11/tank_state/info";
char Flag_topic[] =  "team11/flag";

//connect
char client_Id[] = "team11";
char user_name[] = "iot2022";
char user_password[] = "12345678";

//server & port(外網)
char server[] = "140.127.196.119";
int port = 18311;



//===============================其他設定============================

//警示範位設定
float temp_max_range = 25;
float temp_min_range = 20;
float turb_max_range = 20;
int water_low = 12;

//wife
WiFiClient wifiClient;
int status = WL_IDLE_STATUS;
char ssid[] = "Galaxy A70B599";
char pass[] = "10241023";
PubSubClient client(wifiClient);


void callback(char* topic, byte* payload, unsigned int length) {
  char received;
  // byte -> char
  for (int i=0;i<length;i++)
    received = (char)payload[i];

  
  if(!strcmp(topic, Flag_topic)){
    
    //接收MQTT訊息(team11/flag)(flag == 1)(水溫水濁定時感測與分析)
    if(received == '1'){
      //取得資料
      float tu = turbidity_get();
      float te = getTemp();
  
      //傳送至MQTT
      String msgStr = "";
      char json[40];
      //json格式字串(需要有topic、temp、turb)
      msgStr = msgStr + "{\"topic\":" + TempTurb_topic + ", \"temp\":" + te + ", \"turb\":" + tu + "}";
      // 把String字串轉換成字元陣列格式
      msgStr.toCharArray(json, 40);
      // 發布MQTT主題與訊息
      client.publish(TempTurb_topic, json);
      
      //數據分析
      //水溫判斷 (temp ：11有問題、10無問題)
      if(temp_max_range < te || temp_min_range > te)
          client.publish(State_topic, "11");
      else
          client.publish(State_topic, "10");
          
      //水濁判斷(turb ：21有問題、20無問題)
      if(turb_max_range < tu)
        client.publish(State_topic, "21");
      else
        client.publish(State_topic, "20");
    }
    else if(received == '2'){
      //接收MQTT訊息(team11/flag)(flag == 2)(水位手動感測與分析)
      //取得資料
      float wcm = urs_get();
      String msgStr = "";
      char json[40];
      //json格式字串(需要有topic、level)
      msgStr = msgStr + "{\"topic\":" + Level_topic + ", \"level\":" + wcm + "}";
      // 把String字串轉換成字元陣列格式
      msgStr.toCharArray(json, 40);
      //發布至team11/level/data
      client.publish(Level_topic, json);
      
      //數據分析
      //水位判斷(level ：31有問題、30無問題)
      if(water_low < wcm)
        client.publish(State_topic, "31");
      else
        client.publish(State_topic, "30");
    }
   }


  //接收MQTT訊息(team11/tu/set_min_range)(水溫警示範圍變動)
  if(!strcmp(topic, TempSetMinRange_topic)){
    temp_min_range = (float)received;
  }
  
  //接收MQTT訊息(team11/temp/set_max_range)(水溫警示範圍變動)
  if(!strcmp(topic, TempSetMaxRange_topic)){
    temp_max_range = (float)received;
  }
  
  //接收MQTT訊息(team11/turb/set_range)(水濁警示範圍變動)
  if(!strcmp(topic, TurbSetRange_topic)){
    turb_max_range = (float)received;
  }

  //接收MQTT訊息(team11/level/set_range)(水位警示範圍變動)
  if(!strcmp(topic, LevelSetRange_topic)){
    water_low = (float)received;
  }

  //接收MQTT訊息(team11/light_feed_pump/switch)(開關控制)
  //(light:D4 feed:D5 pump:D6)
  if(!strcmp(topic, Switch_topic)){
    
    //(light)(flag = 11轉高電位 and flag = 10轉低電位)
    if(received == '11' || received == '10'){
      if(received == '11')
          digitalWrite(4, HIGH);
      else
          digitalWrite(4, LOW);
    }
    
    //(feed)(flag = 21轉高電位 and flag = 20轉低電位
    if(received == '21' || received == '20'){
      if(received == '21')
          digitalWrite(5, HIGH);
      else
          digitalWrite(5, LOW);
    }
    
    //(pump)(flag = 31轉高電位 and flag = 30轉低電位)
    if(received == '31' || received == '30'){
      if(received == '31')
          digitalWrite(6, HIGH);
      else
          digitalWrite(6, LOW);
    }
  }
 
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("MQTT connect...");
    // Attempt to connect
    if (client.connect(client_Id, user_name, user_password)) {
      // subscribe
      client.subscribe(Level_topic);
      client.subscribe(LevelSetRange_topic);
      client.subscribe(TempTurb_topic);
      client.subscribe(TempSetMaxRange_topic);
      client.subscribe(TempSetMinRange_topic);
      client.subscribe(TurbSetRange_topic);
      client.subscribe(Switch_topic);
      client.subscribe(State_topic);
      client.subscribe(Flag_topic);
    } 
    else {
      Serial.print("failed, rc=");
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
   Serial.println("MQTT connected success");
}

//setup Wifi
void setup_wifi() {  
  Serial.println("WiFi connected...");
  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
  }
  Serial.println("WiFi connected succes!!");
}

int urs_get(){
  // Get USR
  long distanceA=25.0;
  long distanceB;
  // Read URS(cm)
//  distanceA = ultrasonic1.MeasureInCentimeters();
//  distanceB = ultrasonic2.MeasureInCentimeters();
  distanceB = ultrasonic.MeasureInCentimeters();
  return distanceA-distanceB;
}

float turbidity_get(){
  //get  gravity arduino turbidty sensor
  //A0 = p14
  int sensorValue = analogRead(WNTUpin);// read the input on analog pin 0:
  float voltage = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  //return voltage;
  float ntu = 0;
  if(voltage < 2.5){
    ntu = 3000;
  }else{
    ntu = -1120.4*(voltage*voltage)+5742.3*voltage-4352.9;
  }
  return ntu;
}

float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius
  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  Serial.print("Temperature: ");
  Serial.print(String(TemperatureSum).c_str()); // print out the value you read:
  Serial.println(" 度C");

  //return TemperatureSum;
}

void stepper_run(){
  if ( stepper.stepsToGo() == 0 ){ // 如果stepsToGo=0，表示步進馬達已轉完應走的step了
    delay(500);
    stepper.move(4096);    //正轉一圈
    //stepper.move(-4096);  //負數就是反轉，反轉一圈
  }
}

//https://www.twblogs.net/a/5c53d458bd9eee06ef361597
//char[] --> json


void setup() {
  Serial.begin(9600); //Baud rate: 9600
  
  // Set MQTT broker
  client.setServer(server, port);
  client.setCallback(callback);
  
  // 繼電器 (light:D4 feed:D5 pump:D6)
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  
  // Wife
  //setup_wifi();

}

void loop() {
  // Check MQTT broker connection status
  if (!client.connected()) {
    reconnect();
  }
 
  //stepper.run();  //步進機啟動
  //stepper_run();
  
  // Keep MQTT process on going
  client.loop();
}

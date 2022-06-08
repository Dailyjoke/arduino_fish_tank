/*

#include "Ultrasonic.h"
#include <OneWire.h>
#include <DallasTemperature.h>

//水溫感測DS18B20
//D3 == p3
int DS18S20_Pin = 3; //DS18S20 Signal pin on digital 2
//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2
float temp_max_range = 20;
float temp_min_range = 25;

// 水濁度感測器
//#define WaterDirty 480 //換水的濁度 //add
#define WNTUpin A0 //接腳  //add
float turb_max_range = 20;

// 超音波 D2
Ultrasonic ultrasonic(2);
//add
const int ON = 1;  
const int OFF = 0; 
#define WaterHigh 50 //mm最高水位 
#define WaterLow 80 //mm補水的水位低點 
#define WaterMidUp 60 //mm補水停止的水位點 
#define WaterDeep 120 //mm定義換水時抽水該停止的水面深度


// DHT 
#define DHTPIN 2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

//wife
int status = WL_IDLE_STATUS;
char ssid[] = "3715";
char pass[] = "12345678";
//char ssid[] = "Galaxy A70B599";
//char pass[] = "10241023";

//mqtt topic
char Level_topic[] = "team11/level/data";
char TempTurb_topic[] = "team11/temp_turb/data";
char TurbSetRange_topic[] = "team11/turb/set_range";
char TempSetRange_topic[] = "team11/temp/set_range";
char LightSwitch_topic[] =  "team11/light/switch";
char LightFeed_topic[] =  "team11/feed/switch";
char LightPumb_topic[] =  "team11/pump/switch";
char FishTankState_topic[] =  "team11/fish_tank_state/info";
char Flag_topic[] =  "team11/flag";

//mqtt connect
char client_Id[] = "team11";
char user_name[] = "iot2022";
char user_password[] = "12345678";

//mqtt server & port
// MQTT_2(text)
//char server[] = "test.mosquitto.org";
//int port = 1883;
// MQTT_2(外網)
char server[] = "140.127.196.119";
int port = 18311;
// MQTT_3(內網)
//char server[] = "192.168.251.21";
//int port = 18883;

// Clients for MQTT
WiFiClient wifiClient;
PubSubClient client(wifiClient);

// Timer info
#define TIME 10000
#define LED_FLASH_PERIOD 200
unsigned long temp_last_time, led_last_time;

void callback(char* topic, byte* payload, unsigned int length) {
  char received;
  // byte -> char
    for (int i=0;i<length;i++)
      received = (char)payload[i];
 /*
  //接收MQTT訊息(感測時程偵測)
  if(!strcmp(topic, Flag_topic) && received == '1'){
    
    //is detect
    //float tu = turbidity_get();
    //float te = getTemp();
    // Output incoming message to serial terminal
    Serial.print("Temperature: "); 
    //Serial.print(te);
    Serial.print("*C");
    Serial.print("Turb: "); 
    //Serial.print(tu);
    Serial.println(" ");  
  
    //傳送至MQTT
    // 儲存訊息的字串變數
    String msgStr = "";
    char json[40];
    //json格式字串
    //msgStr = msgStr + "{\"topic\":" + TempTurb_topic + ", \"data\":\"null\",\"temp\":" + te + ",\"turb\":" + tu + "}";
    // 把String字串轉換成字元陣列格式
    msgStr.toCharArray(json, 40);
    // 發布MQTT主題與訊息
    client.publish(TempTurb_topic, json);
    
    //數據分析
    if(temp_max_range < te || temp_min_range > te){
       //水溫、水濁出問題 
       if(turb_max_range < tu)
        client.publish(FishTankState_topic, "3");
       //水溫出問題 
       else
        client.publish(FishTankState_topic, "2");
    }
    else{
      //水濁出問題 
      if(turb_max_range < tu)
        client.publish(FishTankState_topic, "1");
      //都沒問題 
      else
        client.publish(FishTankState_topic, "0");
      }
      
  }

  //接收MQTT訊息(水溫警示範圍變動)
  if(!strcmp(topic, TempSetRange_topic)){
    temp_max_range = (float)received;
    temp_max_range = (float)received;
    }

   //接收MQTT訊息(水濁警示範圍變動)
   if(!strcmp(topic, TurbSetRange_topic)){
    turb_max_range = (float)received;
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
      client.subscribe(TempTurb_topic);
      client.subscribe(TurbSetRange_topic);
      client.subscribe(TempSetRange_topic);
      client.subscribe(LightSwitch_topic);
      client.subscribe(LightFeed_topic);
      client.subscribe(LightPumb_topic);
      client.subscribe(FishTankState_topic);
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

void urs_get(){
  // Get USR
  long distance;
  // Read URS(cm)
  distance = ultrasonic.MeasureInCentimeters();
  // pubilsh to MQTT broker
  char buf[10];
  //sprintf(buf, "%s", String(distance).c_str());
  Serial.print("ultrasonic: ");
  Serial.print(String(distance).c_str()); // print out the value you read:
  Serial.println(" cm");
  //client.publish(Level_topic, buf);
}

float turbidity_get(){
  //get  gravity arduino turbidty sensor
  //A0 = p14
  
    int sensorValue = analogRead(A0);// read the input on analog pin 0:
    float voltage = sensorValue * (5.0 / 1024.0); // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    Serial.println(sensorValue); // print out the value you read:
    Serial.print("turbidity: ");
    Serial.print(voltage); // print out the value you read:
    Serial.println(" NTU");
  //return voltage;
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
  //client.setServer(server, port);
  //client.setCallback(callback);
  
  // start up DHT sensor
  //dht.begin();
  
  // 繼電器
  //pinMode(controlPin_1, OUTPUT);
  
  // Wife
  //setup_wifi();

  led_last_time = millis();
  temp_last_time = millis();
}

void loop() {
  // Check MQTT broker connection status
  //if (!client.connected()) {
   // reconnect();
  //}
  unsigned long current_time = millis();
  if( TIME < (current_time - temp_last_time)){
    urs_get();
    //getTemp();
    turbidity_get();
    // update last time value
    temp_last_time = current_time;
  }
 
  
  //stepper.run();  //步進機啟動
  //stepper_run();
  
  // Keep MQTT process on going
  //client.loop();

}
*/

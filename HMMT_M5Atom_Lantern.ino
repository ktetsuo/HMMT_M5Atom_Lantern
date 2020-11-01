
//
//  Hamamatsu Project
//    Data Transfer System by MQTT for M5StickC
//
//    Oct.18 , 2020 FabLab Hamamatsu
//
#include <M5Atom.h>
#include "StepperDriver.h"
//#include <M5StickC.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

#include "StepperDriver.h"

//Stepper motor sttings
int motor_steps = 200;
int step_divisition = 16;//32
int en_pin = 22;
int dir_pin = 23;
int step_pin = 19;
int step = 0;
int speed = 0;
boolean motor_state = 0;
StepperDriver ss(motor_steps, step_divisition, en_pin, dir_pin, step_pin);

unsigned long duration_time = 100;

// WiFi settings
const char ssid[] = "";         //  #### Your Wifi ID
const char password[] = "";     //  #### Your Wifi PW
WiFiClient wifiClient;

// MQTT settings
const char* mqttBrokerAddr = "";
const char* mqttUserName = "";
const char* mqttPassword = "";
const int mqttPort = 1883;
const char* mqttClientID = "HMMT_ZOETROPE";   // unique id that anything you like
PubSubClient mqttClient(wifiClient);

const String yourDevice("HMMT_lantern");        //  #### Your Device
const String midiTopic("MIDI");
const String gyroTopic("GYRO");
const String swTopic("M5SW");

const String speedTopic("lantern/speed");

const int RELAY_PIN(33);

// Global Variables
unsigned long lastUpdateTime = 0;

//-------------------------------
//  Arduino Functions
//-------------------------------
void setup() {

  //stepper motor
  ss.setSpeed(0);
  ss.powerEnable(true);

  int wifiCheckCount = 0;
  //initPattern();

  M5.begin(true, false, true); // シリアル, I2C, LED
  initPrintSomewhere();
  WiFi.begin(ssid, password);
  Serial.begin(115200);         // RX=32   TX=33
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);
  Serial.println("setup done!!");

  //  while (WiFi.status() != WL_CONNECTED) {
  //    printSomewhere(".");
  //    delay(500);
  //    if ( ++wifiCheckCount > 2000 ) {
  //      printSomewhere("\n");
  //      printSomewhere("No Wifi Connection!\n");
  //      break;
  //    }
  //  }

  //  MQTT
  mqttClient.setServer(mqttBrokerAddr, mqttPort);
  mqttClient.setCallback(mqttCallback);
  printSomewhere("\n");

  // 別タスクの起動
  xTaskCreatePinnedToCore(task0, "Task0", 4096, NULL, 1, NULL, 0);

}

// MQTTの再接続に時間がかかるみたいなので、別タスクで動かす
void task0(void* param)
{
  while (true) {
    if (checkWifi()) {
      if (checkMQTT()) {
        M5.dis.drawpix(0, 0x0000ff);  // WIFI,MQTT接続は青
        mqttClient.loop();
      } else {
        M5.dis.drawpix(0, 0xffff00);  // MQTT未接続は黄色
        vTaskDelay(1000); // 再接続まで間隔を開ける
      }
    } else {
      M5.dis.drawpix(0, 0x00ff00);  // Wifi切断は赤（色はなぜかBRG順）
    }
    vTaskDelay(1); // タスク内の無限ループには必ず入れる
  }
}

void loop() {
  unsigned long t = micros();
  M5.update();
  if (duration_time != 0xffffffff) {
    if (t - lastUpdateTime >= duration_time) {
      lastUpdateTime = t;
      motor_state = !motor_state;
      digitalWrite(19, motor_state);
    }
  }

  // Push Button A
  if (M5.Btn.wasPressed()) {
    // Send
    const String topicStr = yourDevice + "/" + swTopic + "/A";
    const char* const topic = topicStr.c_str();
    const char* const msg = "Pressed";
    printSomewhere(topic);
    printSomewhere(msg);
    mqttClient.publish(topic, msg);
    duration_time = 50.0;
    //startPattern(0);
    //digitalWrite(RELAY_PIN, HIGH);
  }
  // Release Button A
  if (M5.Btn.wasReleased()) {
    // Send
    const String topicStr = yourDevice + "/" + swTopic + "/A";
    const char* const topic = topicStr.c_str();
    const char* const msg = "Released";
    printSomewhere(topic);
    printSomewhere(msg);
    mqttClient.publish(topic, msg);
    duration_time = 100.0;
    //digitalWrite(RELAY_PIN, LOW);
  }
}

//-------------------------------
//  MQTT
//-------------------------------
void mqttCallback(char* topic, byte * payload, unsigned int length)
{
  String yd = topic;
  //  int sp1 = yd.indexOf('/');
  //  int sp2 = yd.lastIndexOf('/');
  //  String dev = yd.substring(0,sp1);
  //  String type = yd.substring(sp1+1,sp2);
  //  String ev = yd.substring(sp2+1,yd.length());

  payload[length] = '\0';

  // Add conditions
  if (yd.equals(speedTopic)) {
    printSomewhere("speed:");
    //workAsMassageChair(0, payload, length);

    const String msg = (char*)payload;
    float value = atof(msg.c_str());
    //    Serial.println(value);
    if (value >= 0.0001) {
      duration_time = 1000000 / (value / 60 * motor_steps * step_divisition * 2);
    } else {
      duration_time = 0xffffffff;
    }
    //Serial.println("test:");
    Serial.println(duration_time);
    //printSomewhere((int)(value*100));
  }
  else {
  }
}

boolean checkWifi() {
  static int lastWiFiStatus = WL_IDLE_STATUS;
  int wifiStatus = WiFi.status();
  if (lastWiFiStatus != WL_CONNECTED && wifiStatus == WL_CONNECTED) {
    printSomewhere("WiFi connected.");
    printSomewhere("IP address: ");
    printSomewhere(WiFi.localIP().toString().c_str());
  } else if (lastWiFiStatus != wifiStatus && wifiStatus != WL_CONNECTED) {
    printSomewhere("WiFi disconnected.");
    printSomewhere(wifiStatus);
    if (wifiStatus == WL_CONNECT_FAILED) {
      WiFi.begin(ssid, password);
    }
  }
  lastWiFiStatus = wifiStatus;
  return (wifiStatus == WL_CONNECTED);
}

boolean checkMQTT() {
  unsigned long t = millis();
  boolean conn = mqttClient.connected();
  if (!conn) {
    printSomewhere("MQTT Disconnect. Connecting...");
    conn = mqttClient.connect(mqttClientID, mqttUserName, mqttPassword);
    if (conn) {
      printSomewhere("MQTT Connect OK.");
      mqttClient.subscribe("lantern/#");
    } else {
      printSomewhere("MQTT Connect failed, rc=");
      // http://pubsubclient.knolleary.net/api.html#state に state 一覧が書いてある
      printSomewhere(mqttClient.state());
    }
  }
  return conn;
}

//-------------------------------
//  Print for Debug
//-------------------------------
void initPrintSomewhere(void)
{
}
void printSomewhere(int num)
{
  char strx[128] = {0};
  sprintf(strx, "%d", num);
  Serial.println(strx);
  //M5.Lcd.printf("%s",strx);
}
void printSomewhere(const char* txt)
{
  Serial.println(txt);
  //M5.Lcd.printf(txt);
}

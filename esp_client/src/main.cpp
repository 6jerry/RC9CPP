#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "XboxSeriesXControllerESP32_asukiaaa.hpp"
#include "RC9Protocol.hpp"

// 定义WiFi AP的SSID和密码
#define WIFI_SSID "ESP32_AP"
#define WIFI_PASS "12345678"

// 定义UDP服务器的IP地址和端口
#define SERVER_IP "192.168.4.1"
#define UDP_PORT 3333

String xbox_string();
RC9Protocol puber;
WiFiUDP udp;

// Required to replace with your xbox address
// 需要在此替换成自己的手柄蓝牙MAC地址
XboxSeriesXControllerESP32_asukiaaa::Core
    xboxController("0c:35:26:56:a0:22");

void setup() {
  // 连接到WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");

  // 设置超时时间（例如10秒）
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
    delay(500);
    Serial.print(".");
  }
  // 检查是否连接成功
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("Connected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println();
    Serial.println("Failed to connect to WiFi");
    // 可以在这里添加重试逻辑或重启ESP32
    ESP.restart(); // 重启ESP32
  }

  // 初始化UDP
  if (udp.begin(UDP_PORT)) {
    Serial.println("UDP initialized");
  } else {
    Serial.println("UDP initialization failed");
    return;
  }
  
  xboxController.begin();
}

void loop() {
  xboxController.onLoop();
  if (xboxController.isConnected())
  {
    if (xboxController.isWaitingForFirstNotification())
    {
    }
    else
    {     
      puber.pack_data(&xboxController.xboxNotif);
      udp.beginPacket(SERVER_IP, UDP_PORT);
      udp.write(puber.sendBuffer_,puber.tx_frame_mat.data_length + 8); // 发送数据
      udp.endPacket();
      puber.send_data();
    }
  }
  else
  {
    Serial.println("not connected xbox");
    if (xboxController.getCountFailedConnection() > 2)
    {
    }
  }
  delay(20);
}

String xbox_string()
{
  String str = String(xboxController.xboxNotif.btnY) + "," +
               String(xboxController.xboxNotif.btnX) + "," +
               String(xboxController.xboxNotif.btnB) + "," +
               String(xboxController.xboxNotif.btnA) + "," +
               String(xboxController.xboxNotif.btnLB) + "," +
               String(xboxController.xboxNotif.btnRB) + "," +
               String(xboxController.xboxNotif.btnSelect) + "," +
               String(xboxController.xboxNotif.btnStart) + "," +
               String(xboxController.xboxNotif.btnXbox) + "," +
               String(xboxController.xboxNotif.btnShare) + "," +
               String(xboxController.xboxNotif.btnLS) + "," +
               String(xboxController.xboxNotif.btnRS) + "," +
               String(xboxController.xboxNotif.btnDirUp) + "," +
               String(xboxController.xboxNotif.btnDirRight) + "," +
               String(xboxController.xboxNotif.btnDirDown) + "," +
               String(xboxController.xboxNotif.btnDirLeft) + "," +
               String(xboxController.xboxNotif.joyLHori) + "," +
               String(xboxController.xboxNotif.joyLVert) + "," +
               String(xboxController.xboxNotif.joyRHori) + "," +
               String(xboxController.xboxNotif.joyRVert) + "," +
               String(xboxController.xboxNotif.trigLT) + "," +
               String(xboxController.xboxNotif.trigRT) + "\n";
  return str;
};
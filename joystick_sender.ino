//=== joystick_sender (發送端) ===
#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// 腳位定義
#define xPin 34
#define yPin 35
#define zPin 32
#define rotate 33

int xPinNormal, yPinNormal, rotateNormal;

uint8_t receiverAddress[] = {0x7C, 0x87, 0xCE, 0x4A, 0x66, 0x22};

// 可調參數
const int deadzone = 100;
const int maxMove = 900;
const float curvePower = 15.0; // 彎曲程度

// 非線性轉換函式
int nonlinearOutput(int value, int center, bool positiveDir) {
  int offset = value - center;
  if (positiveDir && offset > deadzone) {
    float normalized = float(offset - deadzone) / float(maxMove - deadzone);
    normalized = constrain(normalized, 0.0, 1.0);
    float curved = pow(normalized, curvePower);
    return int(curved * 255);
  } else if (!positiveDir && offset < -deadzone) {
    float normalized = float(-offset - deadzone) / float(maxMove - deadzone);
    normalized = constrain(normalized, 0.0, 1.0);
    float curved = pow(normalized, curvePower);
    return int(curved * 255);
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_SENDER", true); // 藍牙名稱
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
  pinMode(zPin, INPUT);
  pinMode(rotate, INPUT);
  delay(100); // ADC 穩定時間

  // 初始化中心值
  xPinNormal = analogRead(xPin);
  yPinNormal = analogRead(yPin);
  rotateNormal = analogRead(rotate); 

  while (!SerialBT.connect(receiverAddress)) {
    Serial.println("Connecting failed... retrying in 1 second");
    SerialBT.connect(receiverAddress);
    delay(1000);
  }

  Serial.println("Connected to receiver!");
  
}

void loop() {

  int X = analogRead(xPin);
  int Y = analogRead(yPin);
  int Z = digitalRead(zPin);
  int Rval = analogRead(rotate);

  int F = nonlinearOutput(Y, yPinNormal, true);
  int B = nonlinearOutput(Y, yPinNormal, false);
  int L = nonlinearOutput(X, xPinNormal, true);
  int R = nonlinearOutput(X, xPinNormal, false);
  int CW = nonlinearOutput(Rval, rotateNormal, true);
  int CCW = nonlinearOutput(Rval, rotateNormal, false);

  int rot = CW - CCW;
  String data = String(F) + "," + String(B) + "," + String(L) + "," + String(R) + "," + String(Z) + "," + String(rot);
  SerialBT.println(data);

  // erial.println("Send: " + data);
  Serial.printf("X:%d Y:%d Z:%d Rotate:%d rot:%d\n", X, Y, Z, Rval, rot);

  delay(10); // 控制頻率


}

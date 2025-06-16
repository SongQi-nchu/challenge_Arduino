#include <BluetoothSerial.h>
#include <esp_bt_device.h>

BluetoothSerial SerialBT;

#define Launch_pin 21  // 發射馬達 (使用 NPN 控制)
float scale = 0.5;

// 馬達結構定義
struct Motor {
  uint8_t in1, in2, pwmPin, pwmChannel;
};

Motor RF = {32, 33, 25, 0};
Motor RB = {26, 27, 14, 1};
Motor LF = {19, 18, 23, 2};
Motor LB = {17, 16, 22, 3};

// 初始化馬達腳位與 PWM
void setupMotor(Motor m) {
  pinMode(m.in1, OUTPUT);
  pinMode(m.in2, OUTPUT);
  ledcSetup(m.pwmChannel, 900, 8);
  ledcAttachPin(m.pwmPin, m.pwmChannel);
}

// 控制單顆馬達
void driveMotor(Motor m, int speed, int direction) {
  if (speed == 0) {
    Serial.printf("Driving PWM %d (Channel %d) Speed=%d Dir=%d\n", m.pwmPin, m.pwmChannel, speed, direction);
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, LOW);
  } else if (direction > 0) {  // 前進
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, LOW);
  } else if (direction < 0) {  // 後退
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, HIGH);
  }
  ledcWrite(m.pwmChannel, speed);
}

// 停止所有馬達
void stopAll() {
  for (Motor m : {RF, RB, LF, LB}) {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, LOW);
    ledcWrite(m.pwmChannel, 0);
  }
}

// 強制煞車
void brakeAll() {
  for (Motor m : {RF, RB, LF, LB}) {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, HIGH);
    ledcWrite(m.pwmChannel, 0);
  }
}

// 動作處理
void handleMotion(int F, int B, int L, int R, int Z,int rot) {
  // 發射馬達
  digitalWrite(Launch_pin, Z > 0 ? HIGH:LOW);
  if (Z > 0) {
    F = B = L = rot = R = 0;
  }
  if (Z == -1) {
    brakeAll();
    return;
  }

  // 斜向移動
  // if (F > 0 && L > 0) { // 左前
  //   driveMotor(RF, F*scale, 1);
  //   driveMotor(LB, F*scale, 1);
  //   driveMotor(RB, 0, 0);
  //   driveMotor(LF, 0, 0);
  // } else if (F > 0 && R > 0) { // 右前
  //   driveMotor(LF, F*scale, 1);
  //   driveMotor(RB, F*scale, 1);
  //   driveMotor(RF, 0, 0);
  //   driveMotor(LB, 0, 0);
  // } else if (B > 0 && L > 0) { // 左後
  //   driveMotor(RF, B*scale, -1);
  //   driveMotor(LB, B*scale, -1);
  //   driveMotor(RB, 0, 0);
  //   driveMotor(LF, 0, 0);
  // } else if (B > 0 && R > 0) { // 右後
  //   driveMotor(LF, B*scale, -1);
  //   driveMotor(RB, B*scale, -1);
  //   driveMotor(RF, 0, 0);
  //   driveMotor(LB, 0, 0);
  // }
  if (rot > 0) { // 順時針 (Turning Right)
    driveMotor(RF, rot*scale,-1);
    driveMotor(RB, rot*scale, 1);
    driveMotor(LF, rot*scale, -1);
    driveMotor(LB, rot*scale, 1);
    return;
  } else if (rot < 0) { // 逆時針 (Turning Left)
    driveMotor(RF, -rot*scale, 1);
    driveMotor(RB, -rot*scale, -1);
    driveMotor(LF, -rot*scale, 1);
    driveMotor(LB, -rot*scale, -1);
    return;
  }
  // 單向移動
  else if (F > 0) {
    driveMotor(RF, F*scale, -1);
    driveMotor(RB, F*scale, -1);
    driveMotor(LF, F*scale, -1);
    driveMotor(LB, F*scale, -1);
  } else if (B > 0) {
    driveMotor(RF, B*scale, 1);
    driveMotor(RB, B*scale, 1);
    driveMotor(LF, B*scale, 1);
    driveMotor(LB, B*scale, 1);
  } else if (L > 0) {
    driveMotor(RF, L*scale, -1);
    driveMotor(RB, L*scale, 1);
    driveMotor(LF, L*scale, 1);
    driveMotor(LB, L*scale, -1);
  } else if (R > 0) {
    driveMotor(RF, R*scale, 1);
    driveMotor(RB, R*scale, -1);
    driveMotor(LF, R*scale, -1);
    driveMotor(LB, R*scale, 1);
  } 
  else {
    stopAll();
  }

  // 旋轉（順時針 / 逆時針）
  

}

// 指令解析
void parseCommand(String msg) {
  int F, B, L, R, Z, rot;
  int comma1 = msg.indexOf(',');
  int comma2 = msg.indexOf(',', comma1 + 1);
  int comma3 = msg.indexOf(',', comma2 + 1);
  int comma4 = msg.indexOf(',', comma3 + 1);
  int comma5 = msg.indexOf(',', comma4 + 1);

  if (comma1 > 0 && comma2 > comma1 && comma3 > comma2 && comma4 > comma3 && comma5 > comma4) {
    F = msg.substring(0, comma1).toInt();
    B = msg.substring(comma1 + 1, comma2).toInt();
    L = msg.substring(comma2 + 1, comma3).toInt();
    R = msg.substring(comma3 + 1, comma4).toInt();
    Z = msg.substring(comma4 + 1, comma5).toInt();
    rot = msg.substring(comma5 + 1).toInt();

    Serial.printf("F:%d B:%d L:%d R:%d Z:%d ROT:%d\n", F, B, L, R, Z, rot);
    handleMotion(F, B, L, R, Z, rot);  // 你需要讓 handleMotion 支援 rot 參數
  }
}


void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_RECEIVER");

  pinMode(Launch_pin, OUTPUT);
  setupMotor(RF);
  setupMotor(RB);
  setupMotor(LF);
  setupMotor(LB);

  Serial.println("等待發送端連線...");
  const uint8_t* mac = esp_bt_dev_get_address();
  Serial.printf("接收端 MAC 地址: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void loop() {
  static unsigned long lastUpdate = 0;
  const int interval = 20;  // 快速輪詢
  if (millis() - lastUpdate >= interval) {
    lastUpdate = millis();

    static String inputBuffer = "";

    while (SerialBT.available()) {
      char c = SerialBT.read();

      if (c == '\n') {
        parseCommand(inputBuffer);
        inputBuffer = "";
      } else {
        inputBuffer += c;
        if (inputBuffer.length() > 100) inputBuffer = ""; // 清除異常長度
      }
    }
  }
}   

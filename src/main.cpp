#include <Arduino.h>

#define SENSOR_INTERVAL 100   // センサの感覚(mm)
#define MAX_MESUREMENT_TIME 100   // 連続測定回数の上限

// ピン定義
const int FRONT_SENSOR = 2;
const int BACK_SENSOR = 3;
const int IND_LED = 4;
const int BUZZER = 8;
const int RIGHT_BUTTON = 5;
const int MIDDLE_BUTTON = 6;
const int LEFT_BUTTON = 7;
const int FRONT_LED = 18;
const int BACK_LED = 19;

volatile uint32_t time_1, time_2;

uint32_t interval_time;

double speed;

bool enable_front;
bool enable;

uint16_t measurement_times;
double speed_data[MAX_MESUREMENT_TIME];

void SensorFront() {
      time_1 = micros();
      enable_front = 1;
}

void SensorBack() {
      time_2 = micros();
      if (enable_front == 1) {
            enable = 1;
            enable_front = 0;
      }
}

void setup() {
      Serial.begin(9600);

      // ピン定義
      pinMode(IND_LED, OUTPUT);
      pinMode(FRONT_LED, OUTPUT);
      pinMode(BACK_LED, OUTPUT);
      pinMode(RIGHT_BUTTON, INPUT);
      pinMode(MIDDLE_BUTTON, INPUT);
      pinMode(LEFT_BUTTON, INPUT);

      // 割り込み定義
      attachInterrupt(digitalPinToInterrupt(2), SensorFront, RISING);
      attachInterrupt(digitalPinToInterrupt(3), SensorBack, RISING);

      // 起動音
      tone(BUZZER, 2000, 100);
      delay(100);
      tone(BUZZER, 1000, 100);
      delay(100);

      // 測定開始
      digitalWrite(FRONT_LED, HIGH);
      digitalWrite(BACK_LED, HIGH);

      Serial.println("-Start measurement-");
}

void loop() {
      if (enable == 1) {
            interval_time = time_2 - time_1;
            speed = ((double)SENSOR_INTERVAL / (double)interval_time) * 1000;
            speed_data[measurement_times] = speed;

            measurement_times++;

            Serial.print(measurement_times);
            Serial.print(": ");
            Serial.print(speed, 4);
            Serial.println(" m/s");

            digitalWrite(IND_LED, HIGH);
            tone(BUZZER, 1000, 100);
            delay(200);

            digitalWrite(IND_LED, LOW);
            enable = 0;
      }

      if (digitalRead(MIDDLE_BUTTON) == 1) {
            if (measurement_times == 0) {
                  Serial.println();
                  Serial.println("Error");
                  Serial.println();
                  Serial.println("-Start measurement-");

                  delay(500);
            } else {
                  double speed_ave = 0;
                  double max_speed = speed_data[0];
                  double min_speed = speed_data[0];

                  for (uint16_t i = 0; i < measurement_times; i++) {
                        speed_ave += speed_data[i];

                        if (max_speed < speed_data[i]) max_speed = speed_data[i];   // 最大値を求める
                        if (min_speed > speed_data[i]) min_speed = speed_data[i];   // 最小値を求める
                  }
                  speed_ave = speed_ave / double(measurement_times);

                  measurement_times = 0;   // 測定結果のリセット

                  // 測定結果の表示
                  Serial.println();

                  Serial.println("-Result-");

                  Serial.print("Ave : ");
                  Serial.print(speed_ave, 4);
                  Serial.println(" m/s");

                  Serial.print("Max : ");
                  Serial.print(max_speed, 4);
                  Serial.println(" m/s");

                  Serial.print("Min : ");
                  Serial.print(min_speed, 4);
                  Serial.println(" m/s");

                  Serial.println();
                  Serial.println("-Start measurement-");

                  delay(500);
            }
      }
}

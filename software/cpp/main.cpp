#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>

// ===== Threads =====
TaskHandle_t TaskMotorHandle;   // Task-Handle-Variable
TaskHandle_t TaskIMUHandle;
// ===== Zustände & Variablen =====
bool motor_enabled = false;
volatile bool motor_done = false;
volatile bool read_imu = false;
volatile bool start_calib = false;
int transitionSpeed = 10000;
float dt = 8; // Default: 2s
int stillstands = 6; // Default: 10 // Eventuell Ändern auf 8 oder 6
float degreeZ = 180; // Default: 45.0
int rotation_count = 0;
float beschleunigung = 10000.0;

// ===== Pins =====
#define DIR_PIN_1 26
#define DIR_PIN_2 19
#define STEP_PIN_1 18
#define STEP_PIN_2 25
#define ENABLE_PIN 33
#define M0_PIN 14
#define M1_PIN 27
#define M2_PIN 32

// ===== Kinematik =====
const float zaehne_Zahnriemen = 15.0f;
const float zaehne_Ritzel = 59.0f;
const float uebersetzung = (zaehne_Ritzel / zaehne_Zahnriemen) / 2.0;
const float vollSchritteproUmdrehung = 200;
int microstepping = 16;
float mikroSchritteProUmdrehung = vollSchritteproUmdrehung * microstepping;

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
const unsigned long startDelayMs = 3000;
unsigned long t0;

// ===== Funktionen =====
float berechneMikroschritteProSekunde(float winkelgeschwindigkeit) {
  float winkelgeschwindigkeit_motor = winkelgeschwindigkeit * uebersetzung;
  float umdrehungen_pro_sekunde = winkelgeschwindigkeit_motor / 360;
  return umdrehungen_pro_sekunde * mikroSchritteProUmdrehung;
}

float berechneMikroschritteProDrehung(float rotation) {
  return rotation * mikroSchritteProUmdrehung * uebersetzung;
}

// ===== Setup Motorpins =====
void starte_kinematik_setup() {
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH); // deaktiviert
  digitalWrite(M0_PIN, HIGH);
  digitalWrite(M1_PIN, HIGH);
  digitalWrite(M2_PIN, LOW);

  stepper1.setMinPulseWidth(5);
  stepper1.setAcceleration(beschleunigung);
  stepper1.setCurrentPosition(0);
  stepper2.setMinPulseWidth(5);
  stepper2.setAcceleration(beschleunigung/2);
  stepper2.setCurrentPosition(0);
}


void disableMotor() {
  digitalWrite(ENABLE_PIN, HIGH);
  motor_enabled = false;
}

void enableMotor() {
  digitalWrite(ENABLE_PIN, LOW);
  motor_enabled = true;;
}

// ===== Gleichverteilte Zufallslisten =====
void init_random_profiles(float *zufall_dt_values, float *zufall_values, int n) {
  float dt_min = 2.0, dt_max = 8.0;
  float val_min = 10.0, val_max = 40.0;

  // gleichmäßig verteilen
  for (int i = 0; i < n; i++) {
    zufall_dt_values[i] = dt_min + i * (dt_max - dt_min) / (n - 1);
    zufall_values[i] = val_min + i * (val_max - val_min) / (n - 1);
  }

  // Fisher-Yates Shuffle
  for (int i = n - 1; i > 0; i--) {
    int j = random(0, i + 1);
    float tmp = zufall_dt_values[i];
    zufall_dt_values[i] = zufall_dt_values[j];
    zufall_dt_values[j] = tmp;

    tmp = zufall_values[i];
    zufall_values[i] = zufall_values[j];
    zufall_values[j] = tmp;
  }
}

// ===== Bewegung in Stillstandspositionen =====
void fahre_stillstandspositionen_ab(int motor) {
  //float rotation = 2.0 / stillstands;
  float deg = 360.0 / stillstands;
  float rotation = deg * 2 / 360.0;

  // Zufallslisten erzeugen
  float zufall_dt_values[stillstands];
  float zufall_values[stillstands];
  init_random_profiles(zufall_dt_values, zufall_values, stillstands);

  if (motor == 1) {

    for (int i = 0; i < stillstands; i++) {
      disableMotor();
      float zufall_dt = zufall_dt_values[i];  // reproduzierbar & verteilt
      vTaskDelay(zufall_dt * 1000 / portTICK_PERIOD_MS); // Stillstandzeit
      enableMotor();
      digitalWrite(DIR_PIN_1, LOW);

      float zufall = zufall_values[i];        // reproduzierbar & verteilt
      float winkelgeschwindigkeit = deg * zufall / dt;
      stepper1.setMaxSpeed(berechneMikroschritteProSekunde(winkelgeschwindigkeit));

      float mikroSchritteInsgesamt = berechneMikroschritteProDrehung(rotation);
      long ziel = stepper1.currentPosition() + mikroSchritteInsgesamt;
      Serial.print("Motor1 Schritte: "); Serial.println(ziel);
      stepper1.moveTo(ziel);
      // Bewegung laufen lassen, ohne zu blockieren
      while (stepper1.distanceToGo() != 0) { stepper1.run(); }
      stepper1.setCurrentPosition(0);
    }
  }
  else if (motor == 2){

    for (int i = 0; i < stillstands; i++) {
      disableMotor();
      float zufall_dt = zufall_dt_values[i];  // reproduzierbar & verteilt; // Ändern auf 8 und 2?
      vTaskDelay(zufall_dt * 1000 / portTICK_PERIOD_MS); // Stillstandzeit
      enableMotor();
      digitalWrite(DIR_PIN_2, LOW);
      float zufall = zufall_values[i];        // reproduzierbar & verteilt
      float winkelgeschwindigkeit = deg * zufall / dt;
      stepper2.setMaxSpeed(berechneMikroschritteProSekunde(winkelgeschwindigkeit)/2);

      float mikroSchritteInsgesamt = berechneMikroschritteProDrehung(rotation);
      long ziel = stepper2.currentPosition() + mikroSchritteInsgesamt /2;
      Serial.print("Motor2 Schritte: "); Serial.println(ziel);
      stepper2.moveTo(ziel);
      // Bewegung laufen lassen, ohne zu blockieren
      while (stepper2.distanceToGo() != 0) { stepper2.run(); }
      stepper2.setCurrentPosition(0);
    }
  }


}

void homing() {
  float rotation = 2.0;  // wie vorher
  float richtungsfaktor = -1.0; // immer rückwärts

  stepper1.setAcceleration(5000);
  stepper2.setAcceleration(5000/2);
  // Geschwindigkeit setzen
  stepper1.setMaxSpeed(transitionSpeed);
  stepper2.setMaxSpeed(transitionSpeed / 2);

  // Zielpositionen (negativ für Rückwärtsdrehung)
  long ziel1 = stepper1.currentPosition() + richtungsfaktor * berechneMikroschritteProDrehung(rotation);
  long ziel2 = stepper2.currentPosition() + richtungsfaktor * berechneMikroschritteProDrehung(rotation) / 2;

  stepper1.moveTo(ziel1);
  stepper2.moveTo(ziel2);

  // Bewegung ausführen
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
  }

  // Position zurücksetzen (optional)
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper1.setAcceleration(beschleunigung);
  stepper2.setAcceleration(beschleunigung/2);
}

void transitionZ() {
  stepper1.setAcceleration(5000);
  stepper2.setAcceleration(5000/2);
  // Geschwindigkeit für Übergangsbewegung setzen
  stepper1.setMaxSpeed(transitionSpeed);
  stepper2.setMaxSpeed(transitionSpeed/2);

  // Rotation berechnen: degreeZ (z. B. 60 °)
  float rotation = (degreeZ / 360.0) * 2.0;   // *2 wegen Übersetzung
  float richtungsfaktor = -1.0; // immer rückwärts

  // Zielposition berechnen und Bewegung starten
  long ziel1 = stepper1.currentPosition() + berechneMikroschritteProDrehung(rotation);
  long ziel2 = stepper2.currentPosition() + berechneMikroschritteProDrehung(rotation) / 2;
  stepper1.moveTo(ziel1);
  stepper2.moveTo(ziel2);

  // Bewegung ausführen, nicht blockierend
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
  }
  stepper1.setAcceleration(beschleunigung);
  stepper2.setAcceleration(beschleunigung/2);
}

void transition_cable() {
  stepper1.setAcceleration(5000);
  stepper2.setAcceleration(5000/2);
  float rotation = 2.0;  // wie vorher
  float richtungsfaktor = -1.0; // immer rückwärts

  // Geschwindigkeit setzen
  stepper1.setMaxSpeed(transitionSpeed);
  stepper2.setMaxSpeed(transitionSpeed / 2);

  // Zielpositionen (negativ für Rückwärtsdrehung)
  long ziel1 = stepper1.currentPosition() + richtungsfaktor * berechneMikroschritteProDrehung(rotation);
  long ziel2 = stepper2.currentPosition() + richtungsfaktor * berechneMikroschritteProDrehung(rotation) / 2;

  stepper1.moveTo(ziel1);
  stepper2.moveTo(ziel2);

  // Bewegung ausführen
  while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
    stepper1.run();
    stepper2.run();
  }

  // Position zurücksetzen (optional)
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper1.setAcceleration(beschleunigung);
  stepper2.setAcceleration(beschleunigung/2);

}

// ===== MPU =====
const uint8_t MPU = 0x68;
const float g0 = 9.81;
const float MPUscaleFactorAcc = 16384.0;
const float MPUscaleFactorAccMS2 = g0 / MPUscaleFactorAcc;
const float MPUscaleFactorGyro = 131.0;
const float MPUscaleFactorGyroRADS = MPUscaleFactorGyro / (PI / 180.0);

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void mpu_setup(uint16_t rate_hz = 100) {
  writeReg(0x6B, 0x01);
  writeReg(0x6C, 0x00);
  writeReg(0x1B, 0x00);
  writeReg(0x1C, 0x00);
  writeReg(0x1A, 0x04);
  uint8_t div = (uint8_t)(1000 / rate_hz - 1);
  writeReg(0x19, div);
  writeReg(0x6A, 0x04);
}

void readDataIMU() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);
  if (Wire.available() == 14) {
    int16_t ax = (Wire.read() << 8) | Wire.read();
    int16_t ay = (Wire.read() << 8) | Wire.read();
    int16_t az = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();
    int16_t wx = (Wire.read() << 8) | Wire.read();
    int16_t wy = (Wire.read() << 8) | Wire.read();
    int16_t wz = (Wire.read() << 8) | Wire.read();
    Serial.print((float)ax * MPUscaleFactorAccMS2, 5); Serial.print(",");
    Serial.print((float)ay * MPUscaleFactorAccMS2, 5); Serial.print(",");
    Serial.print((float)az * MPUscaleFactorAccMS2, 5); Serial.print(",");
    Serial.print((float)wx / MPUscaleFactorGyroRADS, 5); Serial.print(",");
    Serial.print((float)wy / MPUscaleFactorGyroRADS, 5); Serial.print(",");
    Serial.println((float)wz / MPUscaleFactorGyroRADS, 5);
  }
}


// ===== Task: Pyserial =====
void TaskPySerial(void *pvParameters) {
  for (;;) {
    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      // Signale
      if (cmd == "Start Raw Read") { Serial.println("Signal received."); read_imu = true; }
      else if (cmd == "Stop Raw Read") { Serial.println("Signal received."); read_imu = false; }
      else if (cmd == "Start Calibration") { Serial.println("Signal received."); start_calib = true; }
      else if (cmd == "Test") { Serial.println("Signal received."); start_calib = false; }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ===== Task: IMU =====
void TaskReadIMU(void *pvParameters) {
  const TickType_t period = pdMS_TO_TICKS(10); // 100 Hz
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    if (read_imu) {
      readDataIMU();
      vTaskDelayUntil(&lastWake, period); // <-- absoluter Takt, konstante Rate
    } else {
      vTaskDelay(pdMS_TO_TICKS(50));
      lastWake = xTaskGetTickCount();     // Takt resetten wenn pausiert
    }
  }
}

/*
void TaskMotor(void *pvParameters) {
  for (;;) {
    if (start_calib) {
      fahre_stillstandspositionen_ab(1);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }

  }
}
*/


void TaskMotor(void *pvParameters) {
  for (;;) {
    if (start_calib) {
      read_imu = true;
      if (rotation_count % 2 == 0) {
        //Serial.print("rotation: "); Serial.println(rotation_count);
        fahre_stillstandspositionen_ab(1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        rotation_count++;
      }
      else {
        //Serial.print("rotation: "); Serial.println(rotation_count);
        fahre_stillstandspositionen_ab(2);
        read_imu = false;
        vTaskDelay(10 / portTICK_PERIOD_MS);
        transition_cable();
        vTaskDelay(100 / portTICK_PERIOD_MS);
        transitionZ();
        rotation_count++;
      }
      //randomSeed(42 + rotation_count);

      if (rotation_count >= (int)(360.0 * 2/ degreeZ)) {
        read_imu = true;
        vTaskDelay(dt * 1000 / portTICK_PERIOD_MS);
        read_imu = false;
        motor_done = true;
        start_calib = false;
        rotation_count = 0;
        homing();
        disableMotor();
        Serial.println("Calibration Done.");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
      }

      vTaskDelay(1 / portTICK_PERIOD_MS);

    }
    else {
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
  }
}



// ===== Setup =====
void setup() {
  delay(10000);
  Serial.begin(115200);
  randomSeed(40);
  starte_kinematik_setup();
  Wire.begin(21, 22);
  Wire.setClock(400000);
  mpu_setup();
  delay(100);
  Serial.println("ESP32 Dual-Core Motor+IMU gestartet.");

  xTaskCreatePinnedToCore(TaskReadIMU, "TaskIMU", 8192, NULL, 2, &TaskIMUHandle, 0);
  xTaskCreatePinnedToCore(TaskMotor, "TaskMotor", 8192, NULL, 2, &TaskMotorHandle, 1);
  xTaskCreatePinnedToCore(TaskPySerial, "TaskSerial", 4096, NULL, 1, NULL, 0);

}

void loop() {}

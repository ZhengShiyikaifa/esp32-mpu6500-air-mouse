#include <Wire.h>
#include <BleMouse.h>

// ---------- Hardware ----------
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t LEFT_BUTTON_PIN = 25;   // Optional button: GPIO25 -> GND
constexpr uint8_t RIGHT_BUTTON_PIN = 26;  // Optional button: GPIO26 -> GND
constexpr uint8_t AIR_BUTTON_PIN = 27;    // Hold button: GPIO27 -> GND to move cursor
constexpr uint8_t ENCODER_A_PIN = 32;     // Encoder terminal A
constexpr uint8_t ENCODER_B_PIN = 33;     // Encoder terminal B
constexpr int DEBUG_RX2_PIN = 16;         // Board pin marked RX2
constexpr int DEBUG_TX2_PIN = 17;         // Board pin marked TX2

// ---------- Mouse tuning ----------
// Axis mapping after swapping horizontal and vertical controls:
// X rotation moves left/right, Z rotation moves up/down.
constexpr float X_DIRECTION = -1.0f;
constexpr float Y_DIRECTION = 1.0f;
constexpr float DEFAULT_MOUSE_GAIN = 1050.0f;
constexpr float MIN_MOUSE_GAIN = 300.0f;
constexpr float MAX_MOUSE_GAIN = 4000.0f;
constexpr float ENCODER_GAIN_STEP = 100.0f;
constexpr int8_t ENCODER_DIRECTION = 1;    // Change to -1 if knob direction is reversed
constexpr float GYRO_DEAD_ZONE = 0.045f;   // rad/s; suppresses small hand jitter
constexpr float KALMAN_PROCESS_NOISE = 0.04f;     // Larger = quicker response
constexpr float KALMAN_MEASUREMENT_NOISE = 0.008f; // Larger = smoother output
constexpr uint32_t SAMPLE_PERIOD_US = 10000; // 100 Hz
constexpr uint32_t DEBUG_PERIOD_MS = 100;    // 10 debug lines per second
constexpr uint16_t CALIBRATION_SAMPLES = 600;
constexpr float CALIBRATION_STDDEV_LIMIT = 0.025f;
constexpr float CALIBRATION_MEAN_LIMIT = 0.12f;

BleMouse bleMouse("ESP32 MPU6050 Air Mouse", "DIY", 100);
HardwareSerial DebugSerial(2);
uint8_t mpuAddress = 0;
uint8_t mpuWhoAmI = 0;

float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
float mouseGain = DEFAULT_MOUSE_GAIN;
float pixelRemainderX = 0.0f;
float pixelRemainderY = 0.0f;
uint32_t lastSampleUs = 0;
uint32_t lastDebugMs = 0;
bool airEnabled = false;
bool airButtonLastReading = false;
uint32_t airButtonChangedAt = 0;

struct ButtonState {
  uint8_t pin;
  uint8_t mouseButton;
  bool stablePressed;
  bool lastReading;
  uint32_t changedAt;
};

volatile int16_t encoderQuarterStepDelta = 0;
volatile uint8_t encoderLastState = 0;
volatile uint32_t encoderEdgeCount = 0;
volatile int32_t encoderDecodedTotal = 0;
const int8_t DRAM_ATTR ENCODER_TRANSITION_TABLE[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0};

void IRAM_ATTR onEncoderChange() {
  const uint8_t currentState =
      (digitalRead(ENCODER_A_PIN) << 1) | digitalRead(ENCODER_B_PIN);
  const uint8_t transition = (encoderLastState << 2) | currentState;
  const int8_t decoded = ENCODER_TRANSITION_TABLE[transition];
  if (currentState != encoderLastState) ++encoderEdgeCount;
  encoderQuarterStepDelta += decoded;
  encoderDecodedTotal += decoded;
  encoderLastState = currentState;
}

class KalmanFilter1D {
 public:
  float update(float measurement, float dt) {
    errorCovariance += KALMAN_PROCESS_NOISE * dt;
    const float gain = errorCovariance /
                       (errorCovariance + KALMAN_MEASUREMENT_NOISE);
    estimate += gain * (measurement - estimate);
    errorCovariance *= (1.0f - gain);
    return estimate;
  }

  void reset(float initialValue = 0.0f) {
    estimate = initialValue;
    errorCovariance = 1.0f;
  }

 private:
  float estimate = 0.0f;
  float errorCovariance = 1.0f;
};

KalmanFilter1D kalmanX;
KalmanFilter1D kalmanZ;

ButtonState leftButton{LEFT_BUTTON_PIN, MOUSE_LEFT, false, false, 0};
ButtonState rightButton{RIGHT_BUTTON_PIN, MOUSE_RIGHT, false, false, 0};

void debugLine(const char *line) {
  Serial.println(line);       // USB serial monitor
  DebugSerial.println(line);  // External USB-TTL adapter on TX2
}

bool readI2CRegister(uint8_t address, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

bool readI2CRegisters(uint8_t address, uint8_t reg, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, length) != length) return false;
  for (uint8_t i = 0; i < length; ++i) data[i] = Wire.read();
  return true;
}

bool writeI2CRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool supportedImuAt(uint8_t address, uint8_t &whoAmI) {
  if (!readI2CRegister(address, 0x75, whoAmI)) return false;
  // 0x68: MPU6050/MPU6000, 0x70: MPU6500,
  // 0x71/0x73: MPU9250/MPU9255 (same gyro register layout).
  return whoAmI == 0x68 || whoAmI == 0x70 || whoAmI == 0x71 || whoAmI == 0x73;
}

bool initializeImu() {
  if (!writeI2CRegister(mpuAddress, 0x6B, 0x80)) return false; // Device reset
  delay(100);
  if (!writeI2CRegister(mpuAddress, 0x6B, 0x01)) return false; // PLL clock
  if (!writeI2CRegister(mpuAddress, 0x6C, 0x00)) return false; // Enable axes
  if (!writeI2CRegister(mpuAddress, 0x1A, 0x04)) return false; // ~20 Hz DLPF
  if (!writeI2CRegister(mpuAddress, 0x1B, 0x08)) return false; // Gyro +/-500 dps
  if (!writeI2CRegister(mpuAddress, 0x19, 0x09)) return false; // 100 Hz sample rate
  delay(50);
  return true;
}

bool readGyroscope(float &gx, float &gy, float &gz) {
  uint8_t raw[6];
  if (!readI2CRegisters(mpuAddress, 0x43, raw, sizeof(raw))) return false;
  const int16_t rawX = (int16_t)((raw[0] << 8) | raw[1]);
  const int16_t rawY = (int16_t)((raw[2] << 8) | raw[3]);
  const int16_t rawZ = (int16_t)((raw[4] << 8) | raw[5]);
  constexpr float RAD_PER_LSB = (PI / 180.0f) / 65.5f;
  gx = rawX * RAD_PER_LSB;
  gy = rawY * RAD_PER_LSB;
  gz = rawZ * RAD_PER_LSB;
  return true;
}

void scanI2CBus() {
  uint8_t found = 0;
  for (uint8_t address = 1; address < 127; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      char line[48];
      snprintf(line, sizeof(line), "I2C: device responded at 0x%02X", address);
      debugLine(line);
      if (address == 0x68 || address == 0x69) {
        uint8_t whoAmI = 0;
        if (readI2CRegister(address, 0x75, whoAmI)) {
          snprintf(line, sizeof(line), "I2C: WHO_AM_I register = 0x%02X", whoAmI);
          debugLine(line);
        }
      }
      ++found;
    }
  }
  if (found == 0) debugLine("I2C: no devices responded on SDA21/SCL22");
}

float removeDeadZone(float value) {
  if (fabsf(value) <= GYRO_DEAD_ZONE) return 0.0f;
  return value > 0.0f ? value - GYRO_DEAD_ZONE : value + GYRO_DEAD_ZONE;
}

void calibrateGyroscope() {
  debugLine("CAL: Keep the MPU6050 completely still...");
  while (true) {
    float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
    float sumSqX = 0.0f, sumSqY = 0.0f, sumSqZ = 0.0f;
    uint16_t samples = 0;

    while (samples < CALIBRATION_SAMPLES) {
      float gx, gy, gz;
      if (!readGyroscope(gx, gy, gz)) {
        debugLine("ERROR: gyro read failed during calibration");
        delay(5);
        continue;
      }
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      sumSqX += gx * gx;
      sumSqY += gy * gy;
      sumSqZ += gz * gz;
      ++samples;
      delay(5);
    }

    const float meanX = sumX / samples;
    const float meanY = sumY / samples;
    const float meanZ = sumZ / samples;
    const float stdX = sqrtf(fmaxf(0.0f, sumSqX / samples - meanX * meanX));
    const float stdY = sqrtf(fmaxf(0.0f, sumSqY / samples - meanY * meanY));
    const float stdZ = sqrtf(fmaxf(0.0f, sumSqZ / samples - meanZ * meanZ));

    const bool lowVariance = stdX < CALIBRATION_STDDEV_LIMIT &&
                             stdY < CALIBRATION_STDDEV_LIMIT &&
                             stdZ < CALIBRATION_STDDEV_LIMIT;
    const bool reasonableBias = fabsf(meanX) < CALIBRATION_MEAN_LIMIT &&
                                fabsf(meanY) < CALIBRATION_MEAN_LIMIT &&
                                fabsf(meanZ) < CALIBRATION_MEAN_LIMIT;
    if (lowVariance && reasonableBias) {
      gyroBiasX = meanX;
      gyroBiasY = meanY;
      gyroBiasZ = meanZ;
      break;
    }

    char retryLine[180];
    snprintf(retryLine, sizeof(retryLine),
             "CAL: motion detected, retrying; mean=(%.3f,%.3f,%.3f) std=(%.3f,%.3f,%.3f)",
             meanX, meanY, meanZ, stdX, stdY, stdZ);
    debugLine(retryLine);
    delay(300);
  }

  char line[160];
  snprintf(line, sizeof(line), "CAL: done, bias(rad/s) X=%.5f Y=%.5f Z=%.5f",
           gyroBiasX, gyroBiasY, gyroBiasZ);
  debugLine(line);
}

void updateButton(ButtonState &button) {
  const bool reading = digitalRead(button.pin) == LOW;
  const uint32_t now = millis();

  if (reading != button.lastReading) {
    button.lastReading = reading;
    button.changedAt = now;
  }

  if ((now - button.changedAt) >= 25 && reading != button.stablePressed) {
    button.stablePressed = reading;
    if (!bleMouse.isConnected()) return;

    if (reading) {
      bleMouse.press(button.mouseButton);
    } else {
      bleMouse.release(button.mouseButton);
    }
  }
}

void updateSensitivityEncoder() {
  static int16_t partialSteps = 0;
  noInterrupts();
  const int16_t newSteps = encoderQuarterStepDelta;
  encoderQuarterStepDelta = 0;
  interrupts();
  partialSteps += newSteps;

  int16_t detents = 0;
  while (partialSteps >= 4) {
    partialSteps -= 4;
    ++detents;
  }
  while (partialSteps <= -4) {
    partialSteps += 4;
    --detents;
  }
  if (detents == 0) return;

  mouseGain += detents * ENCODER_GAIN_STEP * ENCODER_DIRECTION;
  mouseGain = constrain(mouseGain, MIN_MOUSE_GAIN, MAX_MOUSE_GAIN);
  char line[80];
  snprintf(line, sizeof(line), "SENS: encoder=%+d mouseGain=%.0f", detents, mouseGain);
  debugLine(line);
}

void updateAirButton() {
  const bool buttonPressed = digitalRead(AIR_BUTTON_PIN) == LOW;
  const uint32_t now = millis();

  // Lock immediately on press. Only the release needs debounce.
  if (buttonPressed) {
    airButtonLastReading = true;
    airButtonChangedAt = now;
    if (!airEnabled) return;

    airEnabled = false;
    pixelRemainderX = 0.0f;
    pixelRemainderY = 0.0f;
    kalmanX.reset();
    kalmanZ.reset();
    debugLine("AIR: button pressed, cursor movement locked");
    return;
  }

  if (airButtonLastReading) {
    airButtonLastReading = false;
    airButtonChangedAt = now;
    return;
  }
  if ((now - airButtonChangedAt) < 25 || airEnabled) return;

  airEnabled = true;
  pixelRemainderX = 0.0f;
  pixelRemainderY = 0.0f;
  kalmanX.reset();
  kalmanZ.reset();
  debugLine("AIR: button released, cursor movement enabled");
}

void setup() {
  Serial.begin(115200);
  DebugSerial.begin(115200, SERIAL_8N1, DEBUG_RX2_PIN, DEBUG_TX2_PIN);
  delay(300);
  debugLine("BOOT: ESP32 MPU6050 Air Mouse");

  pinMode(LEFT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(AIR_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  leftButton.lastReading = digitalRead(LEFT_BUTTON_PIN) == LOW;
  rightButton.lastReading = digitalRead(RIGHT_BUTTON_PIN) == LOW;
  airButtonLastReading = digitalRead(AIR_BUTTON_PIN) == LOW;
  encoderLastState =
      (digitalRead(ENCODER_A_PIN) << 1) | digitalRead(ENCODER_B_PIN);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), onEncoderChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_PIN), onEncoderChange, CHANGE);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  uint8_t detectionAttempts = 0;
  while (mpuAddress == 0) {
    if (supportedImuAt(0x68, mpuWhoAmI)) {
      mpuAddress = 0x68;
    } else if (supportedImuAt(0x69, mpuWhoAmI)) {
      mpuAddress = 0x69;
    } else {
      debugLine("ERROR: MPU6050 not found at 0x68/0x69; check 3V3/GND/SDA21/SCL22.");
      if ((detectionAttempts++ % 5) == 0) scanI2CBus();
      delay(1000);
    }
  }

  char addressLine[80];
  snprintf(addressLine, sizeof(addressLine),
           "MPU: found address=0x%02X WHO_AM_I=0x%02X", mpuAddress, mpuWhoAmI);
  debugLine(addressLine);
  if (!initializeImu()) {
    debugLine("ERROR: IMU register initialization failed");
    while (true) delay(1000);
  }

  calibrateGyroscope();
  kalmanX.reset();
  kalmanZ.reset();
  bleMouse.begin();
  debugLine("BLE: advertising; pair with 'ESP32 MPU6050 Air Mouse'.");
  lastSampleUs = micros();
}

void loop() {
  updateSensitivityEncoder();
  updateButton(leftButton);
  updateButton(rightButton);
  updateAirButton();

  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastSampleUs) < SAMPLE_PERIOD_US) return;

  const float dt = (nowUs - lastSampleUs) / 1000000.0f;
  lastSampleUs = nowUs;

  float rawGx, rawGy, rawGz;
  if (!readGyroscope(rawGx, rawGy, rawGz)) {
    debugLine("ERROR: IMU gyro read failed");
    return;
  }
  const float gx = rawGx - gyroBiasX;
  const float gy = rawGy - gyroBiasY;
  const float gz = rawGz - gyroBiasZ;

  const float filteredX = kalmanX.update(gx, dt);
  const float filteredZ = kalmanZ.update(gz, dt);

  const float horizontalRate = removeDeadZone(filteredX) * X_DIRECTION;
  const float verticalRate = removeDeadZone(filteredZ) * Y_DIRECTION;
  if (airEnabled) {
    pixelRemainderX += horizontalRate * mouseGain * dt;
    pixelRemainderY += verticalRate * mouseGain * dt;
  }

  int moveX = (int)pixelRemainderX;
  int moveY = (int)pixelRemainderY;
  moveX = constrain(moveX, -127, 127);
  moveY = constrain(moveY, -127, 127);
  pixelRemainderX -= moveX;
  pixelRemainderY -= moveY;

  if (!airEnabled) {
    moveX = 0;
    moveY = 0;
    pixelRemainderX = 0.0f;
    pixelRemainderY = 0.0f;
  }

  if (airEnabled && bleMouse.isConnected() && (moveX != 0 || moveY != 0)) {
    bleMouse.move((int8_t)moveX, (int8_t)moveY, 0);
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastDebugMs >= DEBUG_PERIOD_MS) {
    lastDebugMs = nowMs;
    char line[300];
    snprintf(line, sizeof(line),
             "DATA: BLE=%d AIR=%d gain=%.0f enc(A,B)=(%d,%d) edges=%lu dec=%ld raw(X,Z)=(%+.3f,%+.3f) kalman(X,Z)=(%+.3f,%+.3f) move=(%d,%d) btn(L,R)=(%d,%d)",
             bleMouse.isConnected(), airEnabled, mouseGain,
             digitalRead(ENCODER_A_PIN), digitalRead(ENCODER_B_PIN),
             (unsigned long)encoderEdgeCount, (long)encoderDecodedTotal,
             gx, gz, filteredX, filteredZ, moveX, moveY,
             leftButton.stablePressed, rightButton.stablePressed);
    debugLine(line);
  }
}

#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <cstring>
#include <string>


namespace homer1 {

const int MY_SIGNAL_PIN = 19;

std::vector<uint8_t> my_data{};
size_t j = 0;

volatile bool my_flag = true;

float sht3x_temperature = 0;
float sht3x_humidity = 0;

float bmp180_temperature = 0;
uint32_t bmp180_pressure = 0;

uint16_t pms5003_pm10_standard = 0;
uint16_t pms5003_pm25_standard = 0;
uint16_t pms5003_pm100_standard = 0;
uint16_t pms5003_pm10_env = 0;
uint16_t pms5003_pm25_env = 0;
uint16_t pms5003_pm100_env = 0;
uint16_t pms5003_pm03_particles = 0;
uint16_t pms5003_pm05_particles = 0;
uint16_t pms5003_pm10_particles = 0;
uint16_t pms5003_pm25_particles = 0;
uint16_t pms5003_pm50_particles = 0;
uint16_t pms5003_pm100_particles = 0;

uint16_t s8_co2 = 0;
uint16_t s8_days = 0;

bool check_frame() {
  if (my_data.size() >= 1024) {
    Serial.println("too much data");
    my_data.clear();
    return false;
  }

  // preamble: 5, ending: 5, len: 2, crc: 2 bytes.
  if (my_data.size() < 14) {
    Serial.println("not enough data");
    return false;
  }

  auto ok = my_data[0] == 'e'
            && my_data[1] == 'k'
            && my_data[2] == 'r'
            && my_data[3] == 'a'
            && my_data[4] == 'n'
            && my_data[my_data.size() - 1] == 'e'
            && my_data[my_data.size() - 2] == 'k'
            && my_data[my_data.size() - 3] == 'r'
            && my_data[my_data.size() - 4] == 'a'
            && my_data[my_data.size() - 5] == 'n';

  if (!ok) {
    Serial.println("bad data");
    my_data.clear();
  }

  return ok;
}

void log() {
  Serial.println();
  Serial.println("===========================================================");
  Serial.printf("sht3x_temperature: %f", sht3x_temperature);
  Serial.println();
  Serial.printf("sht3x_humidity: %f", sht3x_humidity);
  Serial.println();

  Serial.printf("bmp180_temperature: %f", bmp180_temperature);
  Serial.println();
  Serial.printf("bmp180_pressure: %u", bmp180_pressure);
  Serial.println();

  Serial.printf("s8_co2: %hu", s8_co2);
  Serial.println();
  Serial.printf("s8_days: %hu", s8_days);
  Serial.println();

  Serial.printf("pms5003_pm10_standard: %hu", pms5003_pm10_standard);
  Serial.println();
  Serial.printf("pms5003_pm25_standard: %hu", pms5003_pm25_standard);
  Serial.println();
  Serial.printf("pms5003_pm100_standard: %hu", pms5003_pm100_standard);
  Serial.println();
  Serial.printf("pms5003_pm10_env: %hu", pms5003_pm10_env);
  Serial.println();
  Serial.printf("pms5003_pm25_env: %hu", pms5003_pm25_env);
  Serial.println();
  Serial.printf("pms5003_pm100_env: %hu", pms5003_pm100_env);
  Serial.println();
  Serial.printf("pms5003_pm03_particles: %hu", pms5003_pm03_particles);
  Serial.println();
  Serial.printf("pms5003_pm05_particles: %hu", pms5003_pm05_particles);
  Serial.println();
  Serial.printf("pms5003_pm10_particles: %hu", pms5003_pm10_particles);
  Serial.println();
  Serial.printf("pms5003_pm25_particles: %hu", pms5003_pm25_particles);
  Serial.println();
  Serial.printf("pms5003_pm50_particles: %hu", pms5003_pm50_particles);
  Serial.println();
  Serial.printf("pms5003_pm100_particles: %hu", pms5003_pm100_particles);
  Serial.println();
}

void dump() {
  size_t xxx = 0;
  for (auto ddd : my_data) {
    Serial.print("d");
    if (xxx < 10)
      Serial.print("0");
    Serial.print(xxx++);
    Serial.print("= ");
    Serial.println(ddd);
  }
}

float read_float() {
  uint8_t read[] = { 0, 0, 0, 0 };
  read[0] = my_data[j];
  read[1] = my_data[j + 1];
  read[2] = my_data[j + 2];
  read[3] = my_data[j + 3];
  const float* read_back0 = reinterpret_cast<const float*>(read);
  j += 4;
  return *read_back0;
}

uint16_t read_uint16() {
  uint8_t read[] = { 0, 0 };
  read[0] = my_data[j];
  read[1] = my_data[j + 1];
  const uint16_t* read_back0 = reinterpret_cast<const uint16_t*>(read);
  j += 2;
  return *read_back0;
}

uint32_t read_uint32() {
  uint8_t read[] = { 0, 0, 0, 0 };
  read[0] = my_data[j];
  read[1] = my_data[j + 1];
  read[2] = my_data[j + 2];
  read[3] = my_data[j + 3];
  const uint32_t* read_back0 = reinterpret_cast<const uint32_t*>(read);
  j += 4;
  return *read_back0;
}

void act() {
  if (!check_frame())
    return;

  auto len = (my_data[5] << 8) | my_data[6];
  if (len == 0) {
    Serial.println("no sensor data");
    my_data.clear();
    return;
  }

  j = 7;
  while (j < (7 + len)) {
    switch (my_data[j]) {
      case 42:
        {
          Serial.println("reading pms5003");
          j++;
          pms5003_pm10_standard = read_uint16();
          pms5003_pm25_standard = read_uint16();
          pms5003_pm100_standard = read_uint16();

          pms5003_pm10_env = read_uint16();
          pms5003_pm25_env = read_uint16();
          pms5003_pm100_env = read_uint16();

          pms5003_pm03_particles = read_uint16();
          pms5003_pm05_particles = read_uint16();
          pms5003_pm10_particles = read_uint16();
          pms5003_pm25_particles = read_uint16();
          pms5003_pm50_particles = read_uint16();
          pms5003_pm100_particles = read_uint16();
        }
        break;

      case 43:
        {
          Serial.println("reading bmp180");
          j++;
          bmp180_temperature = read_float();
          bmp180_pressure = read_uint32();
        }
        break;

      case 44:
        {
          Serial.println("reading s8");
          j++;
          s8_co2 = read_uint16();
          s8_days = read_uint16();
        }
        break;

      case 45:
        {
          Serial.println("reading sht3x");
          j++;
          sht3x_temperature = read_float();
          sht3x_humidity = read_float();
        }
        break;

      default:
        Serial.print("unknown sensor, can not continue! sensor_id: ");
        Serial.print(my_data[j]);
        Serial.print(" at index: ");
        Serial.println(j);
        my_data.clear();
        return;
    }
  }

  my_data.clear();
}

void onRequest() {
  Wire.print(42);
  Serial.println("onRequest!!");
}

void onReceive(int len) {
  my_flag = false;
  digitalWrite(MY_SIGNAL_PIN, LOW);

  while (Wire.available())
    my_data.push_back((uint8_t)Wire.read());

  act();

  my_flag = true;
}

void signal() {
  digitalWrite(MY_SIGNAL_PIN, HIGH);
  delay(5);
  digitalWrite(MY_SIGNAL_PIN, LOW);
}

}

void setup() {
  Serial.begin(115200);
  pinMode(homer1::MY_SIGNAL_PIN, OUTPUT);
  Wire.onReceive(homer1::onReceive);
  Wire.onRequest(homer1::onRequest);
  Wire.begin((uint8_t)0x42);
}


void loop() {
  if (homer1::my_flag)
    homer1::signal();

  delay(3000);

  homer1::log();
}
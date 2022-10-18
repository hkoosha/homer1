#define MY_DISPLAY true


#ifdef MY_DISPLAY
#include "DEV_Config.h"
#include "EPD.h"
#include "GUI_Paint.h"
#endif


#include <cstdlib>
#include <vector>

#include <Wire.h>




namespace homer1 {

const int LOOP_DELAY_MILLIS = 60000;

const int MY_SIGNAL_PIN = 19;
const uint8_t MY_I2C_ADDR = 0x42;

std::vector<uint8_t> my_data{};
size_t j = 0;

volatile bool my_flag = true;

volatile bool connection = false;

uint16_t sgp30_tvoc = 0;
uint16_t sgp30_eco2 = 0;
uint16_t sgp30_raw_h2 = 0;
uint16_t sgp30_raw_ethanol = 0;


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


void reset_data() {
  sgp30_tvoc = 0;
  sgp30_eco2 = 0;
  sgp30_raw_h2 = 0;
  sgp30_raw_ethanol = 0;


  sht3x_temperature = 0;
  sht3x_humidity = 0;

  bmp180_temperature = 0;
  bmp180_pressure = 0;

  pms5003_pm10_standard = 0;
  pms5003_pm25_standard = 0;
  pms5003_pm100_standard = 0;
  pms5003_pm10_env = 0;
  pms5003_pm25_env = 0;
  pms5003_pm100_env = 0;
  pms5003_pm03_particles = 0;
  pms5003_pm05_particles = 0;
  pms5003_pm10_particles = 0;
  pms5003_pm25_particles = 0;
  pms5003_pm50_particles = 0;
  pms5003_pm100_particles = 0;

  s8_co2 = 0;
  s8_days = 0;
}

bool check_frame() {
  if (my_data.size() >= 1024) {
    printf("too much data\r\n");
    my_data.clear();
    return false;
  }

  // preamble: 5, ending: 5, len: 2, crc: 2 bytes.
  if (my_data.size() < 14) {
    printf("not enough data\r\n");
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
    printf("bad data\r\n");
    my_data.clear();
  }

  return ok;
}

void log() {
  printf("===========================================================\r\n");
  printf("sht3x_temperature: %f\r\n", sht3x_temperature);
  printf("sht3x_humidity: %f\r\n", sht3x_humidity);

  printf("sgp30_tvoc: %f\r\n", sgp30_tvoc);
  printf("sgp30_eco2: %f\r\n", sgp30_eco2);
  printf("sgp30_raw_h2: %f\r\n", sgp30_raw_h2);
  printf("sgp30_raw_ethanol: %f\r\n", sgp30_raw_ethanol);

  printf("bmp180_temperature: %f\r\n", bmp180_temperature);
  printf("bmp180_pressure: %u\r\n", bmp180_pressure);

  printf("s8_co2: %hu\r\n", s8_co2);
  printf("s8_days: %hu\r\n", s8_days);

  printf("pms5003_pm10_standard: %hu\r\n", pms5003_pm10_standard);
  printf("pms5003_pm25_standard: %hu\r\n", pms5003_pm25_standard);
  printf("pms5003_pm100_standard: %hu\r\n", pms5003_pm100_standard);
  printf("pms5003_pm10_env: %hu\r\n", pms5003_pm10_env);
  printf("pms5003_pm25_env: %hu\r\n", pms5003_pm25_env);
  printf("pms5003_pm100_env: %hu\r\n", pms5003_pm100_env);
  printf("pms5003_pm03_particles: %hu\r\n", pms5003_pm03_particles);
  printf("pms5003_pm05_particles: %hu\r\n", pms5003_pm05_particles);
  printf("pms5003_pm10_particles: %hu\r\n", pms5003_pm10_particles);
  printf("pms5003_pm25_particles: %hu\r\n", pms5003_pm25_particles);
  printf("pms5003_pm50_particles: %hu\r\n", pms5003_pm50_particles);
  printf("pms5003_pm100_particles: %hu\r\n", pms5003_pm100_particles);
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

void process_data() {
  connection = true;

  if (!check_frame())
    return;

  auto len = (my_data[5] << 8) | my_data[6];
  if (len == 0) {
    printf("no sensor data\r\n");
    my_data.clear();
    return;
  }
  printf("got sensor data\r\n");

  j = 7;
  while (j < (7 + len)) {
    switch (my_data[j]) {
      case 42:
        {
          // printf("reading pms5003\r\n");
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
          // printf("reading bmp180\r\n");
          j++;
          bmp180_temperature = read_float();
          bmp180_pressure = read_uint32();
        }
        break;

      case 44:
        {
          // printf("reading s8\r\n");
          j++;
          s8_co2 = read_uint16();
          s8_days = read_uint16();
        }
        break;

      case 45:
        {
          // printf("reading sht3x\r\n");
          j++;
          sht3x_temperature = read_float();
          sht3x_humidity = read_float();
        }
        break;

      case 46:
        {
          // printf("reading sgp30\r\n");
          j++;
          sgp30_tvoc = read_uint16();
          sgp30_eco2 = read_uint16();
          sgp30_raw_h2 = read_uint16();
          sgp30_raw_ethanol = read_uint16();
        }
        break;

      default:
        printf("unknown sensor, can not continue! sensor_id: %d at index: %d", my_data[j], j);
        my_data.clear();
        return;
    }
  }

  my_data.clear();
}

void on_request() {
  Wire.print(42);
  printf("onRequest!!\r\n");
}

void my_draw();

void on_receive(int len) {
  my_flag = false;
  digitalWrite(MY_SIGNAL_PIN, LOW);

  while (Wire.available())
    my_data.push_back((uint8_t)Wire.read());

  process_data();
  log();
  my_draw();

  my_flag = true;
}

void signal() {

  if (!my_flag)
    return;

  digitalWrite(MY_SIGNAL_PIN, HIGH);
  delay(5);
  digitalWrite(MY_SIGNAL_PIN, LOW);
}



#ifdef MY_DISPLAY
uint8_t* black_image;
char* print_buffer;
const size_t print_buffer_len = 1024;
#endif

void my_init_and_clear() {
  EPD_7IN5_V2_Init();
  Paint_NewImage(black_image, EPD_7IN5_V2_WIDTH, EPD_7IN5_V2_HEIGHT, 0, WHITE);
  Paint_SelectImage(black_image);
  Paint_Clear(WHITE);
}

void my_flush_and_sleep() {
  EPD_7IN5_V2_Display(black_image);
  DEV_Delay_ms(2000);
  printf("display sleep\r\n");
  EPD_7IN5_V2_Sleep();
}

size_t my_write_int32(char* my, int32_t n) {
  if (n == 0)
    my[0] = '-';
  else
    itoa(n, my, 10);
  return strlen(my);
}

size_t my_write_uint32(char* my, uint32_t n) {
  if (n == 0)
    my[0] = '-';
  else
    utoa(n, my, 10);

  return strlen(my);
}

void my_write_float(char* my, float f) {
  if (f == +0 || f == -0 || f == 0) {
    my[0] = '-';
    return;
  }

  auto r = (int32_t)f;
  auto d = (int32_t)((f - r) * 100);

  size_t at = my_write_int32(my, r);
  my[at] = '.';
  my_write_int32(my + at + 1, d);
}

size_t reset_print_buffer(const char* label) {
#ifdef MY_DISPLAY
  memset(print_buffer, 0, print_buffer_len);
  strcpy(print_buffer, label);
#endif
  return strlen(label);
}

void print_single_msg(const char* msg) {
#ifdef MY_DISPLAY
  my_init_and_clear();
  reset_print_buffer(msg);
  Paint_DrawString_EN(10, 10, print_buffer, &Font16, BLACK, WHITE);
  my_flush_and_sleep();
#endif
}

void print_key_value_f(uint16_t root,
                       uint16_t base,
                       const char* label,
                       float value) {
  auto at = reset_print_buffer(label);
  my_write_float(print_buffer + at, value);
  Paint_DrawString_EN(root, base, print_buffer, &Font16, WHITE, BLACK);
}

void print_key_value(uint16_t root,
                     uint16_t base,
                     const char* label,
                     uint32_t value) {
  auto at = reset_print_buffer(label);
  my_write_uint32(print_buffer + at, value);
  Paint_DrawString_EN(root, base, print_buffer, &Font16, WHITE, BLACK);
}


void my_draw() {
#ifdef MY_DISPLAY
  my_init_and_clear();

  size_t at = 0;
  uint16_t base = 0;
  uint16_t root = 0;

  root = 10;

  base = 10;
  print_key_value_f(root, base, "Humidity: ", sht3x_humidity);

  base += 20;
  print_key_value_f(root, base, "Temp:     ", sht3x_temperature);

  base += 20;
  print_key_value(root, base, "Pressure: ", bmp180_pressure);

  base += 20;
  print_key_value(root, base, "CO2:      ", s8_co2);

  base += 20;
  print_key_value(root, base, "eCO2:     ", sgp30_eco2);

  base += 20;
  print_key_value(root, base, "TVOC:     ", sgp30_tvoc);

  base += 20;
  print_key_value(root, base, "H2:       ", sgp30_raw_h2);

  base += 20;
  print_key_value(root, base, "Ethanol:  ", sgp30_raw_ethanol);

  // ============================

  root = 200;

  base = 10;
  print_key_value(root, base, "PM10 -  ENV: ", pms5003_pm10_env);

  base += 20;
  print_key_value(root, base, "PM25 -  ENV: ", pms5003_pm25_env);

  base += 20;
  print_key_value(root, base, "PM100 - ENV: ", pms5003_pm100_env);

  root = 400;

  base = 10;
  print_key_value(root, base, "PM10 -  STD: ", pms5003_pm10_standard);

  base += 20;
  print_key_value(root, base, "PM25 -  STD: ", pms5003_pm25_standard);

  base += 20;
  print_key_value(root, base, "PM100 - STD: ", pms5003_pm100_standard);


  root = 600;

  base = 10;
  print_key_value(root, base, "PM03 -  PTC: ", pms5003_pm03_particles);

  base += 20;
  print_key_value(root, base, "PM05 -  PTC: ", pms5003_pm05_particles);

  base += 20;
  print_key_value(root, base, "PM10 -  PTC: ", pms5003_pm10_particles);

  base += 20;
  print_key_value(root, base, "PM25 -  PTC: ", pms5003_pm25_particles);

  base += 20;
  print_key_value(root, base, "PM50 -  PTC: ", pms5003_pm50_particles);

  base += 20;
  print_key_value(root, base, "PM100 -  PTC: ", pms5003_pm100_particles);

  my_flush_and_sleep();

#endif
}

}



void setup() {
  printf("setup...\r\n");

#ifdef MY_DISPLAY

  printf("init display\r\n");
  DEV_Module_Init();
  EPD_7IN5_V2_Init();

  printf("clearing display\r\n");
  EPD_7IN5_V2_Clear();
  delay(500);

  auto w = EPD_7IN5_V2_WIDTH;
  auto h = EPD_7IN5_V2_HEIGHT;
  uint16_t image_size = ((w % 8 == 0) ? (w / 8) : (w / 8 + 1)) * h;
  homer1::black_image = new uint8_t[image_size];
  if (!homer1::black_image)
    throw std::runtime_error("failed to allocate black memory");

  homer1::print_buffer = new char[homer1::print_buffer_len];
  if (!homer1::print_buffer)
    throw std::runtime_error("failed to allocate print buffer");

  homer1::print_single_msg("loading, may take up to two minutes...");

#endif

  pinMode(homer1::MY_SIGNAL_PIN, OUTPUT);
  Wire.onReceive(homer1::on_receive);
  Wire.onRequest(homer1::on_request);
  Wire.begin(homer1::MY_I2C_ADDR);
}

void loop() {
  printf("looper\r\n");

  homer1::connection = false;
  homer1::reset_data();
  homer1::signal();
  delay(1000);
  if (!homer1::connection) {
    printf("did not recieve any data after signal");
    homer1::print_single_msg("connection error");
  }

  delay(homer1::LOOP_DELAY_MILLIS);
}


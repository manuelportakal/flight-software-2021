//libraries
#include <WiFi.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <DHT.h>
#include <ESP32Servo.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <HTTPClient.h>
#include "time.h"
#include "ESPAsyncWebServer.h"

//start - wifi config
const char *ssid = "TurkTelekom_T5AA1";
const char *password = "NCxw4ax3";

IPAddress local_IP(192, 168, 1, 184);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);
IPAddress secondaryDNS(8, 8, 4, 4);
//end - wifi config

//start - send package
typedef struct package_structure {
  unsigned short int package_number, turns_number = 0;
  float altitude = 0, acceleration, temperature, battery_voltage, pitch, roll, yaw, pressure;
  double gps_latitude, gps_longitude, gps_altitude;
  bool video_status = 0;
  String team_no = "42489", satellite_status = "Landing";
  char mission_time[32];
} package_type;

package_type package;
package_type *pck;
//end - send package

//start - eeprom
const unsigned short int adres_paket_numarasi = 0;
const unsigned short int adres_durum = 10;
const unsigned short int adres_donus_sayisi = 20;
const unsigned short int adres_video_durumu = 30;
const unsigned short int adres_restart_durumu = 40;
const unsigned short int adres_t1_sayaci = 50;
const unsigned short int adres_t3_sayaci = 60;
unsigned long t1 = 0, t2 = 0, t3 = 0, t4 = 0;
//end - eeprom

unsigned short int status, restart_flag;

const char *serverName = "http://192.168.0.106:5000/Telemetry";

const String seperationPass = "seperation12345";

//start - mpu650
Adafruit_MPU6050 mpu;
//end - mpu650

//start - bmp180
Adafruit_BMP085 bmp;
//end - bmp180

//start - buzzer
const unsigned short int BUZZER_PIN = 27;
//end - buzzer

//start - max471
const unsigned short int MAX471_PIN = 34;
//end - max471

//start - servo
const unsigned short int SERVO_PIN = 14;
Servo myservo;
//end - servo

//start - sdcard
const unsigned short int SDCARD_PIN = 5;
//end - sdcard

//start - gps
const unsigned short int RXPin = 16, TXPin = 17;
const unsigned short int GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial SerialGPS(RXPin, TXPin);
//end - gps

//start - rtc
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 7200;
const int   daylightOffset_sec = 3600;
struct tm timeinfo;
//end - rtc

//start - webserver
AsyncWebServer server(80);
//end - webserver

//functions
void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final);
void startSeperation(void);

//init functions
void initWifi(void);
void initMpu6050(void);
void initBuzzer(void);
void initBmp180(void);
void initMax471(void);
void initServo(void);
void initGps(void);
void initSdCard(void);
void initRtc(void);
void initWebServer(void);

void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
  if(!index){
    Serial.printf("UploadStart: %s\n", filename.c_str());
  }
    File file = SD.open("/" + filename, FILE_APPEND);
  //appendFile(SD, "/temp.jpg", *data);
  for(size_t i=0; i<len; i++){
    file.write(data[i]);
  }
     file.close();
  if(final){
    Serial.printf("UploadEnd: %s, %u B\n", filename.c_str(), index+len);
    package.video_status = 1;
  }
}

void initWifi() {
  Serial.println("Connecting to Wifi");

  WiFi.disconnect();
  WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println(WiFi.SSID() + " connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void initMpu6050() {
  Serial.println("Connecting to MPU6050");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 connected!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

void initBmp180() {
  Serial.println("Connecting to Bmp180");

  if (!bmp.begin()) {
    Serial.println("Failed to find Bmp180 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Bmp180 connected!");
}

void initBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
}

void initMax471() {
  pinMode(MAX471_PIN, INPUT);
}

void initServo() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);
  myservo.attach(SERVO_PIN, 500, 2400);
  myservo.write(60);
}

void initGps() {
  Serial.println("Connecting to Gps");

  SerialGPS.begin(GPSBaud);
  Serial.println("Gps connected!");

  Serial.println("Connecting to satellites");

  while (gps.satellites.value() < 1) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("Satellite connected!");
}

void initSdCard() {
  Serial.println("Connecting to SD Card Module");

  if (!SD.begin(SDCARD_PIN)) {
    Serial.println("Card Mount Failed");
  }
  Serial.println("SD Card Module connected!");

  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
  }
}

void initRtc(){
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

void initWebServer(){
  server.on("/post",HTTP_POST,[](AsyncWebServerRequest * request){},NULL,[](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
    String receiveData = String((char *)data);

    if(receiveData.equals(seperationPass)){
      startSeperation();
    }
 
    request->send(200);
  });

  server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request){
    request->send(200);
  }, handleUpload);
 
  server.begin();
}

void setup() {
  Serial.begin(115200);
  delay(100);
  EEPROM.begin(512);
  delay(100);
  initWifi();
  initMpu6050();
  initBuzzer();
  initBmp180();
  initMax471();
  initServo();
  initGps();
  initSdCard();
  initRtc();
  initWebServer();
}

void loop() {
  ReadEEPROM();
  ReadSensorData();
  SatelliteControl();
  SatelliteStateCalculate();
  writeEEPROM();
  String data = PrepareData();
  writeSdCard(SD, "/telemetries.txt", data);
  SendData(data);
  delay(1000);
}

void ReadEEPROM() {
  if (EEPROM.get(adres_paket_numarasi, package.package_number) != NULL) {
    package.package_number++;
  } else
    package.package_number = 1;

  if (EEPROM.read(adres_donus_sayisi) != NULL) {
    package.turns_number = EEPROM.read(adres_donus_sayisi);
  } else
    package.turns_number = 0;

  if (EEPROM.read(adres_video_durumu) != NULL) {
    package.video_status = EEPROM.read(adres_video_durumu);
  } else
    package.video_status = false;

  if (EEPROM.read(adres_durum) != NULL) {
    status = EEPROM.read(adres_durum);
  } else
    status = 0;

  if (EEPROM.read(adres_t1_sayaci) != NULL) {
    EEPROM.get(adres_t1_sayaci, t1);
  }

  if (EEPROM.read(adres_t3_sayaci) != NULL) {
    EEPROM.get(adres_t3_sayaci, t3);
    if (t2 < t3) {
      restart_flag = true;
      EEPROM.put(adres_restart_durumu, restart_flag);
      EEPROM.commit();
      Serial.println("Sistem yeniden baslatildi");
    }
  }
}

void ReadSensorData() {
  GetMax471();
  GetGPS();
  GetMpu6050();
  GetBmp180();
  GetRtc();
}

void SatelliteControl() {
  if (package.altitude >= 690 && status == 0) {
    status = 1;
    //inise geciliyor;
  }

  else if (package.altitude <= 430 && status == 1) {
    status = 2;
    //startSeperation();
    //otonom ayrilma gerceklesti;
  }

  else if (package.altitude <= 410 && package.altitude > 250 && status == 2) {
    //startMotors();
  }

  else if (package.altitude <= 250 && (status == 2 || status == 3)) {
    //startFixAltitude();
    //irtifa sabitleme baslatildi;

    if (status == 2) {
      status = 3;
      t1 = millis();
      EEPROM.put(adres_t1_sayaci, t1);
    }

    else if (status == 3 && restart_flag == 0) {
      t2 = millis();
      t4 = t2 - t1;
    }

    else {
      t2 = millis();
      t4 = (t2 + t3) - t1;
    }

    if (t4 >= 10000) {
      status = 4;

      //zaman sayaclarini sifirlar
      restart_flag = 0;
      t1 = 0, t2 = 0, t3 = 0, t4 = 0;
      EEPROM.put(adres_restart_durumu, restart_flag);
      EEPROM.put(adres_t1_sayaci, '\0');
      EEPROM.put(adres_t3_sayaci, '\0');
      //stopFixAltitude();
      //irtifa sabitleme mekanizmasini devre disi birakildi
      //inis devam ediyor;
    }
  }

  else if ((package.altitude <= 10) && (status == 4 || status == 5)) {

    if (status == 4) {
      status = 5;
      t1 = millis();
      EEPROM.put(adres_t1_sayaci, t1);
      startBuzzer();
      //inis gerceklesti;
    }

    else if (status == 5 && restart_flag == 0) {
      t2 = millis();
      t4 = t2 - t1;
    }

    else {
      t2 = millis();
      t4 = (t2 + t3) - t1;
    }

    if (t4 >= 120000) {
      status = 6;
      Serial.println("Sistem kapatiliyor..");
      //powerOff();
      package.satellite_status = "Sistem kapatildi";
    }
  }

  EEPROM.commit();
}

void SatelliteStateCalculate() {

  switch (status) {
    case 0:
      package.satellite_status = "Uydu Yukseliyor";
      break;
    case 1:
      package.satellite_status = "Inise Geciliyor";
      break;
    case 2:
      package.satellite_status = "Ayrilma Gerceklesti - iniliyor";
      break;
    case 3:
      package.satellite_status = "Irtifa sabitlendi";
      break;
    case 4:
      package.satellite_status = "Inise devam ediliyor";
      break;
    case 5:
      package.satellite_status = "Inis gerceklesti";
      break;
    case 6:
      package.satellite_status = "Sistem Kapatildi";
      while (1) {
      };
      break;
  }
}

void writeEEPROM() {
  EEPROM.put(adres_paket_numarasi, package.package_number);
  EEPROM.put(adres_donus_sayisi, package.turns_number);
  EEPROM.put(adres_video_durumu, package.video_status);
  EEPROM.put(adres_restart_durumu, restart_flag);
  EEPROM.put(adres_durum, status);
  EEPROM.put(adres_t1_sayaci, t1);

  if (status == 3 || status == 5)
    EEPROM.put(adres_t3_sayaci, t2);

  EEPROM.commit();
}

void SendData(String httpRequestData) {
  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;

    http.begin(client, serverName);
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(httpRequestData);
    //Serial.println("HTTP Response code: " + httpResponseCode);

    http.end();
  } else {
    Serial.println("WiFi Disconnected");
  }
}

String PrepareData() {
  DynamicJsonDocument readings(512);
  readings["TeamNo"] = String(package.team_no);
  readings["PackageNumber"] = String(package.package_number);
  readings["TurnsNumber"] = String(package.turns_number);
  readings["Altitude"] = String(package.altitude);
  readings["Acceleration"] = String(package.acceleration);
  readings["Temperature"] = String(package.temperature);
  readings["Battery_Voltage"] = String(package.battery_voltage);
  readings["Pitch"] = String(package.pitch);
  readings["Roll"] = String(package.roll);
  readings["Yaw"] = String(package.yaw);
  readings["Pressure"] = String(package.pressure);
  readings["GpsLatitude"] = String(package.gps_latitude);
  readings["GpsLongitude"] = String(package.gps_longitude);
  readings["GpsAltitude"] = String(package.gps_altitude);
  readings["SatelliteStatus"] = String(package.satellite_status);
  readings["VideoStatus"] = String(package.video_status);
  readings["SendTime"] = String(package.mission_time);

  String httpRequestData;
  serializeJson(readings, httpRequestData);
  Serial.println(httpRequestData);

  return httpRequestData;
}

void writeSdCard(fs::FS &fs, const char * path, String message) {
  //Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    //Serial.println("Message appended");
  }
  else {
    Serial.println("Append failed");
  }
  file.close();
}

void GetMax471() {
  int raw_value = analogRead(MAX471_PIN);
  float current = (raw_value * 5.0 ) / 1024.0;
  package.battery_voltage = current;
}

void GetGPS() {
  if (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());

    if (gps.location.isValid()) {
      package.gps_latitude = gps.location.lat();
      package.gps_longitude = gps.location.lng();
    }

    if (gps.altitude.isValid()) {
      package.gps_altitude = gps.altitude.meters();
    }
  }
}

void GetMpu6050() {
  // package.pitch = random(0, 36000) / 100.0;
  // package.roll = random(0, 36000) / 100.0;
  // package.yaw = random(0, 36000) / 100.0;
  // package.acceleration = random(500, 1000) / 100.0;


  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //rad/s
  package.pitch = g.gyro.x;
  package.roll = g.gyro.y;
  package.yaw = g.gyro.z;

  //m/s^2
  package.acceleration = a.acceleration.y;

  //degC
  //package.temperature = temp.temperature;
}

void GetBmp180() {
  //degC
  package.temperature = bmp.readTemperature();

  //Pa
  package.pressure = bmp.readPressure();

  //meters
  package.altitude = bmp.readAltitude();

  //read altitude with pressure parameter
  //package.altitude = bmp.readAltitude(101550);

  //Pa
  //Serial.print(bmp.readSealevelPressure());
}

void GetRtc(){
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  //Serial.println(&package.timeinfo, "%A, %B %d %Y %H:%M:%S");
  strftime(package.mission_time, sizeof(package.mission_time), "%FT%TZ", &timeinfo);
}

void startBuzzer() {
  digitalWrite(BUZZER_PIN, HIGH);
}

void startSeperation() {
  myservo.write(0);
}
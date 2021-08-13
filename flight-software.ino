//libraries
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "time.h"

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
  unsigned int package_number, turns_number = 0;
  float altitude = 0, acceleration, temperature, battery_voltage, pitch, roll, yaw;
  double pressure, gps_latitude, gps_longitude, gps_altitude;
  bool video_status = 0;
  String team_no = "42489", satellite_status = "Landing";
  char mission_time[32];
} package_type;

package_type package;
package_type *pck;
//end - send package

//start - eeprom
const int adres_paket_numarasi = 0;
const int adres_durum = 10;
const int adres_donus_sayisi = 20;
const int adres_video_durumu = 30;
const int adres_restart_durumu = 40;
const int adres_t1_sayaci = 50;
const int adres_t3_sayaci = 60;
unsigned long t1 = 0, t2 = 0, t3 = 0, t4 = 0;
//end - eeprom

//start - rtc
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 7200;
const int   daylightOffset_sec = 3600;
struct tm timeinfo;
//end - rtc

short int status, restart_flag;

const char *serverName = "http://192.168.1.108:5000/Telemetry";

//functions
void ReadEEPROM(void);
void initWifi(void);
void initSdCard(void);
void initRtc(void);

void initWifi() {
  Serial.println("Connecting");

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

void initSdCard(){
  Serial.println("Connecting to SD Card Module");

  if(!SD.begin()){
    Serial.println("Card Mount Failed");
  }
  Serial.println("SD Card Module connected!");
  
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
  }
}

void initRtc(){
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  initWifi();
  initSdCard();
  initRtc();

  EEPROM.begin(512);
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
  //ReadIncomingData();
  delay(3000);
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
  GetDHT11();
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
      //startBuzzer();
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

    Serial.println(httpRequestData);

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

void writeSdCard(fs::FS &fs, const char * path, const String message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
    //Serial.println("Message appended");
  } 
  else {
    Serial.println("Append failed");
  }
  file.close();
}

void GetMax471() {
  package.battery_voltage = random(300, 1000) / 100.0;
}

void GetGPS() {
  package.gps_latitude = random(3600, 4200) / 100.0;
  package.gps_longitude = random(2600, 4500) / 100.0;
  package.gps_altitude = random(0, 70000) / 100.0;
  //package.gps_latitude = 40.904798;
  //package.gps_longitude = 31.181144;
  //package.gps_altitude = 195.80;
}

void GetDHT11() {
  package.temperature = random(2000, 3000) / 100.0;
}

void GetMpu6050() {
  package.pitch = random(-300, 300) / 100.0;
  package.roll = random(-300, 300) / 100.0;
  package.yaw = random(-100, 100) / 100.0;
  package.acceleration = random(500, 1000) / 100.0;
}

void GetBmp180() {
  package.pressure = random(0, 1000) / 100.0;

  //irtifa sabitleme pasifken calisir
  if (status != 3) {
    if (package.altitude < 700 && status == 0)
      package.altitude += 10;
    else if (package.altitude >= 70)
      package.altitude -= 7;
    else
      package.altitude = 0;
  }
}

void GetRtc(){
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  //Serial.println(&package.timeinfo, "%A, %B %d %Y %H:%M:%S");
  strftime(package.mission_time, sizeof(package.mission_time), "%FT%TZ", &timeinfo);
}
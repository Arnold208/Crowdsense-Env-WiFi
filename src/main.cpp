#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <Wire.h>
#include "SDCardManager/SDCardManager.h"
#include "SettingsManager/SettingsManager.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "Adafruit_VEML7700.h"
#include "RTClib.h"
#include <SoftwareSerial.h>
#include "WiFi.h"
#include <AsyncTCP.h>
#include <DNSServer.h>
#include "ESPAsyncWebServer.h"
#include <HTTPClient.h>
#include "Adafruit_LTR390.h"

#define SD_CS_PIN 5
#define PWR_CTRL 2
// #define BAT 17
#define RX 16
#define TX 17
#define MIC 33
#define EID "2e4"

SDCardManager sdCardManager;
SettingsManager settingsManager(SD);
Adafruit_BME680 bme;
Adafruit_VEML7700 veml = Adafruit_VEML7700();
Adafruit_LTR390 ltr = Adafruit_LTR390();
RTC_DS1307 rtc;
DNSServer dnsServer;
AsyncWebServer server(80);

String networkList;
const int maxRetries = 3;
const int retryTimeout = 10000;
bool startDevice = false;
bool sDmode = false;

bool bmeMode = false;
bool ltrMode = false;
bool luxMode = false;
bool rtcMode = false;
bool cardMode = false;
String Status = "";

String url = "https://cctelemetry-dev.azurewebsites.net/telemetry";

const char *settingsFilePath = "/preferences/pref.json";
const char *directoryPath = "/data";
const char *filePath = "/data/data.json";
const char *wifiPath = "/WiFimanager/config.json";

JsonDocument currentSettings;
JsonDocument telemetryData;
JsonDocument config;

SoftwareSerial pmsSerial(RX, TX);
float micSensitivity = -26;

struct pms5003data
{
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data data;

boolean readPMSdata(Stream *s)
{
  if (!s->available())
  {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42)
  {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32)
  {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i = 0; i < 30; i++)
  {
    sum += buffer[i];
  }

  /* debugging
    for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
    }
    Serial.println();
  */

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++)
  {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum)
  {
    Serial.println("Checksum failure");
    ESP.restart();
    return false;
  }
  // success!
  return true;
}

float getPeakToPeakVoltage()
{
  unsigned long sampleTime = 50; // Sample for 50 milliseconds
  int maxReading = 0;            // Store maximum reading
  int minReading = 4095;         // Store minimum reading (4095 for 12-bit ADC)
  unsigned long startTime = millis();

  // Collect data for the sample window dura  tion
  while (millis() - startTime < sampleTime)
  {
    int reading = analogRead(MIC);

    if (reading > maxReading)
    {
      maxReading = reading;
    }
    if (reading < minReading)
    {
      minReading = reading;
    }
  }

  // Assuming a 12-bit ADC with a reference of 3.3V
  float maxVoltage = maxReading * (3.3 / 4095.0);
  float minVoltage = minReading * (3.3 / 4095.0);

  // Peak-to-peak voltage is the difference between max and min voltages
  return maxVoltage - minVoltage;
}

void MICP()
{
  unsigned long startMillis = millis();
  unsigned long currentMillis;
  float maxPeakToPeakVoltage = 0;

  Serial.println("Please make a sound or clap for 5 seconds.");

  while (millis() - startMillis < 5000)
  { // 5-second sampling period
    currentMillis = millis();

    float peakToPeakVoltage = getPeakToPeakVoltage();
    if (peakToPeakVoltage > maxPeakToPeakVoltage)
    {
      maxPeakToPeakVoltage = peakToPeakVoltage;
    }
  }

  // After 5 seconds of sampling
  if (maxPeakToPeakVoltage == 0)
  {
    Serial.println("Sound Level: Below detection threshold");
    telemetryData["s"] = 0;

    printf("Microphone Test", false);
  }
  else
  {
    float rmsVoltage = maxPeakToPeakVoltage / (2 * sqrt(2)); // Convert to RMS
    float sensitivityLinear = pow(10, micSensitivity / 20);
    float pressurePascals = rmsVoltage / sensitivityLinear;
    float soundLevelDB = 20 * log10(pressurePascals / 20e-6);

    Serial.print("Sound Level: ");
    Serial.print(soundLevelDB);
    telemetryData["s"] = soundLevelDB;

    Serial.println(" dB");
    printf("Microphone Test", true);
  }
}

void scan()
{
  int numberOfNetworks = WiFi.scanNetworks();

  Serial.println("Scan complete.");
  if (numberOfNetworks == 0)
  {
    Serial.println("No networks found.");
  }
  else
  {
    Serial.print(numberOfNetworks);
    Serial.println(" networks found:");

    String json = "{ \"networks\": [";

    for (int i = 0; i < numberOfNetworks; ++i)
    {
      // Append the SSID to the JSON string
      json += "\"" + WiFi.SSID(i) + "\"";
      if (i < numberOfNetworks - 1)
      {
        json += ", "; // If it's not the last network, add a comma
      }
    }

    json += "]}";

    networkList = json;
    Serial.println("JSON string of SSIDs:");
    Serial.println(networkList);
  }
  // Wait a bit before scanning again
  delay(2000);
}

void connectWiFi(String s, String p)
{
  Serial.println("Connecting to WIFI");
  int retries = 0;
  while (retries < maxRetries)
  {
    Serial.printf("Connecting to WiFi... Retry %d\n", retries + 1);
    WiFi.begin(s, p);

    delay(300);
    // Wait for WiFi connection or timeout
    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED && timeout < retryTimeout)
    {
      delay(500);
      timeout += 500;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("Connected to WiFi!");
      startDevice = true;
      break; // Exit the retry loop if connected
    }
    else
    {
      Serial.println("Connection failed!");
      retries++;
    }
  }
  if (retries == maxRetries)
  {
    Serial.println("Max retry attempts reached. Could not connect to WiFi.");
    Serial.println("Switching to SD Card Mode");
    // settingsManager.updateWiFi(wifiPath,"", "");
    // delay(1000);
    // ESP.restart();
    sDmode = true;
    delay(1000);
  }
}

class CaptiveRequestHandler : public AsyncWebHandler
{
public:
  CaptiveRequestHandler() {}
  virtual ~CaptiveRequestHandler() {}

  bool canHandle(AsyncWebServerRequest *request)
  {
    return true;
  }

  void handleRequest(AsyncWebServerRequest *request)
  {
    if (request->url() == "/config")
    {

      if (request->method() == HTTP_GET)
      {

        if (SD.exists("/WiFimanager/index.html"))
        {
          request->send(SD, "/WiFimanager/index.html", "text/html");
        }

        else
        {
          request->send(200, "text/plain", "Config file not found on SD card");
        }
      }
    }

    else if (request->url() == "/get-networks")
    {
      request->send(200, "application/json", networkList);
    }

    else if (request->url() == "/param" && request->method() == HTTP_GET)
    {
      String ssid = request->arg("ssid");
      String password = request->arg("pass"); // Change to "pass" to match the HTML form
      Serial.println(" ");
      Serial.println("Received SSID: " + ssid);
      // Serial.println("Received Password: " + password);

      if (SD.exists("/WiFimanager/config.json"))
      {
        Serial.println("File Dey Inside");
        settingsManager.updateWiFi(wifiPath, ssid, password);
      }

      delay(500);

      connectWiFi(ssid, password);

      request->send(200, "text/plain", "Configuration successful");
    }
    else
    {
      if (SD.exists("/WiFimanager/wifimanger.html"))
      {
        request->send(SD, "/WiFimanager/wifimanger.html", "text/html");
      }
      else
      {
        request->send(200, "text/plain", "File not found on SD card");
      }
    }
  }
};

void setRTCDateTime(String response) {
  // Extract server time from response
  StaticJsonDocument<200> doc;
  deserializeJson(doc, response);
  long serverTime = doc["serverTime"];

  if (serverTime != 0) {
    // Set RTC with the server time
    rtc.adjust(DateTime(serverTime));
    Serial.println("RTC updated with server time");
  } else {
    Serial.println("Server time is 0, RTC not updated");
  }
}

void setup()
{
  Serial.begin(9600);
  pmsSerial.begin(9600);
  pinMode(PWR_CTRL, OUTPUT);
  // pinMode(BAT, INPUT);

  delay(25);

  digitalWrite(PWR_CTRL, HIGH);

  delay(1000);

  if (!SD.begin(SD_CS_PIN))
  {
    Serial.println("SD card initialization failed.");
    return;
  }
  else
  {
    settingsManager.readSettings(wifiPath, config);
    String con = config["ssid"];
    const char *ssid1 = config["ssid"];
    const char *password1 = config["password"];

    if (con == "" || con == " ")
    {

      Serial.println("No Access Point Credentials Found");

      WiFi.softAP("CrowdSense-2e4");
      dnsServer.start(53, "*", WiFi.softAPIP());
      server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER); // Only when requested from AP
      server.begin();
      scan();
    }

    else
    {
      connectWiFi(ssid1, password1);
      delay(1000);
    }

    if (!SD.exists(settingsFilePath))
    {
      JsonDocument jsonTemplate;
      deserializeJson(jsonTemplate, settingsManager.getJsonTemplate());
      settingsManager.saveSettings(settingsFilePath, jsonTemplate);
    }

    sdCardManager.createDirectory(directoryPath);
  }

  bmeMode = bme.begin();
  if (!bmeMode)
  {
    Serial.println("BME680 Sensor Not Found");
    // Status += "1";
  }
  else
  {
    Serial.println("BME680 Sensor Working");
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
    // Status += "0";
  }

  luxMode = veml.begin();
  if (!luxMode)
  {
    Serial.println("Lux Sensor Not Found");
    // Status += "1";
  }
  else
  {
    Serial.println("Lux Sensor Working");
    // Status += "0";
  }

  rtcMode = rtc.begin();
  if (!rtcMode)
  {
    Serial.println("RTC Sensor Not Found");
    // Status += "1";
  }
  else
  {
    Serial.println("RTC Sensor Working");
    // Status += "0";
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

    // if (!rtc.isrunning()) {
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // }
  }

  ltrMode = ltr.begin();
  if (!ltrMode)
  {
    Serial.println("Couldn't find LTR sensor!");
    // Status += "1";
  }
  else
  {
    Serial.println("LTR Sensor Working");
    // Status += "0";

    ltr.setMode(LTR390_MODE_UVS);
    if (ltr.getMode() == LTR390_MODE_ALS)
    {
      Serial.println("In ALS mode");
    }
    else
    {
      Serial.println("In UVS mode");
    }
    ltr.setGain(LTR390_GAIN_3);
    ltr.setResolution(LTR390_RESOLUTION_16BIT);
    ltr.setThresholds(100, 1000);
    ltr.configInterrupt(true, LTR390_MODE_UVS);
  }

  delay(5000);
}

void loop()
{
  dnsServer.processNextRequest();

  if (startDevice)
  {
    Serial.println("WiFi Mode!!");
    Serial.println("Current Settings:");
    settingsManager.readSettings(settingsFilePath, currentSettings);
    serializeJsonPretty(currentSettings, Serial);
    Serial.println();
    Serial.println(" ");

    telemetryData["s"] = 0;
    telemetryData["t"] = 0;
    telemetryData["p"] = 0;
    telemetryData["h"] = 0;
    telemetryData["p1"] = 0;
    telemetryData["p2"] = 0;
    telemetryData["p0"] = 0;
    telemetryData["l"] = 0;
    telemetryData["b"] = 0;
    telemetryData["d"] = 0;
    telemetryData["i"] = "";
    telemetryData["uv"] = 0;
    telemetryData["e"] = 0;

    if (bme.endReading() && bme.begin())
    {
      printf("BME680 Sensor Data Reading", true);

      Serial.print(F("Temperature = "));
      Serial.print(bme.temperature);
      float dataTemp = currentSettings["temp-offset"];
      telemetryData["t"] = dataTemp + bme.temperature;
      Serial.println();
      Serial.print(F("Temperature_Offset = "));
      Serial.print(dataTemp);
      Serial.println(" ");
      Serial.println(" ");

      Serial.print(F("Humidity = "));
      Serial.print(bme.humidity);
      float datahumi = currentSettings["humi-offset"];
      telemetryData["h"] = datahumi + bme.humidity;
      Serial.println();
      Serial.print(F("Humidity_Offset = "));
      Serial.print(datahumi);
      Serial.println(" ");
      Serial.println(" ");

      Serial.print(F("Pressure = "));
      Serial.print(bme.pressure / 100.00);
      float dataPress = currentSettings["press-offset"];
      telemetryData["p"] = dataPress + (bme.pressure / 100.0);
      Serial.println();
      Serial.print(F("Pressure_Offset = "));
      Serial.print(dataPress);
      Serial.println(" ");
      Serial.println(" ");
      Status += "0";
    }
    else
    {
      printf("BME680 Sensor Data Reading Failed", false);
      Status += "1";

      Serial.println(" ");
      Serial.println(" ");
    }

    if (veml.begin())
    {
      Serial.print(F("Lux = "));
      Serial.print(veml.readLux());
      float datalux = currentSettings["Lux-offset"];
      float luxy = veml.readLux();
      telemetryData["l"] = luxy + datalux;
      Serial.println();
      Serial.print(F("Lux_Offset = "));
      Serial.print(datalux);
      Serial.println(" ");
      Serial.println(" ");
      Status += "0";
    }
    else
    {
      Serial.println("Lux Sensor not working");
      Status += "1";
    }

    if (rtc.begin())
    {
      DateTime now = rtc.now();
      Serial.print(F("Timestamp = "));
      Serial.print(now.unixtime());
      telemetryData["d"] = now.unixtime();
      Serial.println();
      Serial.println(" ");
      Serial.println(" ");

      Serial.print(F("Battery Raw = "));
      Serial.print(4.2);   // analogRead(BAT));
      float battery = 4.2; // analogRead(BAT);
      telemetryData["b"] = battery;
      Serial.println(" ");
      Serial.println(" ");
      Status += "0";
    }
    else
    {
      Serial.println("RTC is not working");
      Status += "1";
    }

    if (ltr.newDataAvailable())
    {
      Serial.print(F("UV = "));
      Serial.print(ltr.readUVS());
      float datauv = currentSettings["UV-offset"];
      float uv = ltr.readUVS();
      telemetryData["uv"] = uv + datauv;
      Serial.println();
      Serial.print(F("UV_Offset = "));
      Serial.print(datauv);
      Serial.println(" ");
      Serial.println(" ");
      Status += "0";
    }

    else
    {
      Status += "1";
    }

    if (readPMSdata(&pmsSerial))
    {
      // reading data was successful!
      // Serial.println();
      // Serial.println("---------------------------------------");
      // Serial.println("Concentration Units (standard)");
      // Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
      // Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
      // Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
      // Serial.println("---------------------------------------");
      // Serial.println("Concentration Units (environmental)");
      // Serial.print("PM 1.0: "); Serial.print(data.pm10_env);
      // Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);
      // Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);
      // Serial.println("---------------------------------------");

      Serial.print(F("PM1.0 = "));
      Serial.print(data.pm10_env);
      float dataP0 = currentSettings["p0-offset"];
      telemetryData["p0"] = data.pm10_env;
      Serial.println();
      Serial.print(F("P0_Offset = "));
      Serial.print(dataP0);
      Serial.println(" ");
      Serial.println(" ");

      Serial.print(F("PM2.5 = "));
      Serial.print(data.pm25_env);
      float dataP2 = currentSettings["p2-offset"];
      telemetryData["p2"] = data.pm25_env;
      Serial.println();
      Serial.print(F("P0_Offset = "));
      Serial.print(dataP2);
      Serial.println(" ");
      Serial.println(" ");

      Serial.print(F("PM10 = "));
      Serial.print(data.pm100_env);
      float dataP1 = currentSettings["p1-offset"];
      telemetryData["p1"] = data.pm100_env;
      Serial.println();
      Serial.print(F("P1_Offset = "));
      Serial.print(dataP1);
      Serial.println(" ");
      Serial.println(" ");
    }
    else
    {
      ESP.restart();
    }

    MICP();
    // telemetryData["s"] =random(120.1,140.4);
    telemetryData["i"] = EID;
    telemetryData["e"] = Status;

    Serial.println(Status);
    serializeJsonPretty(telemetryData, Serial);
    String jsonString;
    serializeJson(telemetryData, jsonString);
    Serial.println(" ");
    Serial.println(" ");

    sdCardManager.appendFile(filePath, jsonString.c_str());

    HTTPClient http;

    http.begin(url);
    http.addHeader("Content-Type", "application/json");

    int httpCode = http.POST(jsonString);

    if (httpCode == HTTP_CODE_OK)
    {
      String response = http.getString();
      Serial.println(response);
      setRTCDateTime(response);
    }
    else
    {

      Serial.println("Failed 404");
    }

    telemetryData.clear();
    Status = "";

    delay(1000);

    http.end();

    delay(58000);
  }

  if (sDmode)
  {
    Serial.println("SD Card Mode!!");

    Serial.println("Current Settings:");
    settingsManager.readSettings(settingsFilePath, currentSettings);
    serializeJsonPretty(currentSettings, Serial);
    Serial.println();
    Serial.println(" ");

    telemetryData["s"] = 0;
    telemetryData["t"] = 0;
    telemetryData["p"] = 0;
    telemetryData["h"] = 0;
    telemetryData["p1"] = 0;
    telemetryData["p2"] = 0;
    telemetryData["p0"] = 0;
    telemetryData["l"] = 0;
    telemetryData["b"] = 0;
    telemetryData["d"] = 0;
    telemetryData["i"] = "";
    telemetryData["uv"] = 0;

    if (bme.endReading())
    {
      printf("BME680 Sensor Data Reading", true);

      Serial.print(F("Temperature = "));
      Serial.print(bme.temperature);
      float dataTemp = currentSettings["temp-offset"];
      telemetryData["t"] = dataTemp + bme.temperature;
      Serial.println();
      Serial.print(F("Temperature_Offset = "));
      Serial.print(dataTemp);
      Serial.println(" ");
      Serial.println(" ");

      Serial.print(F("Humidity = "));
      Serial.print(bme.humidity);
      float datahumi = currentSettings["humi-offset"];
      telemetryData["h"] = datahumi + bme.humidity;
      Serial.println();
      Serial.print(F("Humidity_Offset = "));
      Serial.print(datahumi);
      Serial.println(" ");
      Serial.println(" ");

      Serial.print(F("Pressure = "));
      Serial.print(bme.pressure / 100.00);
      float dataPress = currentSettings["press-offset"];
      telemetryData["p"] = dataPress + (bme.pressure / 100.0);
      Serial.println();
      Serial.print(F("Pressure_Offset = "));
      Serial.print(dataPress);
      Serial.println(" ");
      Serial.println(" ");
    }
    else
    {
      printf("BME680 Sensor Data Reading Failed", false);
      Serial.println(" ");
      Serial.println(" ");
    }

    Serial.print(F("Lux = "));
    Serial.print(veml.readLux());
    float datalux = currentSettings["Lux-offset"];
    float luxy = veml.readLux();
    telemetryData["l"] = luxy + datalux;
    Serial.println();
    Serial.print(F("Lux_Offset = "));
    Serial.print(datalux);
    Serial.println(" ");
    Serial.println(" ");

    if (ltr.newDataAvailable())
    {
      Serial.print(F("UV = "));
      Serial.print(ltr.readUVS());
      float datauv = currentSettings["UV-offset"];
      float uv = ltr.readUVS();
      telemetryData["u"] = uv + datauv;
      Serial.println();
      Serial.print(F("UV_Offset = "));
      Serial.print(datauv);
      Serial.println(" ");
      Serial.println(" ");
    }

    DateTime now = rtc.now();
    Serial.print(F("Timestamp = "));
    Serial.print(now.unixtime());
    telemetryData["d"] = now.unixtime();
    Serial.println();
    Serial.println(" ");
    Serial.println(" ");

    Serial.print(F("Battery Raw = "));
    Serial.print(4.2);
    float battery = 4.2;
    telemetryData["b"] = battery;
    Serial.println(" ");
    Serial.println(" ");

    if (readPMSdata(&pmsSerial))
    {
      // reading data was successful!
      // Serial.println();
      // Serial.println("---------------------------------------");
      // Serial.println("Concentration Units (standard)");
      // Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
      // Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
      // Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
      // Serial.println("---------------------------------------");
      // Serial.println("Concentration Units (environmental)");
      // Serial.print("PM 1.0: "); Serial.print(data.pm10_env);
      // Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);
      // Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);
      // Serial.println("---------------------------------------");

      Serial.print(F("PM1.0 = "));
      Serial.print(data.pm10_env);
      float dataP0 = currentSettings["p0-offset"];
      telemetryData["p0"] = data.pm10_env;
      Serial.println();
      Serial.print(F("P0_Offset = "));
      Serial.print(dataP0);
      Serial.println(" ");
      Serial.println(" ");

      Serial.print(F("PM2.5 = "));
      Serial.print(data.pm25_env);
      float dataP2 = currentSettings["p2-offset"];
      telemetryData["p2"] = data.pm25_env;
      Serial.println();
      Serial.print(F("P0_Offset = "));
      Serial.print(dataP2);
      Serial.println(" ");
      Serial.println(" ");

      Serial.print(F("PM10 = "));
      Serial.print(data.pm100_env);
      float dataP1 = currentSettings["p1-offset"];
      telemetryData["p1"] = data.pm100_env;
      Serial.println();
      Serial.print(F("P1_Offset = "));
      Serial.print(dataP1);
      Serial.println(" ");
      Serial.println(" ");
    }
    else
    {
      ESP.restart();
    }

    MICP();
    // telemetryData["s"] = 126.7;
    telemetryData["i"] = EID;

    serializeJsonPretty(telemetryData, Serial);
    String jsonString;
    serializeJson(telemetryData, jsonString);
    Serial.println(" ");
    Serial.println(" ");

    sdCardManager.appendFile(filePath, jsonString.c_str());
    telemetryData.clear();
    delay(60000);
  }
}
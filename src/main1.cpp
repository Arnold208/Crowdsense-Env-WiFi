#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "Adafruit_VEML7700.h"
#include <RtcDS3231.h>
#include "RTClib.h"

  
#define rxPin 14
#define txPin 4
#define RX 33
#define TX 35
#define MIC 27
#define BAT 15
#define PWR_CTRL 2
#define PM_CTRL 34

#define offset 0.183
#define BATTERY_FULL_VOLTAGE 4.2 
#define BATTERY_EMPTY_VOLTAGE 3.3

#define uS_TO_S_FACTOR 1000000ULL
#define TIME_TO_SLEEP  900
#define TIME_TO_SLEEP1  15
#define TIMEOUT 30000
 
float micSensitivity = -46;
long previousMillis = 0;
long interval = 10000;

int lastMicValue = 0;  
int pulseCount = 0;

RTC_DATA_ATTR int bootCount = 0;

Adafruit_BME680 bme;
Adafruit_VEML7700 veml = Adafruit_VEML7700();
RTC_DS1307 rtc;

int attempt = 0;
float pm1 = 0, pm2_5 = 0, pm10 = 0;
int8_t answer;
char aux_str[100];
int x = 0, attempts, numHTTPRequests = 2;
int time_check = 0;
char apn[] = "internet";
const char* server = "cctelemetry-dev.azurewebsites.net";
const char* path = "/telemetry";
String EID = "2e2";

unsigned long lastConnectionTime = 0;
const unsigned long postingInterval = 15000L;

SoftwareSerial pmsSerial(RX, TX);
SoftwareSerial serialSIM800(txPin, rxPin);
StaticJsonDocument<526> doc;

struct pms5003data {

  uint16_t framelen;

  uint16_t pm10_standard, pm25_standard, pm100_standard;

  uint16_t pm10_env, pm25_env, pm100_env;

  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;

  uint16_t unused;

  uint16_t checksum;

};

struct pms5003data data;


int count1 = 0;
int count2 = 0;

void timestmp(){
  
    DateTime now = rtc.now();
    Serial.print(now.unixtime());
    doc["d"] = now.unixtime();
  
}

float getPeakToPeakVoltage() {
  unsigned long sampleTime = 50; // Sample for 50 milliseconds
  int maxReading = 0; // Store maximum reading
  int minReading = 4095; // Store minimum reading (4095 for 12-bit ADC)
  unsigned long startTime = millis();

  // Collect data for the sample window duration
  while (millis() - startTime < sampleTime) {
    int reading = analogRead(MIC);

    if (reading > maxReading) {
      maxReading = reading;
    }
    if (reading < minReading) {
      minReading = reading;
    }
  }

  // Assuming a 12-bit ADC with a reference of 3.3V
  float maxVoltage = maxReading * (3.3 / 4095.0);
  float minVoltage = minReading * (3.3 / 4095.0);

  // Peak-to-peak voltage is the difference between max and min voltages
  return maxVoltage - minVoltage;
}

double round2(double value) {
  return (int)(value * 100 + 0.5) / 100.0;
}


void task1(void * parameters){
  unsigned long currentMillis = millis();
 
 for (;;) {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis > interval) {
      previousMillis = currentMillis;
      time_check++;
      Serial.println(time_check);
    }
    
  if (time_check == 35){
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP1 * uS_TO_S_FACTOR);
      Serial.println("I am coming to sleep!!");
      esp_deep_sleep_start();
      time_check = 0;
  }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void task2(void * parameters){
    for(;;){
    Serial.println(getPeakToPeakVoltage());
    float peakToPeakVoltage = getPeakToPeakVoltage(); // Get the peak-to-peak voltage

    // Check if there is no voltage change detected
    if (peakToPeakVoltage == 0) {
      Serial.println("Sound Level: Below detection threshold");
    } else {

      float rmsVoltage = peakToPeakVoltage / (2 * sqrt(2)); // Convert to RMS

      // Convert voltage to Pascals using the microphone sensitivity
      // micSensitivity is given in dB re 1V/Pa, so we first convert it to linear scale
      float sensitivityLinear = pow(10, micSensitivity / 20);
      float pressurePascals = rmsVoltage / sensitivityLinear;

      // Convert Pascals to dB
      float referencePressure = 20e-6; // Reference pressure in Pascals
      float soundLevelDB = 20 * log10(pressurePascals / referencePressure);

      // Output the dB level
      Serial.print("Sound Level: ");
      Serial.print(soundLevelDB);
      Serial.println(" dB");

      vTaskDelay(3000 / portTICK_PERIOD_MS);
        doc["s"] = round2(soundLevelDB);

    }
  }
}


void bme680() {

  unsigned long endTime = bme.beginReading();

  if (endTime == 0) {

    Serial.println(F("Failed to begin reading :("));

  }

  Serial.print(F("Reading started at "));

  Serial.print(millis());

  Serial.print(F(" and will finish at "));

  Serial.println(endTime);

  delay(50);

  if (!bme.endReading()) {

    Serial.println(F("Failed to complete reading :("));

 
  }

  Serial.print(F("Reading completed at "));

  Serial.println(millis());

  Serial.print(F("Temperature = "));
  Serial.print(bme.temperature);
  Serial.println(F(" *C"));

  Serial.print(F("Pressure = "));
  Serial.print(bme.pressure / 100.0);
  Serial.println(F(" hPa"));

  Serial.print(F("Humidity = "));
  Serial.print(bme.humidity);
  Serial.println(F(" %"));

  doc["t"] =  round2(bme.temperature);
  doc["p"] = round2((bme.pressure / 1000.0));
  doc["h"] = round2(bme.humidity);
}


void lux() {
  float luxy = veml.readLux(VEML_LUX_AUTO);
  luxy = (luxy * 100) / 30199;
  luxy = constrain(luxy, 0, 100);
  Serial.print("Lux = "); Serial.println(luxy);
  doc["l"] = round2(luxy);

}

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
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
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}


void PM() {
  if (readPMSdata(&pmsSerial)) {

    Serial.println("Concentration Units (environmental)");

    Serial.print("PM 1.0: "); Serial.print(data.pm10_env);

    Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);

    Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);

    pm1 = data.pm10_env;

    pm2_5 = data.pm25_env;

    pm10 = data.pm100_env;

  }
  if (pm1 > 100 || pm2_5 > 100  || pm10 > 100 ) {

    doc["p1"] = 0;

    doc["p2"] = 0;

    doc["p0"] = 0;

  }

  else {

    doc["p1"] = pm1;

    doc["p2"] = pm2_5;

    doc["p0"] = pm10;
  }

  delay(1000);

}


void microphone() {

  float mic = analogRead(MIC);

  doc["s"] = round2(mic / 100);

  Serial.println(mic);

}

 

void battery() {

  float battery_lvl = analogRead(BAT);

  float voltage = (((float)battery_lvl / 4095.0 * 3.3) * 2) + 0.183;

  float percentage = (voltage - BATTERY_EMPTY_VOLTAGE) / (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE) * 100;
  
  doc["b"] = round2(percentage);

  delay(500);
}

int8_t sendATcommand(const char* ATcommand, const char* expected_answer1, unsigned int timeout) {
  uint8_t x = 0,  answer = 0;
  char response[100];
  unsigned long previous;

  memset(response, '\0', 100);    // Initialize the string

  delay(100);

  while (serialSIM800.available())
  { //Cleans the input buffer
    serialSIM800.read();
  }



  Serial.println(ATcommand);    // Prints the AT command
  serialSIM800.write(ATcommand); // Sends the AT command


  x = 0;
  previous = millis();

  // this loop waits for the answer
  do
  {

    if (serialSIM800.available() != 0)
    {
      response[x] = serialSIM800.read();
      x++;
      if (strstr(response, expected_answer1) != NULL)
      {
        answer = 1;
      }
    }
    // Waits for the asnwer with time out
  }
  while ((answer == 0) && ((millis() - previous) < timeout));

  return answer;
}

void power_on()
{

  uint8_t answer = 0;

  Serial.println("On Power_on...");

  // checks if the module is started
  answer = sendATcommand("AT\r\n", "OK\r\n", TIMEOUT);
  if (answer == 0)
  {
    // power on pulse
    delay(3000);

    // waits for an answer from the module
    while (answer == 0)
    {
      // Send AT every two seconds and wait for the answer
      answer = sendATcommand("AT\r\n", "OK\r\n", TIMEOUT);
      Serial.println("Trying connection with module...");
    }
  }
}

void restartPhoneActivity()
{
  do
  {
    sendATcommand("AT+CFUN=0\r\n", "OK\r\n", TIMEOUT);
    delay(2000);
    answer = sendATcommand("AT+CFUN=1\r\n", "Call Ready\r\n", TIMEOUT);
  } while (answer == 0);
}

void connectToNetwork()
{
  sendATcommand("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n", "OK\r\n", TIMEOUT);//sets Contype
  snprintf(aux_str, sizeof(aux_str), "AT+SAPBR=3,1,\"APN\",\"%s\"\r\n", apn);//sets APN
  sendATcommand(aux_str, "OK\r\n", TIMEOUT);
  attempts = 0;//tries 3 times or gets on the loop until sendATcommand != 0
  while (sendATcommand("AT+SAPBR=1,1\r\n", "OK\r\n", TIMEOUT) == 0)
  {
    delay(5000);
    attempts = attempts + 1;
    if (attempts > 2)
    {
      restartPhoneActivity();
      attempts = 0;
    }
  }
}


void initHTTPSession()
{
  while (sendATcommand("AT+HTTPINIT\r\n", "OK\r\n", TIMEOUT) == 0)
  {
    restartPhoneActivity();
    connectToNetwork();
  }
}

void HTTPRequest()
{

  bme680();

  delay(100);

  lux();

  //microphone();

  delay(5000);

  PM();

  delay(100);

  battery();

  doc["i"] = EID;

  delay(2000);

  
  timestmp();

  String jsonPayload;

  serializeJson(doc, jsonPayload);

  const char* jsonPayloadChar = jsonPayload.c_str();


  String atCommand = "AT+HTTPPARA=\"URL\",\"http://" + String(server) + path + "\"\r\n";
  sendATcommand(atCommand.c_str(), "OK\r\n", TIMEOUT);

  atCommand = "AT+HTTPPARA=\"CID\",1\r\n";
  sendATcommand(atCommand.c_str(), "OK\r\n", TIMEOUT);

  atCommand = "AT+HTTPPARA=\"CONTENT\",\"application/json\"\r\n";
  sendATcommand(atCommand.c_str(), "OK\r\n", TIMEOUT);

  atCommand = "AT+HTTPDATA=" + String(jsonPayload.length()) + ",10000\r\n";
  sendATcommand(atCommand.c_str(), "DOWNLOAD", TIMEOUT);

  atCommand = jsonPayload;
  sendATcommand(atCommand.c_str(), "OK\r\n", TIMEOUT);

  atCommand = "AT+HTTPACTION=1\r\n";
  sendATcommand(atCommand.c_str(), "+HTTPACTION: 1,200", TIMEOUT);

  Serial.println("Successfully uploaded");
  delay(1000);
  jsonPayloadChar = "";
  lastConnectionTime = millis();

}

void disconnectFromNetwork() {

  sendATcommand("AT+SAPBR=0,1\r\n", "OK\r\n", TIMEOUT);

  sendATcommand("AT+HTTPTERM\r\n", "OK\r\n", TIMEOUT);
}


void printDateTime(const RtcDateTime& dt) {
  char datestring[26];
  snprintf_P(datestring,
              sizeof(datestring),
              PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
              dt.Month(),
              dt.Day(),
              dt.Year(),
              dt.Hour(),
              dt.Minute(),
              dt.Second());
//  Serial.println(datestring);
}


void run_program(){
Serial.println("I am on!!");

    digitalWrite(PWR_CTRL, HIGH);

    delay(1000);

    pmsSerial.begin(9600);

    serialSIM800.begin(9600);

    delay(1000);

    bool rtcInitSuccess = rtc.begin();
  Serial.printf("RTC Initialization", rtcInitSuccess);
  if (rtcInitSuccess && !rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  

    if (!veml.begin()) {

      Serial.println("Sensor not found");

    }

    if (!bme.begin()) {

      Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));

    }

    bme.setTemperatureOversampling(BME680_OS_8X);

    bme.setHumidityOversampling(BME680_OS_2X);

    bme.setPressureOversampling(BME680_OS_4X);

    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);

    bme.setGasHeater(320, 150);

    delay(2000);


    xTaskCreate(
      task1,
      "Task 1",
      1000,
      NULL,
      1,
      NULL
    );

    xTaskCreate(
      task2,
      "Task 2",
      1000,
      NULL,
      1,
      NULL
    );
    
    power_on();
    
    delay(1000);

    connectToNetwork();

    initHTTPSession();

    // xTaskCreate(
    //   task1,
    //   "Task 1",
    //   1000,
    //   NULL,
    //   1,
    //   NULL
    // );

    for (int i = 0; i < 3; ++i) {

      Serial.println("Running HTTP request #" + String(i + 1));

      HTTPRequest();

      delay(3000);
    }

    disconnectFromNetwork();

    delay(2000);

    digitalWrite(PWR_CTRL, LOW);

    Serial.println("Going to sleep...");

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

    Serial.flush();

    esp_deep_sleep_start();

    Serial.println("This will never be printed");

}

void setup() {
  Serial.begin(115200);
  pinMode(MIC, INPUT);
  pinMode(BAT, INPUT);
  pinMode(PM_CTRL, INPUT);
  pinMode(PWR_CTRL, OUTPUT);
  analogReadResolution(12); 
  analogSetAttenuation(ADC_0db);
  delay(1000);
  ++bootCount;
  Serial.println(bootCount);


  if(bootCount > 0 && bootCount == 1){
    Serial.println("First Booting Sequence");
     run_program();
  }

  if (bootCount % 4 != 0) {
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.println("Going to sleep now");
    Serial.flush();
    esp_deep_sleep_start();
    Serial.println("This will never be printed");
  }

 else {
     run_program();
  }
}

void loop() {
 //Nothing to do here
}

 
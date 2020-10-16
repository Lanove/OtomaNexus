/*
Todo :
- Add softSSID,softPW,softIPAddress at setting menu of otoma web
- Check last update of ESP8266 on server to notify the user when the ESP8266 is not updating for more than 5 minute (because disconnect)
- Add notifier on otoma web when user update automation program but automation program is not fetched by ESP8266 for some reason (disconnected or such)
- Add buzzer notification for WiFi fail connect, disconnect, time out.
- Add LED status for WiFi connecting, and setup status
- Create Shift Register API -- done part
- Create Analog Input API -- done part
- DONE Remove != and == from comparator list
- DONE Lol, the name is operator, not comparator. to be exact, it was relational operators
- Lk sempet add OTA Web Server HTTP Update
- Fix parse input on web automation program
- Optimize stack usage by moving memory usage to heap with malloc/free/calloc
- DONE Do reload status just for read-only thing (like temp, humid)


Idea : 
- do something like this on automation program
bool condition;
If(program == "suhu")
  if(operator == ">")
    cond = (suhu>x)
  If(operator == "<")
    cond = (suhu<x)
  If(operator == "==")
    cond = (suhu==x)
endif
If(program == "jadwal")
  cond = (time > jadwalstart && time < jadwalstop)

- Proposed logic of automation program
Lk program e Tanggal Waktu or Jadwal Harian, EEPROM e yo nyimpen flag.
Jika waktu saiki memenuhi kondisi terus keadaan output ga sesuai karo aksi, flag e dadi true karo eksekusi aksi.
terus lha lk wes kondisi tidak terpenuhi (waktune ga pas or smth else), flag e dicek.
lha lk flag e jk 1, maka eksekusi anti-aksi terus flag e dirubah dadi 0
tsumari, dia punya anti-aksi lk wes ga memenuhi kondisi, tapi hanya one-shot saja

Ngubah keadaan output pas akhir program, kinda like ladder logic

Keterangan Pin :
- GPIO16 - ENC CLK/A
- GPIO14 - ENC DT/B
- GPIO12 - ENC BTN
- GPIO13 - SENSE
- GPIO0 - 595 CLK
- GPIO2 - 595 LATCH
- GPIO15 - 595 DATA

*/
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <time.h>
#include <constants.h>

#include <ArduinoJson.h>
#include <Wire.h>
#include <RTClib.h>
#include <Eeprom24C04_16.h>
#include "DHT.h"
#include <ShiftRegister74HC595.h>
#include <OneWire.h>
#include <DallasTemperature.h>

ESP8266WebServer server(80); // Create a webserver object that listens for HTTP request on port 80
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "asia.pool.ntp.org", 25200); // 25200 for UTC+7, 3600*(UTC)
static Eeprom24C04_16 eeprom(EEPROM_ADDRESS);
RTC_DS1307 rtc;
DHT dht(DHTPIN, DHTTYPE);
ShiftRegister74HC595<1> sr(DATA595, CLOCK595, LATCH595);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b(&oneWire);

String storedUsername;
String storedSSID;
String storedWifiPass;
String storedSoftSSID;
String storedSoftWifiPass;
String storedDeviceToken;
byte storedFirstByte;
bool serverAvailable;
bool triedLog = false;
unsigned long disconnectStamp;
unsigned long requestMillis;
unsigned long sensorMillis;
byte dhtSampleCounter;
bool disconnectFlag;
byte byte595Status;
bool bit595Status;
byte deviceStatus;

float newTemp;
float newHumid;

int thermalSetPoint;
float heaterKp;
float heaterKi;
float heaterKd;
float heaterDs;

float heaterBa;
float heaterBb;

float coolerKp;
float coolerKi;
float coolerKd;
float coolerDs;

float coolerBa;
float coolerBb;

byte progTrig[30];
byte progRB1[30][4];
byte progRB2[30][4];
byte progAct[30];

void debugMemory(); // Function to check free stack and heap remaining
void storeString(int addrOffset, const String &strToWrite);
byte *readString(int addrOffset);
const String readFromEEPROM(byte what);
void writeToEEPROM(byte what, const String &strToWrite);
byte byteReadFB();
void byteWriteFB(byte data);
bool bitReadFB(byte docchi);
void bitWriteFB(byte docchi, bool status);
void loadAllPrograms();

void loadInfo();
bool initiateSoftAP();
bool initiateClient(const String &ssid, const String &pass);
void deployWebServer();
void closeServer();
bool closeSoftAP();
bool closeClient();
bool isWifiConnected();
const String fetchURL(const String &URL, const String &payload, int &responseCode);
void writeByteEEPROM(int addr, byte data);
void pgRoot(); // function prototypes for HTTP handlers
void pgInit();
void pgRestart();
void pgReqStatus();
void pgAccInfo();
void handleNotFound();
void toggle595Bit(byte docchi);
void bitWrite595(byte docchi, bool status);
byte byteRead595();
bool bitRead595(byte docchi);
void byteWrite595(const uint8_t data);
void selectMux(byte channel);
int readMux();

void debugMemory()
{
  Serial.printf("Stack : %d\nHeap : %d\n", ESP.getFreeContStack(), ESP.getFreeHeap());
}

void setup(void)
{
  Serial.println("Init");
  debugMemory();
  storedUsername.reserve(32);
  storedSSID.reserve(32);
  storedWifiPass.reserve(64);
  storedSoftSSID.reserve(32);
  storedSoftWifiPass.reserve(64);
  storedDeviceToken.reserve(10);
  Serial.begin(74880); // Start the Serial communication to send messages to the computer

  delay(10);
  // pinMode(LED_BUILTIN, OUTPUT);
  delay(100);

  Serial.println("PreLib");
  debugMemory();
  if (!rtc.begin())
  {
    Serial.println("Couldn't Start RTC!");
    // failed to init rtc
    // add some loud ass error buzzer here
    // rtc is important element, so stop right here if something is wrong
    while (true)
    {
    }
  }
  eeprom.initialize();
  ds18b.begin();
  dht.begin();
  Serial.println("AftLib");
  debugMemory();

  Serial.println("PreLdInfo");
  debugMemory();
  loadInfo();
  Serial.println("AftLdInfo");
  debugMemory();
  loadAllPrograms();
  Serial.println("AftLdPr");
  debugMemory();

  Serial.println();
  Serial.print("storedFirstByte : ");
  Serial.println(storedFirstByte);
  Serial.print("storedSoftSSID : ");
  Serial.println(storedSoftSSID);
  Serial.print("storedSoftWifiPass : ");
  Serial.println(storedSoftWifiPass);
  Serial.print("storedSSID : ");
  Serial.println(storedSSID);
  Serial.print("storedWifiPass : ");
  Serial.println(storedWifiPass);
  Serial.print("storedUsername : ");
  Serial.println(storedUsername);
  debugMemory();

  byteWrite595(0x00);
  closeClient();
  closeSoftAP();
  closeServer();
  if (!bitReadFB(FB_CONNECTED))
  {
    initiateSoftAP();
    deployWebServer(); // Deploy web server
  }
  else
  {
    int timeOutCounter = 0;
    while (timeOutCounter < 5) // Try for maximum of 5 attempts
    {
      if (initiateClient(storedSSID, storedWifiPass))
      {
        bitWriteFB(FB_WIFI_ERROR1, false);
        bitWriteFB(FB_WIFI_ERROR2, false);
        break;
      }
      timeOutCounter++;
    }
    if (timeOutCounter >= 5) // If timed out for 5 attempts after 2 reboot, rollback to softAP mode and reset FB
    {
      Serial.println("Fail reconnect WiFi");
      if (bitReadFB(!FB_WIFI_ERROR1))
      {
        Serial.println("Rebooting attempt 1");
        bitWriteFB(FB_WIFI_ERROR1, true);
        delay(500);
        ESP.restart();
      }
      else
      {
        if (bitReadFB(!FB_WIFI_ERROR2))
        {
          Serial.println("Rebooting attempt 2");
          bitWriteFB(FB_WIFI_ERROR2, true);
          delay(500);
          ESP.restart();
        }
        else
        {
          initiateSoftAP(); // Seems that SSID is invalid, initiate Soft AP instead
          deployWebServer();
          bitWriteFB(FB_CONNECTED, false);
          bitWriteFB(FB_WIFI_ERROR1, false);
          bitWriteFB(FB_WIFI_ERROR2, false);
        }
      }
    }
  }
  if (isWifiConnected())
  {
    timeClient.begin(); // If WiFi connection success, start NTP timeClient
    if (timeClient.forceUpdate())
    {
      rtc.adjust(DateTime(timeClient.getEpochTime()));
    }
    else
    {
      Serial.println("Failed fetching NTP Clock");
      if (!rtc.isrunning())
      {
        Serial.println("Could'nt Initiate RTC!");
        // We're fucked up my friend, RTC is not running and can't fetch NTP time, so we'll stop right there too
        // failed to init rtc
        // add some loud ass error buzzer here
        // rtc is important element, so stop right here if something is wrong
        while (true)
        {
          // do something here
        }
      }
    }

    debugMemory();
  }

  ds18b.requestTemperatures(); // Send the command to get temperatures
  newTemp = ds18b.getTempCByIndex(0);
  newHumid = dht.readHumidity();
  debugMemory();
}

void loop(void)
{
  if (millis() - sensorMillis >= SENSOR_DELAY)
  {
    float *sensorBuffer = (float *)malloc(sizeof(float) * 2);
    dhtSampleCounter--;
    ds18b.requestTemperatures(); // Send the command to get temperatures
    sensorBuffer[0] = ds18b.getTempCByIndex(0);
    if (sensorBuffer[0] != DEVICE_DISCONNECTED_C)
      newTemp = sensorBuffer[0];
    if (dhtSampleCounter <= 0 || dhtSampleCounter > DHT_LOOP)
    {
      sensorBuffer[1] = dht.readHumidity();
      if (!isnan(sensorBuffer[1]))
        newHumid = sensorBuffer[1];
      dhtSampleCounter = DHT_LOOP;
    }
    sensorMillis = millis();
    free(sensorBuffer);
  }

  if (serverAvailable)
  {
    server.handleClient(); // Listen for HTTP requests from clients
  }
  else
  {
    if (isWifiConnected())
    {
      if (millis() - requestMillis >= REQUEST_DELAY)
      {
        DateTime now = rtc.now();
        if (timeClient.update())
        {
          // If unixtime of RTC is deviating + or - 30 seconds than NTP time then readjust RTC
          if (timeClient.getEpochTime() > now.unixtime() + 10 || timeClient.getEpochTime() < now.unixtime() - 10)
            rtc.adjust(DateTime(timeClient.getEpochTime()));
        }
        const size_t capacity = JSON_ARRAY_SIZE(7) + JSON_OBJECT_SIZE(4);
        DynamicJsonDocument doc(capacity);
        String json;
        // Type is splitted by 'n'
        // t means temperature
        // h means humidity
        // s means status of aux1,aux2,th,ht,cl (ordered)
        // the highest order is t and goes lower to s
        // so tnhns means ESP8266 is sending temperature and humidity
        doc["type"] = "tnhns";
        doc["unix"] = now.unixtime();
        JsonArray data = doc.createNestedArray("data");

        data.add(newTemp);
        data.add(newHumid);
        data.add(1);
        data.add(0);
        data.add(1);
        data.add(1);
        data.add(0);
        serializeJson(doc, json);

        int httpCode;
        String response = fetchURL(FPSTR(requestURL), json, httpCode);
        Serial.printf("Response : %s\n", response.c_str());
        if (httpCode > 0)
        {
          if (httpCode == HTTP_CODE_OK || httpCode >= 500 || (httpCode > 200 && httpCode < 300)) // Successful Fetch or Server Mistake reset disconnectFlag
          {
            disconnectFlag = false;
            disconnectStamp = millis();
          }
          else if (httpCode >= 400 && httpCode < 500)
          {
            disconnectFlag = true;
            disconnectStamp = millis();
          }
          if (httpCode == HTTP_CODE_OK)
          {
            if (response == "forget")
            {
              bitWriteFB(FB_CONNECTED, false);
              delay(500);
              ESP.reset();
            }
          }
        }
        else
        {
          disconnectFlag = true;
          disconnectStamp = millis();
        }

        debugMemory();
        requestMillis = millis();
      }
    }
    else
    {
      disconnectFlag = true;
      disconnectStamp = millis();
    }
  }
  if (disconnectFlag && millis() - disconnectStamp >= MAXIMUM_DISCONNECT_TIME)
  {
    writeToEEPROM(WIFISSID, "GAGAL KONEKSI");
    writeToEEPROM(WIFIPW, " ");
    bitWriteFB(FB_CONNECTED, false);
    delay(500);
    ESP.reset();
  }
}

////////////////////////////////////// IO API ///////////////////////////////////////////////////////
void toggle595Bit(byte docchi)
{
  byte595Status ^= 1 << docchi;
  sr.set(docchi, bitRead595(docchi));
}

void bitWrite595(byte docchi, bool status)
{
  sr.set(docchi, status);
  if (status)
    byte595Status |= (1 << docchi);
  else
    byte595Status &= ~(1 << docchi);
}

byte byteRead595()
{
  return byte595Status;
}

bool bitRead595(byte docchi)
{
  return ((byte595Status >> docchi) & 0x01);
}

void byteWrite595(const uint8_t data)
{
  const uint8_t dataCopy[1] = {data};
  sr.setAll(dataCopy);
  memcpy((uint8_t *)&byte595Status, &data, 1 * sizeof(uint8_t));
}

void selectMux(byte channel)
{
  if (channel == 1)
  { // A1 is X0
    bitWrite595(MUX_A, LOW);
    bitWrite595(MUX_B, LOW);
  }
  else if (channel == 2)
  { // A2 is X1
    bitWrite595(MUX_A, HIGH);
    bitWrite595(MUX_B, LOW);
  }
  else if (channel == 3)
  { // A3 is X2
    bitWrite595(MUX_A, LOW);
    bitWrite595(MUX_B, HIGH);
  }
  else if (channel == 0)
  { // A0 is X3
    bitWrite595(MUX_A, HIGH);
    bitWrite595(MUX_B, HIGH);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////// EEPROM API /////////////////////////////////////////////////////////
void storeString(int addrOffset, const String &strToWrite)
{
  byte len = strToWrite.length();
  byte *data = (byte *)malloc(sizeof(byte) * (len + 1));
  strToWrite.getBytes(data, len + 1);
  eeprom.writeByte(addrOffset, len);
  delay(10);
  eeprom.writeBytes(addrOffset + 1, len, data);
  free(data);
}

byte *readString(int addrOffset)
{
  byte *buffer;
  int newStrLen = eeprom.readByte(addrOffset);
  buffer = (byte *)malloc(sizeof(byte) * newStrLen);
  eeprom.readBytes(addrOffset + 1, newStrLen, buffer);
  buffer[newStrLen] = '\0';
  return buffer;
}

byte byteReadFB()
{
  storedFirstByte = eeprom.readByte(0);
  return storedFirstByte;
}

void byteWriteFB(byte data)
{
  eeprom.writeByte(0, data);
  storedFirstByte = data;
  delay(10);
}

bool bitReadFB(byte docchi)
{
  return ((storedFirstByte >> docchi) & 0x01);
}

void bitWriteFB(byte docchi, bool status)
{
  if (status)
    storedFirstByte |= (1 << docchi);
  else
    storedFirstByte &= ~(1 << docchi);
  byteWriteFB(storedFirstByte);
}

const String readFromEEPROM(byte what)
{
  byte *data;
  if (what == WIFISSID)
  {
    data = readString(1);
    storedSSID = String((char *)data);
    free(data);
    return storedSSID;
  }
  else if (what == WIFIPW)
  {
    data = readString(34);
    storedWifiPass = String((char *)data);
    free(data);
    return storedWifiPass;
  }
  else if (what == SOFTSSID)
  {
    data = readString(99);
    storedSoftSSID = String((char *)data);
    free(data);
    return storedSoftSSID;
  }
  else if (what == SOFTPW)
  {
    data = readString(132);
    storedSoftWifiPass = String((char *)data);
    free(data);
    return storedSoftWifiPass;
  }
  else if (what == USERNAME)
  {
    data = readString(197);
    storedUsername = String((char *)data);
    free(data);
    return storedUsername;
  }
  else if (what == DEVICETOKEN)
  {
    data = readString(230);
    storedDeviceToken = String((char *)data);
    free(data);
    return storedDeviceToken;
  }
  free(data);
  return "INVALID";
}

void writeToEEPROM(byte what, const String &strToWrite)
{
  unsigned int address = 1;

  if (what == WIFISSID)
  {
    if (strToWrite.length() < 33)
      storedSSID = strToWrite;
    address = 1;
  }
  else if (what == WIFIPW)
  {
    if (strToWrite.length() < 65)
      storedWifiPass = strToWrite;
    address = 34;
  }
  else if (what == SOFTSSID)
  {
    if (strToWrite.length() < 33)
      storedSoftSSID = strToWrite;
    address = 99;
  }
  else if (what == SOFTPW)
  {
    if (strToWrite.length() < 65)
      storedSoftWifiPass = strToWrite;
    address = 132;
  }
  else if (what == USERNAME)
  {
    if (strToWrite.length() < 33)
      storedUsername = strToWrite;
    address = 197;
  }
  else if (what == DEVICETOKEN)
  {
    if (strToWrite.length() < 11)
      storedDeviceToken = strToWrite;
    address = 230;
  }

  storeString(address, strToWrite);
}

void loadInfo()
{
  byteReadFB();
  readFromEEPROM(WIFISSID);
  readFromEEPROM(WIFIPW);
  readFromEEPROM(SOFTSSID);
  readFromEEPROM(SOFTPW);
  readFromEEPROM(USERNAME);
  readFromEEPROM(DEVICETOKEN);
  if (storedSSID == " ")
    storedSSID = "";
  if (storedWifiPass == " ")
    storedWifiPass = "";
  if (storedSoftSSID == " ")
    storedSoftSSID = "";
  if (storedSoftWifiPass == " ")
    storedSoftWifiPass = "";
  if (storedUsername == " ")
    storedUsername = "";
  if (storedDeviceToken == " ")
    storedDeviceToken = "";
}

void loadAllPrograms()
{
  byte *data = (byte *)malloc(sizeof(byte) * (361)); // From 241 to 601 is
  eeprom.readBytes(241, 361, data);

  deviceStatus = data[ADDR_DEVICE_STATUS - ADDR_DEVICE_STATUS];
  memcpy(&thermalSetPoint, &data[ADDR_THERMAL_SETPOINT - ADDR_DEVICE_STATUS], 2);
  memcpy(&heaterKp, &data[ADDR_HEATER_KP - ADDR_DEVICE_STATUS], sizeof(float));
  memcpy(&heaterKi, &data[ADDR_HEATER_KI - ADDR_DEVICE_STATUS], sizeof(float));
  memcpy(&heaterKd, &data[ADDR_HEATER_KD - ADDR_DEVICE_STATUS], sizeof(float));
  memcpy(&heaterDs, &data[ADDR_HEATER_DS - ADDR_DEVICE_STATUS], sizeof(float));
  memcpy(&heaterBa, &data[ADDR_HEATER_BA - ADDR_DEVICE_STATUS], sizeof(float));
  memcpy(&heaterBb, &data[ADDR_HEATER_BB - ADDR_DEVICE_STATUS], sizeof(float));

  memcpy(&coolerKp, &data[ADDR_COOLER_KP - ADDR_DEVICE_STATUS], sizeof(float));
  memcpy(&coolerKi, &data[ADDR_COOLER_KI - ADDR_DEVICE_STATUS], sizeof(float));
  memcpy(&coolerKd, &data[ADDR_COOLER_KD - ADDR_DEVICE_STATUS], sizeof(float));
  memcpy(&coolerDs, &data[ADDR_COOLER_DS - ADDR_DEVICE_STATUS], sizeof(float));
  memcpy(&coolerBa, &data[ADDR_COOLER_BA - ADDR_DEVICE_STATUS], sizeof(float));
  memcpy(&coolerBb, &data[ADDR_COOLER_BB - ADDR_DEVICE_STATUS], sizeof(float));
  for (byte i = 0; i < 30; i++)
  {
    progTrig[i] = data[ADDR_PROG_TRIGGER(i) - ADDR_DEVICE_STATUS];
    memcpy(&progRB1[i], &data[ADDR_PROG_RB1(i) - ADDR_DEVICE_STATUS], 4);
    memcpy(&progRB2[i], &data[ADDR_PROG_RB2(i) - ADDR_DEVICE_STATUS], 4);
    progAct[i] = data[ADDR_PROG_ACTION(i) - ADDR_DEVICE_STATUS];
  }
  free(data);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////// SOFT AP, CLIENT AND WEB SERVER API //////////////////////////////////
const String fetchURL(const String &URL, const String &data, int &responseCode)
{
  WiFiClient client;
  HTTPClient http;
  // configure traged server and url
  http.begin(client, URL); //HTTP
  http.addHeader(F("Content-Type"), F("application/json"));
  http.addHeader(F("Device-Token"), storedDeviceToken);

  // start connection and send HTTP header and body
  int httpCode = http.POST(data);
  responseCode = httpCode;

  // httpCode will be negative on error
  if (httpCode > 0)
  {
    // HTTP header has been send and Server response header has been handled
    // file found at server
    if (httpCode == HTTP_CODE_OK)
    {
      return http.getString();
    }
  }
  else
  {
    Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
  return "";
}

bool isWifiConnected()
{
  return (WiFi.status() == WL_CONNECTED);
}

bool initiateSoftAP()
{
  bool timeOutFlag = 1;
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 2);
  IPAddress subnet(255, 255, 255, 0);
  delay(100);
  for (int i = 0; i < 20; i++)
  {
    if (WiFi.softAPConfig(local_IP, gateway, subnet))
    {
      Serial.println("softAP configured");
      timeOutFlag = 0;
      break;
    }
    else
    {
      Serial.print(".");
      delay(500);
    }
  }
  Serial.println();
  delay(100);
  for (int i = 0; i < 20; i++)
  {
    if (WiFi.softAP(storedSoftSSID, storedSoftWifiPass))
    {
      Serial.println("soft AP started");
      timeOutFlag = 0;
      break;
    }
    else
    {
      Serial.print(".");
      delay(500);
    }
  }
  delay(100);
  return timeOutFlag;
}

bool initiateClient(const String &ssid, const String &pass)
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  bool successFlag = false;
  for (int i = 0; i < 30; i++)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("Connected!");
      Serial.println(WiFi.localIP().toString());
      successFlag = true;
      break;
    }
    else
    {
      Serial.print(".");
      bitWrite595(LED_STATUS, HIGH);
      delay(250);
      bitWrite595(LED_STATUS, LOW);
      delay(250);
    }
    if (i >= 29)
    {
      successFlag = 0;
    }
  }
  return successFlag;
}

bool closeSoftAP()
{
  return WiFi.softAPdisconnect(true);
}

bool closeClient()
{
  return WiFi.disconnect(true);
}

void deployWebServer()
{
  // Call the 'handleRoot' function when a client requests URI "/"
  server.on("/", HTTP_GET, pgRoot);

  server.on("/reqStatus", HTTP_POST, pgReqStatus);
  server.on("/accInfo", HTTP_POST, pgAccInfo);
  server.on("/restart", HTTP_POST, pgRestart);

  // When a client requests an unknown URI (i.e. something other than "/"), call function "handleNotFound"
  server.onNotFound(handleNotFound);
  server.begin();
  serverAvailable = true;
}

void closeServer()
{
  server.close();
  serverAvailable = false;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////// WEB SERVER REQUEST HANDLER ////////////////////////////////////////
void pgRoot()
{
  server.send(200, "text/html", FPSTR(htmlDoc));
}

void pgRestart()
{
  server.send(200, "text/plain", "restart");
  delay(500);
  ESP.reset();
}

String responseStatus = "empty";
void pgAccInfo()
{
  String usrn = server.arg(F("usrn"));
  String unpw = server.arg(F("unpw"));
  String ssid = server.arg(F("ssid"));
  String wfpw = server.arg(F("wfpw"));

  if (usrn.length() > 32)
    usrn = "overlength";
  if (unpw.length() > 64)
    unpw = "overlength";
  if (ssid.length() > 32)
    ssid = "overlength";
  if (wfpw.length() > 64)
    wfpw = "overlength";

  Serial.printf("Username : %s\nPassword : %s\nSSID : %s\nWiFiPW : %s\n", usrn.c_str(), unpw.c_str(), ssid.c_str(), wfpw.c_str());
  if (initiateClient(ssid, wfpw))
  {
    // This WiFi seems legit, let's save to EEPROM
    writeToEEPROM(WIFISSID, ssid);
    writeToEEPROM(WIFIPW, wfpw);

    // Initiate HTTP Request to identifyDevice.php
    Serial.print("[HTTP] begin...\n");
    int httpCode;
    StaticJsonDocument<350> doc;
    String json;
    doc[F("username")] = usrn.c_str();
    doc[F("password")] = unpw.c_str();
    doc[F("devicetoken")] = storedDeviceToken.c_str();
    doc[F("softssid")] = storedSoftSSID.c_str();
    doc[F("softpw")] = storedSoftWifiPass.c_str();
    serializeJson(doc, json);
    Serial.printf("JSON Size : %d\n", doc.memoryUsage());
    Serial.printf("Transferred JSON : %s\n", json.c_str());
    responseStatus = fetchURL(FPSTR(identifyURL), json, httpCode);
    Serial.printf("Code : %d\nResponse : %s\n", httpCode, responseStatus.c_str());
    if (httpCode == HTTP_CODE_OK)
    { // Success fetched!, store the message to responseStatus!
      if (responseStatus == "success" || responseStatus == "recon")
      {
        writeToEEPROM(USERNAME, usrn);
        bitWriteFB(FB_CONNECTED, true);
      }
    }
    else
    { // It seems that first request is failed, let's wait for 1s and try again for the second time
      delay(1000);
      responseStatus = fetchURL(FPSTR(identifyURL), json, httpCode);
      if (httpCode == HTTP_CODE_OK)
      { // Success fetched!, store the message to responseStatus!
        if (responseStatus == "success" || responseStatus == "recon")
        {
          writeToEEPROM(USERNAME, usrn);
          bitWriteFB(FB_CONNECTED, true);
        }
      }
      else // It failed once again, probably the WiFi is offline or server is offline, let's report
        responseStatus = "nocon";
    }
  }
  else // Cannot connect to WiFi, report invalid SSID or Password!
    responseStatus = "invwifi";
  // Reinitiate softAP and re deploy the web server to 192.168.4.1
  closeClient();
  closeServer();
  closeSoftAP();
  initiateSoftAP();
  deployWebServer();
  debugMemory();
}

void pgReqStatus()
{
  StaticJsonDocument<240> doc;
  String json;
  doc[F("usrn")] = storedUsername.c_str();
  doc[F("ssid")] = storedSSID.c_str();
  doc[F("wfpw")] = storedWifiPass.c_str();
  doc[F("message")] = responseStatus.c_str();
  serializeJson(doc, json);
  Serial.printf("JSON Size : %d\n", doc.memoryUsage());
  Serial.printf("Transferring JSON : %s\n", json.c_str());
  server.send(200, "application/json", json);
  debugMemory();
}

void handleNotFound()
{
  server.send(404, "text/plain", F("404: Not found"));
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
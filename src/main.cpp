/*
Todo :
DONE Add softSSID,softPW,softIPAddress at setting menu of otoma web
DONE Check last update of ESP8266 on server to notify the user when the ESP8266 is not updating for more than 5 minute (because disconnect)
CANCELLED Add notifier on otoma web when user update automation program but automation program is not fetched by ESP8266 for some reason (disconnected or such)
DONE Add buzzer notification for WiFi fail connect, disconnect, time out.
DONE Add LED status for WiFi connecting, and setup status
DONE Create Shift Register API -- done part
POSTPONED Create Analog Input API -- done part
DONE Remove != and == from comparator list
DONE Lol, the name is operator, not comparator. to be exact, it was relational operators
- Lk sempet add OTA Web Server HTTP Update
DONE Fix parse input on web automation program
- Optimize stack usage by moving memory usage to heap with malloc/free/calloc
DONE Do reload status just for read-only thing (like temp, humid)
DONE Add timeout response for AJAX on web
DONE Please remove that useless auto filter of pid/hys parameter input
DONE PARTIAL Create our own parser for jadwal harian picker (fkin buggy)
DONE Reload Status of web every 3 second, and create a flag that there are update from ESP, and web will fetch it and server will clear that flag.
X Change the logic of plot graphing
- Add stop/pause for automation program, if paused controller will ignore it. 
- Add PID output status on web
DONE Add timeout feature
- Improve update fetch algorithm
DONE polish PID algorithm
- Fix offline request crash

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
// #include <WiFiClientSecure.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <time.h>
#include <constants.h>
#include <ESP8266httpUpdate.h>

#include <Ticker.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <RTClib.h>
#include <Eeprom24C04_16.h>
#include "DHT.h"
#include <ShiftRegister74HC595.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Blinker.h>
#include <ClickEncoder.h>
#include <LiquidCrystal_I2C.h>

void debugMemory(); // Function to check free stack and heap remaining
void storeString(int addrOffset, const String &strToWrite);
byte *readString(int addrOffset);
const String readFromEEPROM(byte what);
void writeToEEPROM(byte what, const String &strToWrite);
byte byteReadFB();
void byteWriteFB(byte data);
bool bitReadFB(byte docchi);
void loadAllPrograms();

void loadInfo();
bool initiateSoftAP();
bool initiateClient(const String &ssid, const String &pass);
void deployWebServer();
void closeServer();
void closeSoftAP();
void closeClient();
bool isWifiConnected();
void fetchURL(const String &URL, const String &payload, int &responseCode, String &response);
void writeByteEEPROM(int addr, byte data);
void pgRoot(); // function prototypes for HTTP handlers
void pgInit();
void pgRestart();
void pgReqStatus();
void pgAccInfo();
void handleNotFound();
void toggle595Bit(byte docchi);
void bitWrite595(uint8_t docchi, uint8_t status);
byte byteRead595();
bool bitRead595(byte docchi);
void byteWrite595(const uint8_t data);
void selectMux(byte channel);
int readMux();
void programScan(void);
void encoderInit();
void encoderUpdate(bool &direction, uint8_t &buttonstate, int16_t &encvalue, int16_t &delta);
void encoderService();
void lcdSetup();
void lcdTransition(int screen, int progNum = 0);
void lcdUpdate();

HTTPClient http;
WiFiClient client;
// BearSSL::WiFiClientSecure *client = new BearSSL::WiFiClientSecure();
// BearSSL::X509List cert;
// BearSSL::Session session;
ESP8266WebServer server(80); // Create a webserver object that listens for HTTP request on port 80
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "asia.pool.ntp.org", 25200); // 25200 for UTC+7, 3600*(UTC)
static Eeprom24C04_16 eeprom(EEPROM_ADDRESS);
RTC_DS1307 rtc;
DHT dht(DHTPIN, DHTTYPE);
ShiftRegister74HC595<1> sr(DATA595, CLOCK595, LATCH595);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b(&oneWire);
Ticker programScanner;
Ticker encoderServicer;
Blinker statusLED(SFT_LED_STATUS, &bitWrite595);
Blinker statusBuzzer(SFT_BUZZER, &bitWrite595);
LiquidCrystal_I2C lcd(0x27, 20, 4);

uint8_t buzzerErrorPattern[] = {1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0},
        buzzerSuccessPattern[] = {1, 0, 1, 0, 1, 0, 0, 0, 0, 0};

ClickEncoder *encoder;

ClickEncoder::Button b;
int16_t enc_last, enc_value, enc_lastbutstate;
bool dir;
uint8_t bs;
int16_t ev;
int16_t dt;

bool programStarted = false,
     blinkerStarted = false;

String storedUsername,
    storedSSID,
    storedWifiPass,
    storedSoftSSID,
    storedSoftWifiPass,
    storedUserpass,
    storedDeviceToken;
byte storedFirstByte;
bool serverAvailable,
    triedLog = false;
unsigned long disconnectStamp,
    requestMillis,
    sensorMillis,
    updateCheckMillis,
    lcdUpdateMillis;

byte dhtSampleCounter,
    byte595Status,
    deviceStatus,
    htclMode;
bool disconnectFlag,
    bit595Status;

float newTemp,
    newHumid;

bool hysteresisBuffer[30];

byte progTrig[30],
    progRB1[30][4],
    progRB2[30][4],
    progAct[30];
bool progFlag[30];

int lcdScreen = 1,
    lcdCursor,
    lcdRowPos,
    lcdCursorBackPos;
bool lcdCursorBlinkFlag,
    lcdCursorBlinkStatus;
unsigned long lcdCursorBlinkTimer;
bool encSelecting;
byte encSelectorCounter;
unsigned long encSelectorTimer;
int encUpperLimit;
const int encLowerLimit = 0;
unsigned long screen2millis;
byte activeProgCount = 0;
byte activeProg[30];

// Page 1 dynamics
// suhu,humid and sp respectively
float prev_newTemp,
    prev_newHumid;
bool prev_wifiStatus;

// Page 2 dynamics
// 0 is date, 1 is month, 2 is hour, 3 is minute, 4 is second
// byte prev_jdw[5];
// int prev_year;

// Page 3 dynamics
bool prev_outStatus[4];

char lcdRowBuffer[4][20];

// Page 7 had no dynamics

// Page 8 dynamics
// byte prev_activeProgramCount,
//     prev_activeProgram;

// Arbitrary page 9 dynamics
// byte prev_trig,
//     prev_rb1[4],
//     prev_rb2[4],
//     prev_act;

// cursor dynamics
int prev_lcdCursor;
int prev_lcdRowPos;
int ds18bFaultCounter;

unsigned long averageRequestTime,
    totalRequestTime,
    maximumRequestTime,
    minimumRequestTime = 99999999,
    totalRequestCount,
    failedRequestCount,
    successRequestCount;

const String freeSketch = String(ESP.getFreeSketchSpace()),
             sketchSize = String(ESP.getSketchSize()),
             chipSize = String(ESP.getFlashChipSize()),
             sketchMD5 = ESP.getSketchMD5();

void setup(void)
{
  std::fill_n(hysteresisBuffer, 30, false);
  Serial.begin(74880); // Start the Serial communication to send messages to the computer
  Serial.printf(
      "freeSketch : %s\nsketchSize : %s\nchipSize : %s\nsketchMD5 : %s\n", freeSketch.c_str(), sketchSize.c_str(), chipSize.c_str(), sketchMD5.c_str());
  Serial.println("Init");
  ESP.resetFreeContStack();
  uint32_t freeStackStart = ESP.getFreeContStack();
  storedUsername.reserve(32);
  storedUserpass.reserve(64);
  storedSSID.reserve(32);
  storedWifiPass.reserve(64);
  storedSoftSSID.reserve(32);
  storedSoftWifiPass.reserve(64);
  storedDeviceToken.reserve(10);
  statusBuzzer.on();
  // pinMode(LED_BUILTIN, OUTPUT);
  delay(100);
  statusBuzzer.off();
  lcdSetup();

  if (!rtc.begin())
  {
    lcd.print("GAGAL MEMULAI SISTEM");
    lcd.setCursor(0, 1);
    lcd.print("RTC ERROR");
    Serial.println("Couldn't Start RTC!");
    // failed to init rtc
    // add some loud ass error buzzer here
    // rtc is important element, so stop right here if something is wrong

    statusBuzzer.off();
    statusBuzzer.on();
    delay(2000);
    statusBuzzer.off();
    delay(2000);
    statusBuzzer.on();
    delay(2000);
    statusBuzzer.off();
    delay(500);
    ESP.restart();
  }
  eeprom.initialize();
  if (eeprom.readByte(0) == 0xff && eeprom.readByte(1) == 0xff && eeprom.readByte(10) == 0xff)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Men-setup sistem");
    lcd.setCursor(0, 1);
    lcd.print("Tunggu sebentar");
    byte zeroBlank[PROG_LENGTH];
    std::fill_n(zeroBlank, PROG_LENGTH, 0);
    eeprom.writeBytes(ADDR_DEVICE_STATUS, PROG_LENGTH, zeroBlank);
    eeprom.writeByte(0, 0);
    delay(10);
    writeToEEPROM(WIFISSID, " ");
    writeToEEPROM(WIFIPW, " ");
    writeToEEPROM(SOFTSSID, "Otoma Nexus");
    writeToEEPROM(SOFTPW, "12121313");
    writeToEEPROM(USERNAME, " ");
    writeToEEPROM(USERPASS, " ");
    writeToEEPROM(DEVICETOKEN, "4f6YqT9GdI");
    /*
    4f6YqT9GdI
    dNLxiQAUAL
    aXvNuWsuec
    oT2pFKfAnS
    zdrmhiDoqR
    Md5zpyoKtE
    CRNAnQV0GR
    aRdG5Pn0aI
    WraEc78ODU
    1MC2BDiZQR
    */
  }
  delay(100);
  ds18b.begin();
  delay(100);
  dht.begin();
  delay(100);
  newHumid = dht.readHumidity();
  delay(100);
  encoderInit();
  loadInfo();
  //75753us
  //512006us
  http.setReuse(true);
  if (ds18b.getDS18Count() == 0)
  {
    if (!bitReadFB(FB_DS_NF1))
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sensor Suhu Error!");
      lcd.setCursor(0, 1);
      lcd.print("Mengulang sistem...");
      Serial.println("No DS18B20 FOUND!\nRestarting!");
      bitWrite(storedFirstByte, FB_DS_NF1, true);
      eeprom.writeByte(ADDR_FIRST_BYTE, storedFirstByte);
      statusBuzzer.on();
      delay(2000);
      statusBuzzer.off();
      delay(500);
      ESP.restart();
    }
    else if (!bitReadFB(FB_DS_NF2))
    {
      lcd.setCursor(0, 0);
      lcd.print("Sensor Suhu Error!");
      lcd.setCursor(0, 1);
      lcd.print("Mengulang sistem...");
      Serial.println("No DS18B20 FOUND!\nRestarting!");
      bitWrite(storedFirstByte, FB_DS_NF2, true);
      eeprom.writeByte(ADDR_FIRST_BYTE, storedFirstByte);
      statusBuzzer.on();
      delay(2000);
      statusBuzzer.off();
      delay(500);
      ESP.restart();
    }
    else
    {
      lcd.setCursor(0, 0);
      lcd.print("Sensor Suhu Error!");
      lcd.setCursor(0, 1);
      lcd.print("Melanjutkan");
      lcd.setCursor(0, 2);
      lcd.print("tanpa sensor suhu");
      statusBuzzer.on();
      delay(2000);
      statusBuzzer.off();
      delay(500);
      Serial.println("No DS18B20 FOUND!\nProgress without DS18B20!");
      bitWrite(storedFirstByte, FB_DS_NF1, false);
      bitWrite(storedFirstByte, FB_DS_NF2, false);
      eeprom.writeByte(ADDR_FIRST_BYTE, storedFirstByte);
      delay(10);
    }
  }
  else
  {
    if (bitReadFB(FB_DS_NF1) || bitReadFB(FB_DS_NF2))
    {
      bitWrite(storedFirstByte, FB_DS_NF1, false);
      bitWrite(storedFirstByte, FB_DS_NF2, false);
      eeprom.writeByte(ADDR_FIRST_BYTE, storedFirstByte);
      delay(10);
    }
  }

  loadAllPrograms();
  ds18b.requestTemperatures(); // Send the command to get temperatures
  newTemp = ds18b.getTempCByIndex(0);

  byteWrite595(0x00);
  programStarted = false;
  programScanner.attach_ms(40, programScan);
  encoderServicer.attach_ms(1, encoderService);

  closeClient();
  closeSoftAP();
  closeServer();
  programStarted = true;
  if (!bitReadFB(FB_CONNECTED))
  {
    initiateSoftAP();
    deployWebServer(); // Deploy web server
  }
  else
  {
    int timeOutCounter = 0;
    while (timeOutCounter < 2) // Try for maximum of 5 attempts
    {
      if (initiateClient(storedSSID, storedWifiPass))
      {
        if (bitReadFB(FB_WIFI_ERROR1) || bitReadFB(FB_WIFI_ERROR2))
        {
          bitWrite(storedFirstByte, FB_WIFI_ERROR1, false);
          bitWrite(storedFirstByte, FB_WIFI_ERROR2, false);
          eeprom.writeByte(ADDR_FIRST_BYTE, storedFirstByte);
          delay(10);
        }
        break;
      }
      timeOutCounter++;
    }
    if (timeOutCounter >= 2) // If timed out for 5 attempts after 2 reboot, rollback to softAP mode and reset FB
    {
      Serial.println("Fail reconnect WiFi");
      if (!bitReadFB(FB_WIFI_ERROR1))
      {
        Serial.println("Rebooting attempt 1");
        bitWrite(storedFirstByte, FB_WIFI_ERROR1, true);
        eeprom.writeByte(ADDR_FIRST_BYTE, storedFirstByte);
        delay(500);
        ESP.restart();
      }
      else
      {
        if (!bitReadFB(FB_WIFI_ERROR2))
        {
          Serial.println("Rebooting attempt 2");
          bitWrite(storedFirstByte, FB_WIFI_ERROR2, true);
          eeprom.writeByte(ADDR_FIRST_BYTE, storedFirstByte);
          delay(500);
          ESP.restart();
        }
        else
        {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Gagal menyambung WiFi");
          lcd.setCursor(0, 1);
          lcd.printf("%.20s", storedSSID.c_str());
          delay(1000);
          initiateSoftAP(); // Seems that SSID is invalid, initiate Soft AP instead
          deployWebServer();
          bitWrite(storedFirstByte, FB_CONNECTED, false);
          bitWrite(storedFirstByte, FB_WIFI_ERROR1, false);
          bitWrite(storedFirstByte, FB_WIFI_ERROR2, false);
          eeprom.writeByte(ADDR_FIRST_BYTE, storedFirstByte);
          delay(10);
        }
      }
    }
  }
  if (isWifiConnected())
  {
    timeClient.begin(); // If WiFi connection success, start NTP timeClient
    if (timeClient.forceUpdate())
    {
      Serial.println("Successfully fetching NTP Clock!");
      rtc.adjust(DateTime(timeClient.getEpochTime()));
    }
    else
    {
      Serial.println("Failed fetching NTP Clock");
      DateTime now = rtc.now();
      if (!rtc.isrunning() || now < 1577836800)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("RTC Gagal");
        lcd.setCursor(0, 1);
        lcd.print("Merestart kontroller");
        Serial.println("Could'nt Initiate RTC!");
        // We're fucked up my friend, RTC is not running and can't fetch NTP time, so we'll stop right there too
        // failed to init rtc
        // add some loud ass error buzzer here
        // rtc is important element, so stop right here if something is wrong
        statusBuzzer.off();
        statusBuzzer.on();
        delay(2000);
        statusBuzzer.off();
        delay(2000);
        statusBuzzer.on();
        delay(2000);
        statusBuzzer.off();
        delay(500);
        ESP.restart();
      }
      else
      {
        Serial.printf("Initializing client x509 time RTC : %lu", rtc.now().unixtime());
      }
    }

    programStarted = false;
    t_httpUpdate_return ret = ESPhttpUpdate.update(client, FPSTR(espUpdater), BUILD_VERSION);
    switch (ret)
    {
    case HTTP_UPDATE_FAILED:
      Serial.println("[update] Update failed.");
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("[update] Update no Update.");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("[update] Update ok."); // may not be called since we reboot the ESP
      break;
    }
    programStarted = true;
  }

  lcdTransition(1);
  uint32_t freeStackEnd = ESP.getFreeContStack();
  Serial.printf("\nCONT stack used at start: %d\n-------\n\n", freeStackStart - freeStackEnd);
}

void loop(void)
{
  if (serverAvailable)
  {
    server.handleClient(); // Listen for HTTP requests from clients
  }
  else
  {
    if (isWifiConnected())
    {
      if (millis() - requestMillis >= HTTP_FETCH_INTERVAL)
      {
        ESP.resetFreeContStack();
        if (totalRequestCount % 100000 == 0)
          totalRequestTime = 0;
        totalRequestCount++;
        uint32_t freeStackStart = ESP.getFreeContStack();
        uint32_t timeTakenStart = millis();
        DateTime now = rtc.now();
        if (timeClient.update())
        {
          // If unixtime of RTC is deviating + or - 30 seconds than NTP time then readjust RTC
          if (timeClient.getEpochTime() > now.unixtime() + 10 || timeClient.getEpochTime() < now.unixtime() - 10)
            rtc.adjust(DateTime(timeClient.getEpochTime()));
        }
        const size_t capacity = JSON_ARRAY_SIZE(3) + JSON_OBJECT_SIZE(9);
        DynamicJsonDocument doc(capacity);
        String json;
        // Type is splitted by 'n'
        // t means temperature
        // h means humidity
        // s means status of aux1,aux2,th,ht,cl (ordered)
        // the highest order is t and goes lower to s
        // so tnhns means ESP8266 is sending temperature and humidity
        doc[F("type")] = F("tnhns");
        doc[F("unix")] = now.unixtime();
        doc[F("a1")] = bitRead(deviceStatus, BITPOS_AUX1_STATUS);
        doc[F("a2")] = bitRead(deviceStatus, BITPOS_AUX2_STATUS);
        doc[F("a3")] = bitRead(deviceStatus, BITPOS_AUX3_STATUS);
        doc[F("a4")] = bitRead(deviceStatus, BITPOS_AUX4_STATUS);

        JsonArray data = doc.createNestedArray("data");
        data.add(newTemp);
        (!isnan(newHumid)) ? data.add(newHumid) : data.add(0.1);

        serializeJson(doc, json);
        int httpCode;
        String response;
        fetchURL(FPSTR(requestURL), json, httpCode, response);
        Serial.println(response);
        if (httpCode > 0)
        {
          if (httpCode == HTTP_CODE_OK || httpCode >= 500 || (httpCode > 200 && httpCode < 300)) // Successful Fetch or Server Mistake reset disconnectFlag
          {
            disconnectFlag = false;
            disconnectStamp = millis();
          }
          else if (httpCode >= 400 && httpCode < 500 && !disconnectFlag)
          {
            disconnectFlag = true;
            disconnectStamp = millis();
          }
          if (httpCode == HTTP_CODE_OK)
          {
            DynamicJsonDocument out(4096);
            deserializeJson(out, response);
            const char *order = out["order"];
            if (order)
            {
              if (strcmp(order, "setParam") == 0)
              {
                statusBuzzer.setInterval(300);
                statusBuzzer.disableAfter(301);
                byte *buffer = (byte *)malloc(4);
                if (out["st"])
                {
                  if (!out["auxStatus1"].isNull())
                    bitWrite(deviceStatus, BITPOS_AUX1_STATUS, out["auxStatus1"].as<bool>());
                  if (!out["auxStatus2"].isNull())
                    bitWrite(deviceStatus, BITPOS_AUX2_STATUS, out["auxStatus2"].as<bool>());
                  if (!out["auxStatus3"].isNull())
                    bitWrite(deviceStatus, BITPOS_AUX3_STATUS, out["auxStatus3"].as<bool>());
                  if (!out["auxStatus4"].isNull())
                    bitWrite(deviceStatus, BITPOS_AUX4_STATUS, out["auxStatus4"].as<bool>());
                  delay(10);
                  Serial.printf("Received Status Update\nAux 1 Status : %d\nAux 2 Status : %d\nAux 3 Status : %d\nAux 4 Status : %d\n", bitRead(deviceStatus, BITPOS_AUX1_STATUS), bitRead(deviceStatus, BITPOS_AUX2_STATUS), bitRead(deviceStatus, BITPOS_AUX3_STATUS), bitRead(deviceStatus, BITPOS_AUX4_STATUS));
                }
                if (out["prog"])
                {
                  char *num = (char *)malloc(sizeof(char) * 3);
                  for (byte i = 0; i < 30; i++)
                  {
                    sprintf(num, "%d", i);
                    if (out["prog"][num])
                    {
                      Serial.printf("Updating program %s\n", num);
                      unsigned long lbuffer = out["prog"][num][0].as<unsigned long>(); // Trigger Type
                      memcpy(buffer, &lbuffer, 1);
                      progTrig[i] = buffer[0];
                      eeprom.writeByte(ADDR_PROG_TRIGGER(i), buffer[0]);
                      delay(10);
                      lbuffer = out["prog"][num][1].as<unsigned long>(); // RB 1
                      memcpy(buffer, &lbuffer, 4);
                      memcpy(&progRB1[i], buffer, 4);
                      eeprom.writeBytes(ADDR_PROG_RB1(i), 4, buffer);
                      lbuffer = out["prog"][num][2].as<unsigned long>(); // RB 2
                      memcpy(buffer, &lbuffer, 4);
                      memcpy(&progRB2[i], buffer, 4);
                      eeprom.writeBytes(ADDR_PROG_RB2(i), 4, buffer);
                      lbuffer = out["prog"][num][3].as<unsigned long>(); // Action Type
                      memcpy(buffer, &lbuffer, 1);
                      progAct[i] = buffer[0];
                      eeprom.writeByte(ADDR_PROG_ACTION(i), buffer[0]);
                      delay(10);

                      Serial.printf("Trigger Type : %d\nRB1 : 0x%X%X%X%X,\nRB2 : 0x%X%X%X%X\nAction Type : %d\n", progTrig[i], progRB1[i][3], progRB1[i][2], progRB1[i][1], progRB1[i][0], progRB2[i][3], progRB2[i][2], progRB2[i][1], progRB2[i][0], progAct[i]);
                    }
                  }
                  free(num);
                }

                free(buffer);
              }
              else if (strcmp(order, "fallback") == 0)
              {
                bitWrite(storedFirstByte, FB_CONNECTED, false);
                eeprom.writeByte(ADDR_FIRST_BYTE, storedFirstByte);
                delay(10);
                delay(500);
                ESP.reset();
              }
              else if (strcmp(order, "restart") == 0)
              {
                delay(500);
                ESP.reset();
              }
            }
          }
        }
        else
        {
          if (!disconnectFlag)
          {
            disconnectFlag = true;
            disconnectStamp = millis();
          }
        }

        requestMillis = millis();
        debugMemory();
        uint32_t freeStackEnd = ESP.getFreeContStack();
        uint32_t timeTaken = millis() - timeTakenStart;
        if (timeTaken > maximumRequestTime)
          maximumRequestTime = timeTaken;
        if (timeTaken < minimumRequestTime)
          minimumRequestTime = timeTaken;
        totalRequestTime += timeTaken;
        averageRequestTime = totalRequestTime / (totalRequestCount % 100000);
        Serial.printf("Main online routine takes : %lums - average %lums - minimum %lums - maximum %lums\nTotal request count %lu - fail %lu - success %lu %04.2f%%\nCONT stack used at main fetch url: %lu\n-------\n\n", timeTaken, averageRequestTime, minimumRequestTime, maximumRequestTime, totalRequestCount, failedRequestCount, successRequestCount, float(successRequestCount) / float(totalRequestCount) * 100.0, freeStackStart - freeStackEnd);
      }

      if (millis() - updateCheckMillis >= UPDATE_CHECK_INTERVAL)
      {
        Serial.println("Checking update");
        programStarted = false;
        byteWrite595(0x00);
        t_httpUpdate_return ret = ESPhttpUpdate.update(client, FPSTR(espUpdater), BUILD_VERSION);
        switch (ret)
        {
        case HTTP_UPDATE_FAILED:
          Serial.println("[update] Update failed.");
          break;
        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("[update] Update no Update.");
          break;
        case HTTP_UPDATE_OK:
          Serial.println("[update] Update ok."); // may not be called since we reboot the ESP
          break;
        }
        programStarted = true;
        updateCheckMillis = millis();
      }
    }
    else
    {
      if (!disconnectFlag)
      {
        disconnectFlag = true;
        disconnectStamp = millis();
      }
    }
  }
  if (disconnectFlag && millis() - disconnectStamp >= MAXIMUM_DISCONNECT_TIME)
  {
    writeToEEPROM(WIFISSID, "KONEKSI TERPUTUS");
    writeToEEPROM(WIFIPW, " ");
    bitWrite(storedFirstByte, FB_CONNECTED, false);
    eeprom.writeByte(ADDR_FIRST_BYTE, storedFirstByte);
    delay(10);
    statusBuzzer.off();
    statusBuzzer.on();
    delay(2000);
    statusBuzzer.off();
    delay(2000);
    statusBuzzer.on();
    delay(2000);
    statusBuzzer.off();
    delay(500);
    ESP.reset();
  }
  if (millis() - sensorMillis >= SENSOR_UPDATE_INTERVAL)
  {
    float *sensorBuffer = (float *)malloc(sizeof(float) * 2);
    if (dhtSampleCounter <= 0)
    {
      sensorBuffer[1] = dht.readHumidity();
      if (!isnan(sensorBuffer[1]))
        newHumid = sensorBuffer[1];
      dhtSampleCounter = DHT_LOOP;
      delay(100);
    }
    dhtSampleCounter--;
    ds18b.requestTemperatures(); // Send the command to get temperatures
    for (uint8_t count = ds18b.getDS18Count(); count > 0; count--)
    {
      sensorBuffer[0] += ds18b.getTempCByIndex(count - 1); // index of sensor start at 0
    }
    sensorBuffer[0] /= ds18b.getDS18Count();
    if (sensorBuffer[0] > 0.00)
    {
      newTemp = sensorBuffer[0];
      ds18bFaultCounter = 0;
    }
    else if (isnan(sensorBuffer[0]) || !(sensorBuffer[0] > 0.00) || isinf(sensorBuffer[0]))
      ds18bFaultCounter++;
    if (newTemp <= 0.0 || newTemp > 200.0)
    {
      newTemp = 0.0;
      ds18bFaultCounter++;
    }
    if (newHumid < 2.0 || newHumid > 100.0)
      newHumid = 0.0;
    free(sensorBuffer);
    if (ds18bFaultCounter >= 30)
    {
      programStarted = false;
      byteWrite595(0x00);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sensor Suhu Error!");
      lcd.setCursor(0, 1);
      lcd.print("Merestart kontroller");
      statusBuzzer.on();
      delay(2000);
      statusBuzzer.off();
      delay(200);
      ESP.restart();
    }
    sensorMillis = millis();
  }
}

void debugMemory()
{
  Serial.printf("Stack : %d\nHeap : %d\n", ESP.getFreeContStack(), ESP.getFreeHeap());
}

void programScan(void)
{
  if (programStarted)
  {
    bool statusBuffer[10];
    statusBuffer[0] = bitRead(deviceStatus, BITPOS_AUX1_STATUS);
    statusBuffer[1] = bitRead(deviceStatus, BITPOS_AUX2_STATUS);
    statusBuffer[2] = bitRead(deviceStatus, BITPOS_AUX3_STATUS);
    statusBuffer[3] = bitRead(deviceStatus, BITPOS_AUX4_STATUS);
    encoderUpdate(dir, bs, ev, dt); //
    if (millis() - lcdUpdateMillis >= LCD_UPDATE_INTERVAL)
    {
      lcdUpdate();
      lcdUpdateMillis = millis();
    }
    if (millis() >= encSelectorTimer + 400)
    {
      encSelectorTimer = millis();
      encSelectorCounter = 0;
    }
    if (dt != 0)
    { // If encoder is rotated and it's not selecting, then
      // Need 2 step of encode to move the cursor, and if no movement, the counter will reset.
      encSelectorCounter++;
      encSelectorTimer = millis();
      if (encSelectorCounter >= 2)
      {
        if (dir == ENC_UP) // if the rotation is clockwise, then add value of lcdCursor
          lcdCursor++;
        else // if the rotation is counter-clockwise, then reduce value of lcdCursor
          lcdCursor--;
        encSelectorCounter = 0;
      }
      //Constrain the value of lcdCursor according to each lcdScreen limit
      if (lcdScreen == 1)
        encUpperLimit = 0;
      else if (lcdScreen == 3 || lcdScreen == 2)
        encUpperLimit = 4;
      else if (lcdScreen == 4)
        encUpperLimit = 5;
      else if (lcdScreen == 6 || lcdScreen == 5)
        encUpperLimit = 7;
      if (lcdCursor < encLowerLimit)
        lcdCursor = encLowerLimit;
      if (lcdCursor > encUpperLimit)
        lcdCursor = encUpperLimit;
    }

    if (bs == ENC_CLICKED)
    { // Navigation
      if (lcdScreen == 1)
        lcdTransition(2);
      else if (lcdScreen == 2)
      {
        if (lcdCursor == 0)
          lcdTransition(3);
        else if (lcdCursor == 1)
          lcdTransition(8);
        else if (lcdCursor == 2)
          lcdTransition(7);
        else if (lcdCursor == 3)
          lcdTransition(1);
      }
      else if (lcdScreen == 3)
      {
        if (lcdCursor == 4)
          lcdTransition(2);
      }
      else if (lcdScreen == 4)
      {
        if (lcdCursor == 5)
          lcdTransition(2);
        if (lcdCursor == 4)
          lcdTransition(6);
        if (lcdCursor == 3)
          lcdTransition(5);
      }
      else if (lcdScreen == 5)
      {
        if (lcdCursor == 7)
          lcdTransition(4);
      }
      else if (lcdScreen == 6)
      {
        if (lcdCursor == 7)
          lcdTransition(4);
      }
      else if (lcdScreen == 7)
        lcdTransition(2);
      else if (lcdScreen == 8)
      {
        if (lcdCursor < 30 && lcdCursorBackPos != lcdCursor)
          lcdTransition(9, activeProg[lcdCursor] - 1);
        if (lcdCursorBackPos == lcdCursor)
          lcdTransition(2);
      }
      else if (lcdScreen == 9)
        lcdTransition(8);
    }
    if (bs == ENC_DBCLICKED)
    {
      if (lcdScreen == 2 && lcdCursor == 4)
      {
        statusBuzzer.on();
        delay(500);
        statusBuzzer.off();
        delay(100);
        ESP.restart();
      }
      if (lcdScreen == 3)
        statusBuffer[lcdCursor] = !statusBuffer[lcdCursor];
    }
    statusLED.update();
    statusBuzzer.update();

    for (int i = 0; i < 30; i++)
    {
      if (progTrig[i] != 0)
      {
        bool condition;
        if (progTrig[i] == 1 || progTrig[i] == 2)
        {
          float copyValue[2];
          memcpy(&copyValue[0], &progRB2[i], sizeof(float));
          copyValue[1] = (progTrig[i] == 1) ? newTemp : newHumid;

          // We only care about first byte, because nilai suhu's or humidity's(operator) value is no more than 3
          if (progRB1[i][0] == 0)
            condition = (copyValue[1] < copyValue[0]);
          else if (progRB1[i][0] == 1)
            condition = (copyValue[1] > copyValue[0]);
          else if (progRB1[i][0] == 2)
            condition = (copyValue[1] <= copyValue[0]);
          else if (progRB1[i][0] == 3)
            condition = (copyValue[1] >= copyValue[0]);
          else
            condition = false;

          if (condition)
          {
            if (progAct[i] == 1 || progAct[i] == 5)
              statusBuffer[0] = (progAct[i] == 1) ? MURUP : MATI;
            if (progAct[i] == 2 || progAct[i] == 6)
              statusBuffer[1] = (progAct[i] == 2) ? MURUP : MATI;
            if (progAct[i] == 3 || progAct[i] == 7)
              statusBuffer[2] = (progAct[i] == 3) ? MURUP : MATI;
            if (progAct[i] == 4 || progAct[i] == 8)
              statusBuffer[3] = (progAct[i] == 4) ? MURUP : MATI;
          }
        }
        else if (progTrig[i] == 3 || progTrig[i] == 4)
        {
          DateTime now = rtc.now();
          bool condition;
          unsigned long uCopyValue[3];
          memcpy(&uCopyValue[0], &progRB1[i], sizeof(unsigned long));
          memcpy(&uCopyValue[1], &progRB2[i], sizeof(unsigned long));
          if (progTrig[i] == 4)
          {
            if (now.unixtime() >= uCopyValue[0])
            {
              condition = (now.unixtime() < uCopyValue[1]) ? true : false;
              if (condition)
              {
                if (progAct[i] < 5)
                  statusBuffer[progAct[i] - 1] = MURUP;
                else if (progAct[i] >= 5)
                  statusBuffer[progAct[i] - 5] = MATI;
                progFlag[i] = true;
              }
              else if (progFlag[i])
              {
                if (progAct[i] < 5)
                  statusBuffer[progAct[i] - 1] = MURUP;
                else if (progAct[i] >= 5)
                  statusBuffer[progAct[i] - 5] = MATI;
                progFlag[i] = false;
              }
            }
          }
          else if (progTrig[i] == 3)
          {
            uCopyValue[2] = (now.hour() * 3600) + (now.minute() * 60) + now.second();
            condition = (uCopyValue[2] >= uCopyValue[0] && uCopyValue[2] <= uCopyValue[1]) ? true : false;
            if (condition)
            {
              if (progAct[i] < 5)
                statusBuffer[progAct[i] - 1] = MURUP;
              else if (progAct[i] >= 5)
                statusBuffer[progAct[i] - 5] = MATI;
              progFlag[i] = true;
            }
            else if (progFlag[i])
            {
              if (progAct[i] < 5)
                statusBuffer[progAct[i] - 1] = MURUP;
              else if (progAct[i] >= 5)
                statusBuffer[progAct[i] - 5] = MATI;
              progFlag[i] = false;
            }
          }
        }
        else if (progTrig[i] == 5 && progRB2[i][0] != 0)
        {
          if (progRB2[i][0] != 0)
          {
            if (progRB1[i][0] == 0)
              condition = (bitRead(deviceStatus, BITPOS_AUX1_STATUS) == ((progRB2[i][0] == 1) ? MURUP : MATI));
            else if (progRB1[i][0] == 1)
              condition = (bitRead(deviceStatus, BITPOS_AUX2_STATUS) == ((progRB2[i][0] == 1) ? MURUP : MATI));
            else if (progRB1[i][0] == 2)
              condition = (bitRead(deviceStatus, BITPOS_AUX3_STATUS) == ((progRB2[i][0] == 1) ? MURUP : MATI));
            else if (progRB1[i][0] == 3)
              condition = (bitRead(deviceStatus, BITPOS_AUX4_STATUS) == ((progRB2[i][0] == 1) ? MURUP : MATI));
            else
              condition = false;
            if (condition)
            {
              if (progAct[i] == 1 || progAct[i] == 5)
                statusBuffer[0] = (progAct[i] == 1) ? MURUP : MATI;
              if (progAct[i] == 2 || progAct[i] == 6)
                statusBuffer[1] = (progAct[i] == 2) ? MURUP : MATI;
              if (progAct[i] == 3 || progAct[i] == 7)
                statusBuffer[2] = (progAct[i] == 3) ? MURUP : MATI;
              if (progAct[i] == 4 || progAct[i] == 8)
                statusBuffer[3] = (progAct[i] == 4) ? MURUP : MATI;
            }
          }
        }
        else if (progTrig[i] == 6 || progTrig[i] == 7 || progTrig[i] == 8)
        {
          float aturKe, toleransi;
          memcpy(&aturKe, &progRB1[i], sizeof(float));
          memcpy(&toleransi, &progRB2[i], sizeof(float));
          if (((progTrig[i] == 6 || progTrig[i] == 7) ? newTemp : newHumid) > aturKe + (toleransi / 2))
            hysteresisBuffer[i] = ((progTrig[i] == 7) ? MURUP : MATI);
          else if (((progTrig[i] == 6 || progTrig[i] == 7) ? newTemp : newHumid) < aturKe - (toleransi / 2))
            hysteresisBuffer[i] = ((progTrig[i] == 7) ? MATI : MURUP);
          statusBuffer[progAct[i]] = hysteresisBuffer[i];
        }
      }
    }

    bitWrite595(SFT_AUX1_RELAY, statusBuffer[0]);
    bitWrite595(SFT_AUX2_RELAY, statusBuffer[1]);
    bitWrite595(SFT_AUX3_RELAY, statusBuffer[2]);
    bitWrite595(SFT_AUX4_RELAY, statusBuffer[3]);
    bitWrite(deviceStatus, BITPOS_AUX1_STATUS, statusBuffer[0]);
    bitWrite(deviceStatus, BITPOS_AUX2_STATUS, statusBuffer[1]);
    bitWrite(deviceStatus, BITPOS_AUX3_STATUS, statusBuffer[2]);
    bitWrite(deviceStatus, BITPOS_AUX4_STATUS, statusBuffer[3]);
    // free(statusBuffer);
  }
}

/////////////////////////////////////////// LCD API ///////////////////////////////////////////////

void lcdSetup()
{
  lcd.begin();
  lcd.clear();
  lcd.setCursor(0, 0);
}

void lcdTransition(int screen, int progNum)
{
  lcdScreen = screen;
  lcdCursor = 0;
  lcdRowPos = 0;
  lcd.clear();
  if (screen == 1)
  {
    lcd.setCursor(0, 0);
    lcd.printf("Suhu:%05.2f", newTemp);
    lcd.print(LCD_DEGREE); // print [degree] character
    lcd.print("C");
    lcd.setCursor(13, 0);
    lcd.printf("%s", (isWifiConnected()) ? "ONLINE" : "OFFLINE");
    lcd.setCursor(0, 1);
    lcd.printf("Hmdt:    %2.0f%%", newHumid);
    lcd.setCursor(0, 3);
    lcd.print(LCD_ARROW);
    lcd.print("Menu");
  }
  else if (screen == 2)
  {
    DateTime now = rtc.now();
    lcd.setCursor(0, 0);
    lcd.print(LCD_ARROW);
    lcd.printf("Output      Back");
    lcd.setCursor(1, 1);
    lcd.printf("Program     Restart");
    lcd.setCursor(1, 2);
    lcd.printf("Info     %02d/%02d/%04d", now.day(), now.month(), now.year());
    lcd.setCursor(1, 3);
    lcd.printf("          %02d:%02d:%02d", now.hour(), now.minute(), now.second());
  }
  else if (screen == 3)
  {
    lcd.setCursor(0, 0);
    lcd.print(LCD_ARROW);
    lcd.printf("Output 1:%s", (bitRead(deviceStatus, BITPOS_AUX1_STATUS) == MURUP) ? "ON" : "OFF");
    lcd.setCursor(16, 0);
    lcd.print("Back");
    lcd.setCursor(1, 1);
    lcd.printf("Output 2:%s", (bitRead(deviceStatus, BITPOS_AUX2_STATUS) == MURUP) ? "ON" : "OFF");
    lcd.setCursor(1, 2);
    lcd.printf("Output 3:%s", (bitRead(deviceStatus, BITPOS_AUX3_STATUS) == MURUP) ? "ON" : "OFF");
    lcd.setCursor(1, 3);
    lcd.printf("Output 4:%s", (bitRead(deviceStatus, BITPOS_AUX4_STATUS) == MURUP) ? "ON" : "OFF");
  }
  else if (screen == 7)
  {
    lcd.setCursor(0, 0);
    lcd.printf("Soft AP:%.12s", storedSoftSSID.c_str());
    lcd.setCursor(0, 1);
    lcd.printf("Soft PW:%.12s", storedSoftWifiPass.c_str());
    lcd.setCursor(0, 2);
    lcd.print("IP:192.168.4.1"); // this guy is the same on every ESP, so its fine to hardcode
    lcd.setCursor(0, 3);
    lcd.printf("Ver:%s", BUILD_VERSION);
    lcd.setCursor(15, 3);
    lcd.print(LCD_ARROW);
    lcd.print("Back");
  }
  else if (screen == 8)
  {
    activeProgCount = 0;
    std::fill_n(activeProg, 30, 0);
    for (uint8_t i = 0; i < 30; i++)
    {
      if (progTrig[i] != 0)
      {
        activeProg[activeProgCount] = i + 1;
        activeProgCount++;
      }
    }

    if (activeProgCount > 0)
    {
      encUpperLimit = activeProgCount;
      lcdCursorBackPos = activeProgCount;
      lcd.setCursor(0, 0);
      lcd.print(LCD_ARROW);
      if (activeProgCount == 1)
      {
        lcd.printf("Program %d", activeProg[0]);
      }
      else if (activeProgCount == 2)
      {
        lcd.printf("Program %d", activeProg[0]);
        lcd.setCursor(1, 1);
        lcd.printf("Program %d", activeProg[1]);
      }
      else if (activeProgCount == 3)
      {
        lcd.printf("Program %d", activeProg[0]);
        lcd.setCursor(1, 1);
        lcd.printf("Program %d", activeProg[1]);
        lcd.setCursor(1, 2);
        lcd.printf("Program %d", activeProg[2]);
      }
      else if (activeProgCount >= 4)
      {
        lcd.printf("Program %d", activeProg[0]);
        sprintf(lcdRowBuffer[0], "Program %d", activeProg[0]);
        lcd.setCursor(1, 1);
        lcd.printf("Program %d", activeProg[1]);
        sprintf(lcdRowBuffer[1], "Program %d", activeProg[1]);
        lcd.setCursor(1, 2);
        lcd.printf("Program %d", activeProg[2]);
        sprintf(lcdRowBuffer[2], "Program %d", activeProg[2]);
        lcd.setCursor(1, 3);
        lcd.printf("Program %d", activeProg[3]);
        sprintf(lcdRowBuffer[3], "Program %d", activeProg[3]);
      }
    }
    else
    {
      lcdCursorBackPos = 0;
      encUpperLimit = 0;
      lcd.setCursor(11, 0);
      lcd.print(LCD_ARROW);
      lcd.setCursor(2, 0);
      lcd.print("Tidak ada");
      lcd.setCursor(1, 1);
      lcd.print("program");
    }
    lcd.setCursor(16, 0);
    lcd.print("Back");
  }
  else if (screen == 9)
  {
    lcd.setCursor(0, 0);
    lcd.printf("Pemicu:%s", (progTrig[progNum] == 1) ? "Suhu" : (progTrig[progNum] == 2) ? "Humidts" : (progTrig[progNum] == 3) ? "Jdwlhrn" : (progTrig[progNum] == 4) ? "TglWktu" : (progTrig[progNum] == 5) ? "Keadaan" : (progTrig[progNum] == 6) ? "Pemanas" : (progTrig[progNum] == 7) ? "Pendingin" : (progTrig[progNum] == 8) ? "Hmdfier" : "null");
    lcd.setCursor(0, 1);
    if (progTrig[progNum] == 1 || progTrig[progNum] == 2)
    {
      lcd.printf("%s", (progRB1[progNum][0] == 0) ? "<" : (progRB1[progNum][0] == 1) ? ">" : (progRB1[progNum][0] == 2) ? "<=" : (progRB1[progNum][0] == 3) ? ">=" : "null");
      lcd.print(" ");
      float copyValue;
      memcpy(&copyValue, &progRB2[progNum], sizeof(float));
      lcd.printf("%05.1f", copyValue);
    }
    else if (progTrig[progNum] == 3 || progTrig[progNum] == 4)
    {
      unsigned long fromCopy;
      unsigned long toCopy;
      memcpy(&fromCopy, &progRB1[progNum], sizeof(unsigned long));
      memcpy(&toCopy, &progRB2[progNum], sizeof(unsigned long));
      if (progTrig[progNum] == 3)
      {
        lcd.printf("Dari %02d:%02d:%02d", fromCopy / 3600, (fromCopy % 3600) / 60, fromCopy % 60);
        lcd.setCursor(0, 2);
        lcd.printf("Hingga %02d:%02d:%02d", toCopy / 3600, (toCopy % 3600) / 60, toCopy % 60);
      }
      else
      {
        DateTime fromdt((uint32_t)fromCopy);
        DateTime todt((uint32_t)toCopy);
        lcd.printf("%02d/%02d/%04d %02d:%02d", fromdt.day(), fromdt.month(), fromdt.year(), fromdt.hour(), fromdt.minute());
        lcd.setCursor(0, 2);
        lcd.printf("%02d/%02d/%04d %02d:%02d", todt.day(), todt.month(), todt.year(), todt.hour(), todt.minute());
      }
    }
    else if (progTrig[progNum] == 5)
    {
      lcd.printf("%s", (progRB1[progNum][0] == 1) ? "Output 1" : (progRB1[progNum][0] == 2) ? "Output 2" : (progRB1[progNum][0] == 3) ? "Pemanas" : (progRB1[progNum][0] == 4) ? "Pendingin" : (progRB1[progNum][0] == 5) ? "Thermocontrol" : "null");
      lcd.setCursor(0, 2);
      lcd.printf("%s", (progRB2[progNum][0] == 1) ? "Menyala" : (progRB2[progNum][0] == 2) ? "Mati" : "null");
    }
    else if (progTrig[progNum] == 6 || progTrig[progNum] == 7 || progTrig[progNum] == 8)
    {
      float keValue, tlrsiValue;
      memcpy(&keValue, &progRB1[progNum], sizeof(float));
      memcpy(&tlrsiValue, &progRB2[progNum], sizeof(float));
      lcd.setCursor(0, 1);
      lcd.printf("Ke: %04.1f", keValue);
      lcd.setCursor(0, 2);
      lcd.printf("Toleransi: %04.1f", tlrsiValue);
    }
    lcd.setCursor(0, 3);
    lcd.printf("Aksi:%s", (progAct[progNum] == 1) ? "Ny Out1" : (progAct[progNum] == 2) ? "Ny Out2" : (progAct[progNum] == 3) ? "Ny Pmns" : (progAct[progNum] == 4) ? "Ny Pndn" : (progAct[progNum] == 5) ? "Ny Thco" : (progAct[progNum] == 6) ? "Mt Out1" : (progAct[progNum] == 7) ? "Mt Out2" : (progAct[progNum] == 8) ? "Mt Pmns" : (progAct[progNum] == 9) ? "Mt Pndn" : (progAct[progNum] == 10) ? "Mt Thco" : "null");
    lcd.setCursor(15, 0);
    lcd.printf("PRG%02d", progNum + 1);
    lcd.setCursor(15, 3);
    lcd.print(LCD_ARROW);
    lcd.print("Back");
  }
}

void lcdUpdate()
{
  if (lcdScreen == 1)
  {
    lcd.setCursor(0, 3);
    lcd.print(LCD_ARROW);
    if (prev_newTemp != newTemp)
    {
      lcd.setCursor(5, 0);
      lcd.printf("%05.2f", newTemp);
    }
    if (prev_newHumid != newHumid)
    {
      lcd.setCursor(8, 1);
      lcd.print("      ");
      lcd.setCursor(9, 1);
      lcd.printf("%2.0f%%", newHumid);
    }
    if (prev_wifiStatus != isWifiConnected())
    {
      lcd.setCursor(13, 0);
      lcd.printf("%s", (isWifiConnected()) ? "ONLINE" : "OFFLINE");
    }
  }
  else if (lcdScreen == 2)
  {
    DateTime now = rtc.now();
    if (prev_lcdCursor != lcdCursor)
    {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(12, 0);
      lcd.print(" ");
      lcd.setCursor(12, 1);
      lcd.print(" ");
      if (lcdCursor == 0)
        lcd.setCursor(0, 0);
      else if (lcdCursor == 1)
        lcd.setCursor(0, 1);
      else if (lcdCursor == 2)
        lcd.setCursor(0, 2);
      else if (lcdCursor == 3)
        lcd.setCursor(12, 0);
      else if (lcdCursor == 4)
        lcd.setCursor(12, 1);
      lcd.print(LCD_ARROW);
    }
    if (millis() - screen2millis > 1000)
    {
      lcd.setCursor(11, 3);
      lcd.printf("%02d:%02d:%02d", now.hour(), now.minute(), now.second());
      screen2millis = millis();
    }
  }
  else if (lcdScreen == 3)
  {
    if (prev_lcdCursor != lcdCursor)
    {
      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
      lcd.setCursor(15, 0);
      lcd.print(" ");
      if (lcdCursor == 0)
        lcd.setCursor(0, 0);
      else if (lcdCursor == 1)
        lcd.setCursor(0, 1);
      else if (lcdCursor == 2)
        lcd.setCursor(0, 2);
      else if (lcdCursor == 3)
        lcd.setCursor(0, 3);
      else if (lcdCursor == 4)
        lcd.setCursor(15, 0);
      lcd.print(LCD_ARROW);
    }
    if (prev_outStatus[0] != bitRead(deviceStatus, BITPOS_AUX1_STATUS))
    {
      lcd.setCursor(10, 0);
      lcd.printf("%s ", (bitRead(deviceStatus, BITPOS_AUX1_STATUS) == MURUP) ? "ON" : "OFF");
    }
    if (prev_outStatus[1] != bitRead(deviceStatus, BITPOS_AUX2_STATUS))
    {
      lcd.setCursor(10, 1);
      lcd.printf("%s ", (bitRead(deviceStatus, BITPOS_AUX2_STATUS) == MURUP) ? "ON" : "OFF");
    }
    if (prev_outStatus[2] != bitRead(deviceStatus, BITPOS_AUX3_STATUS))
    {
      lcd.setCursor(10, 2);
      lcd.printf("%s ", (bitRead(deviceStatus, BITPOS_AUX3_STATUS) == MURUP) ? "ON" : "OFF");
    }
    if (prev_outStatus[3] != bitRead(deviceStatus, BITPOS_AUX4_STATUS))
    {
      lcd.setCursor(10, 3);
      lcd.printf("%s ", (bitRead(deviceStatus, BITPOS_AUX4_STATUS) == MURUP) ? "ON" : "OFF");
    }
  }
  else if (lcdScreen == 8)
  {
    if (prev_lcdCursor != lcdCursor)
    {
      if (prev_lcdCursor == 3 + lcdRowPos && lcdCursor == 4 + lcdRowPos && lcdCursor != lcdCursorBackPos)
      { // Scroll down
        lcdRowPos++;
        lcd.clear();
        memcpy(&lcdRowBuffer[0], &lcdRowBuffer[1], sizeof lcdRowBuffer[1]);
        memcpy(&lcdRowBuffer[1], &lcdRowBuffer[2], sizeof lcdRowBuffer[2]);
        memcpy(&lcdRowBuffer[2], &lcdRowBuffer[3], sizeof lcdRowBuffer[3]);
        sprintf(lcdRowBuffer[3], "Program %d", (uint8_t)activeProg[lcdCursor]);
        lcd.setCursor(1, 0);
        lcd.print(lcdRowBuffer[0]);
        lcd.setCursor(1, 1);
        lcd.print(lcdRowBuffer[1]);
        lcd.setCursor(1, 2);
        lcd.print(lcdRowBuffer[2]);
        lcd.setCursor(1, 3);
        lcd.print(lcdRowBuffer[3]);
        lcd.setCursor(16, 0);
        lcd.print("Back");
      }
      if (prev_lcdCursor == lcdRowPos && lcdCursor == lcdRowPos - 1 && lcdRowPos != 0)
      {
        lcdRowPos--;
        lcd.clear();
        memcpy(&lcdRowBuffer[3], &lcdRowBuffer[2], sizeof lcdRowBuffer[2]);
        memcpy(&lcdRowBuffer[2], &lcdRowBuffer[1], sizeof lcdRowBuffer[1]);
        memcpy(&lcdRowBuffer[1], &lcdRowBuffer[0], sizeof lcdRowBuffer[0]);
        sprintf(lcdRowBuffer[0], "Program %d", (uint8_t)activeProg[lcdCursor]);
        lcd.setCursor(1, 0);
        lcd.print(lcdRowBuffer[0]);
        lcd.setCursor(1, 1);
        lcd.print(lcdRowBuffer[1]);
        lcd.setCursor(1, 2);
        lcd.print(lcdRowBuffer[2]);
        lcd.setCursor(1, 3);
        lcd.print(lcdRowBuffer[3]);
        lcd.setCursor(16, 0);
        lcd.print("Back");
      }

      lcd.setCursor(0, 0);
      lcd.print(" ");
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
      lcd.setCursor(15, 0);
      lcd.print(" ");
      if (lcdCursor != lcdCursorBackPos)
        lcd.setCursor(0, lcdCursor - lcdRowPos);
      else
        lcd.setCursor(15, 0);
      lcd.print(LCD_ARROW);
    }
  }

  prev_lcdCursor = lcdCursor;
  prev_lcdRowPos = lcdRowPos;

  prev_newTemp = newTemp;
  prev_newHumid = newHumid;
  prev_wifiStatus = isWifiConnected();

  prev_outStatus[0] = bitRead(deviceStatus, BITPOS_AUX1_STATUS);
  prev_outStatus[1] = bitRead(deviceStatus, BITPOS_AUX2_STATUS);
  prev_outStatus[2] = bitRead(deviceStatus, BITPOS_AUX3_STATUS);
  prev_outStatus[3] = bitRead(deviceStatus, BITPOS_AUX4_STATUS);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

////////////// ENCODER API /////////////////
void encoderInit()
{
  encoder = new ClickEncoder(ENCODER_PINA, ENCODER_PINB, ENCODER_BTN, ENCODER_STEPS_PER_NOTCH);
  encoder->setAccelerationEnabled(true);
  enc_last = -1;
}

void encoderService()
{
  encoder->service();
}

void encoderUpdate(bool &direction, uint8_t &buttonstate, int16_t &encvalue, int16_t &delta)
{
  enc_value += encoder->getValue();
  b = encoder->getButton();
  buttonstate = b;
  encvalue = enc_value;
  delta = enc_value - enc_last;

  if (enc_value != enc_last)
  {
    if (enc_last > enc_value)
    {
      direction = ENC_DOWN;
    }
    else if (enc_last < enc_value)
    {
      direction = ENC_UP;
    }
  }
  enc_lastbutstate = b;
  enc_last = enc_value;
}
////////////////////////////////////////////

////////////////////////////////////// IO API ///////////////////////////////////////////////////////
void toggle595Bit(byte docchi)
{
  byte595Status ^= 1 << docchi;
  sr.set(docchi, bitRead595(docchi));
}

void bitWrite595(uint8_t docchi, uint8_t status)
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
    bitWrite595(SFT_MUX_A, LOW);
    bitWrite595(SFT_MUX_B, LOW);
  }
  else if (channel == 2)
  { // A2 is X1
    bitWrite595(SFT_MUX_A, HIGH);
    bitWrite595(SFT_MUX_B, LOW);
  }
  else if (channel == 3)
  { // A3 is X2
    bitWrite595(SFT_MUX_A, LOW);
    bitWrite595(SFT_MUX_B, HIGH);
  }
  else if (channel == 0)
  { // A0 is X3
    bitWrite595(SFT_MUX_A, HIGH);
    bitWrite595(SFT_MUX_B, HIGH);
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
  else if (what == USERPASS)
  {
    data = readString(618);
    storedUserpass = String((char *)data);
    free(data);
    return storedUserpass;
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
  else if (what == USERPASS)
  {
    if (strToWrite.length() < 65)
      storedUserpass = strToWrite;
    address = 618;
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
  readFromEEPROM(USERPASS);
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
  if (storedUserpass == " ")
    storedUserpass = "";
}

void loadAllPrograms()
{
  byte *data = (byte *)malloc(sizeof(byte) * (PROG_LENGTH)); // From 241 to 601
  eeprom.readBytes(ADDR_DEVICE_STATUS, PROG_LENGTH, data);

  deviceStatus = data[ADDR_DEVICE_STATUS - ADDR_DEVICE_STATUS];
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
void fetchURL(const String &URL, const String &data, int &responseCode, String &response)
{
  unsigned long dt = millis();

  // if (!client.connect(baseUri, httpPort))
  // {
  //   Serial.printf("Connecting takes %lums\n", millis() - dt);
  //   Serial.println("connection failed");
  //   responseCode = 408;
  //   response = "";
  //   failedRequestCount++;
  // }
  // else
  // {
  Serial.printf("Connecting takes %lums\n", millis() - dt);
  // configure target server and url
  http.begin(client, URL); //HTTP
  http.addHeader(F("Content-Type"), F("application/json"));
  http.addHeader(F("Device-Token"), storedDeviceToken);
  http.addHeader(F("ESP8266-BUILD-VERSION"), F(BUILD_VERSION));
  http.addHeader(F("ESP8266-SDK-VERSION"), String(ESP.getSdkVersion()));
  http.addHeader(F("ESP8266-CORE-VERSION"), ESP.getCoreVersion());
  http.addHeader(F("ESP8266-MAC"), WiFi.macAddress());
  http.addHeader(F("ESP8266-SKETCH-MD5"), sketchMD5);
  http.addHeader(F("ESP8266-SKETCH-FREE-SPACE"), freeSketch);
  http.addHeader(F("ESP8266-SKETCH-SIZE"), sketchSize);
  http.addHeader(F("ESP8266-CHIP-SIZE"), chipSize);
  dt = millis();
  // start connection and send HTTP header and body
  int httpCode = http.POST(data);
  Serial.printf("POST HTTP Takes %lums\n", millis() - dt);
  dt = millis();
  responseCode = httpCode;
  // httpCode will be negative on error
  if (httpCode > 0)
  {
    // HTTP header has been send and Server response header has been handled
    // file found at server
    if (httpCode == HTTP_CODE_OK)
    {
      successRequestCount++;
      response = http.getString();
    }
  }
  else
  {
    failedRequestCount++;
    Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
  // }
}

bool isWifiConnected()
{
  return (WiFi.status() == WL_CONNECTED);
}

bool initiateSoftAP()
{
  programStarted = false;
  lcd.clear();
  delay(100);
  lcd.setCursor(0, 0);
  lcd.print("Membuka soft AP");
  lcd.setCursor(0, 1);
  lcd.printf("%.20s", storedSoftSSID.c_str());
  lcd.setCursor(0, 2);
  lcd.print("Pada 192.168.4.1");

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

  lcd.clear();
  programStarted = true;

  statusBuzzer.off();
  statusBuzzer.on();
  delay(500);
  statusBuzzer.off();
  delay(500);
  statusBuzzer.on();
  delay(500);
  statusBuzzer.off();
  delay(500);
  statusBuzzer.off();
  return timeOutFlag;
}

bool initiateClient(const String &ssid, const String &pass)
{
  programStarted = false;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Menyambung ke WiFi");
  lcd.setCursor(0, 1);
  lcd.printf("%.20s", ssid.c_str());
  byteWrite595(0x00);
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
      statusLED.on();
      statusBuzzer.on();
      delay(250);
      statusBuzzer.off();
      statusLED.off();
      delay(250);
      statusBuzzer.off();
      statusLED.off();
    }
    if (i >= 29)
    {
      successFlag = 0;
    }
  }
  if (successFlag)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sukses tersambung");
    statusBuzzer.off();
    statusBuzzer.on();
    delay(100);
    statusBuzzer.off();
    delay(100);
    statusBuzzer.on();
    delay(100);
    statusBuzzer.off();
    delay(100);
    statusBuzzer.off();
  }
  else
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Gagal tersambung");
    statusBuzzer.off();
    statusBuzzer.on();
    delay(2000);
    statusBuzzer.off();
    delay(100);
    statusBuzzer.off();
  }
  lcd.clear();
  programStarted = true;
  return successFlag;
}

void closeSoftAP()
{
  programStarted = false;
  WiFi.softAPdisconnect(true);
  programStarted = true;
}

void closeClient()
{
  programStarted = false;
  WiFi.disconnect(true);
  programStarted = true;
}

void deployWebServer()
{
  programStarted = false;
  // Call the 'handleRoot' function when a client requests URI "/"
  server.on("/", HTTP_GET, pgRoot);

  server.on("/reqStatus", HTTP_POST, pgReqStatus);
  server.on("/accInfo", HTTP_POST, pgAccInfo);
  server.on("/restart", HTTP_POST, pgRestart);

  // When a client requests an unknown URI (i.e. something other than "/"), call function "handleNotFound"
  server.onNotFound(handleNotFound);
  server.begin();
  serverAvailable = true;
  programStarted = true;
}

void closeServer()
{
  programStarted = false;
  server.close();
  serverAvailable = false;
  programStarted = true;
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
  statusBuzzer.off();
  statusBuzzer.on();
  delay(500);
  statusBuzzer.off();
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
    programStarted = false;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Menghubungi Server..");
    lcd.setCursor(0, 1);
    lcd.print("Mohon tunggu...");

    timeClient.begin(); // If WiFi connection success, start NTP timeClient
    bool forceUpd = timeClient.forceUpdate();
    // try to fetch ntp time for maximum of 4 time before
    if (!forceUpd)
    {
      delay(500);
      forceUpd = timeClient.forceUpdate();
      if (!forceUpd)
      {
        delay(500);
        forceUpd = timeClient.forceUpdate();
        if (!forceUpd)
        {
          delay(500);
          forceUpd = timeClient.forceUpdate();
          if (!forceUpd)
          {
            delay(500);
            forceUpd = timeClient.forceUpdate();
          }
        }
      }
    }

    if (forceUpd)
    {
      Serial.println("Successfully fetching NTP Clock!");
      rtc.adjust(DateTime(timeClient.getEpochTime()));
    }
    else
    {
      Serial.println("Failed fetching NTP Clock");
      DateTime now = rtc.now();
      if (!rtc.isrunning() || now < 1577836800)
      {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("RTC Gagal");
        lcd.setCursor(0, 1);
        lcd.print("Merestart kontroller");
        Serial.println("Could'nt Initiate RTC!");
        // We're fucked up my friend, RTC is not running and can't fetch NTP time, so we'll stop right there too
        // failed to init rtc
        // add some loud ass error buzzer here
        // rtc is important element, so stop right here if something is wrong
        statusBuzzer.off();
        statusBuzzer.on();
        delay(2000);
        statusBuzzer.off();
        delay(2000);
        statusBuzzer.on();
        delay(2000);
        statusBuzzer.off();
        delay(500);
        ESP.restart();
      }
      else
      {
        Serial.printf("Initializing client x509 time RTC : %lu", rtc.now().unixtime());
      }
    }
    // This WiFi seems legit, let's save to EEPROM
    writeToEEPROM(WIFISSID, ssid);
    writeToEEPROM(WIFIPW, wfpw);

    // Initiate HTTP Request to identifyDevice.php
    Serial.print("[HTTP] begin...\n");
    int httpCode;
    StaticJsonDocument<360> doc;
    String json;
    doc[F("username")] = usrn.c_str();
    doc[F("password")] = unpw.c_str();
    doc[F("softssid")] = storedSoftSSID.c_str();
    doc[F("softpswd")] = storedSoftWifiPass.c_str();
    serializeJson(doc, json);
    Serial.printf("JSON Size : %d\n", doc.memoryUsage());
    Serial.printf("Transferred JSON : %s\n", json.c_str());
    fetchURL(FPSTR(identifyURL), json, httpCode, responseStatus);
    Serial.printf("Code : %d\nResponse : %s\n", httpCode, responseStatus.c_str());
    if (httpCode == HTTP_CODE_OK)
    { // Success fetched!, store the message to responseStatus!
      if (responseStatus == "success" || responseStatus == "recon")
      {
        writeToEEPROM(USERNAME, usrn);
        writeToEEPROM(USERPASS, unpw);
        bitWrite(storedFirstByte, FB_CONNECTED, true);
        eeprom.writeByte(ADDR_FIRST_BYTE, storedFirstByte);
        delay(10);
      }
    }
    else
    { // It seems that first request is failed, let's wait for 1s and try again for the second time
      delay(3000);
      fetchURL(FPSTR(identifyURL), json, httpCode, responseStatus);
      if (httpCode == HTTP_CODE_OK)
      { // Success fetched!, store the message to responseStatus!

        if (responseStatus == "success" || responseStatus == "recon")
        {
          writeToEEPROM(USERNAME, usrn);
          bitWrite(storedFirstByte, FB_CONNECTED, true);
          eeprom.writeByte(ADDR_FIRST_BYTE, storedFirstByte);
          delay(10);
        }
      }
      else // It failed once again, probably the WiFi is offline or server is offline, let's report
        responseStatus = "nocon";
    }
  }
  else // Cannot connect to WiFi, report invalid SSID or Password!
    responseStatus = "invwifi";
  // Reinitiate softAP and re deploy the web server to 192.168.4.1

  programStarted = false;
  if (responseStatus == "success" || responseStatus == "recon")
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Berhasil terhubung");
    lcd.setCursor(0, 1);
    lcd.print("Ke server");
    lcd.setCursor(0, 2);
    lcd.print("Mohon restart");
    lcd.setCursor(0, 3);
    lcd.print("kontroller");
    statusBuzzer.on();
    delay(500);
    statusBuzzer.off();
    delay(100);
    statusBuzzer.on();
    delay(500);
    statusBuzzer.off();
  }
  else
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Terjadi kesalahan");
    lcd.setCursor(0, 1);
    lcd.print("Cek 192.168.4.1 lagi");
    lcd.setCursor(0, 2);
    lcd.print("Untuk info detailnya");
    statusBuzzer.on();
    delay(1000);
    statusBuzzer.off();
    delay(100);
    statusBuzzer.on();
    delay(1000);
    statusBuzzer.off();
  }
  delay(5000);
  closeClient();
  closeServer();
  closeSoftAP();
  initiateSoftAP();
  deployWebServer();

  lcdTransition(1);
  programStarted = true;
}

void pgReqStatus()
{
  StaticJsonDocument<360> doc;
  String json;
  doc[F("usrn")] = storedUsername.c_str();
  doc[F("unpw")] = storedUserpass.c_str();
  doc[F("ssid")] = storedSSID.c_str();
  doc[F("wfpw")] = storedWifiPass.c_str();
  doc[F("message")] = responseStatus.c_str();
  serializeJson(doc, json);
  Serial.printf("JSON Size : %d\n", doc.memoryUsage());
  Serial.printf("Transferring JSON : %s\n", json.c_str());
  server.send(200, "application/json", json);
}

void handleNotFound()
{
  server.send(404, "text/plain", F("404: Not found"));
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
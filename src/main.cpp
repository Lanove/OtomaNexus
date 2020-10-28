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
#include <WiFiClientSecure.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <time.h>
#include <constants.h>
#include <PID_v1.h>
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
uint8_t uselessNumberParser(uint8_t progAct);
void programScan(void);
void encoderInit();
void encoderUpdate(bool &direction, uint8_t &buttonstate, int16_t &encvalue, int16_t &delta);
void encoderService();
void lcdSetup();
void lcdTransition(int screen, int progNum = 0);
void lcdUpdate();

HTTPClient http;
BearSSL::WiFiClientSecure *client = new BearSSL::WiFiClientSecure();
BearSSL::X509List cert;
BearSSL::Session session;
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

byte dhtSampleCounter = DHT_LOOP,
     byte595Status,
     deviceStatus,
     htclMode;
bool disconnectFlag,
    bit595Status;

float newTemp,
    newHumid,

    thermalSetPoint,

    heaterKp,
    heaterKi,
    heaterKd,
    heaterDs,
    heaterPidOutput;
unsigned long heaterWindowStart;
PID heaterPID(&newTemp, &heaterPidOutput, &thermalSetPoint, heaterKp, heaterKi, heaterKd, DIRECT);

bool heaterHysteresis;
float heaterBa,
    heaterBb,

    coolerKp,
    coolerKi,
    coolerKd,
    coolerDs,
    coolerPidOutput;
unsigned long coolerWindowStart;
PID coolerPID(&newTemp, &coolerPidOutput, &thermalSetPoint, coolerKp, coolerKi, coolerKd, REVERSE);

float coolerBa,
    coolerBb;
bool coolerHysteresis;

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
    prev_newHumid,
    prev_thermalSetPoint;
bool prev_wifiStatus;

// Page 2 dynamics
// 0 is date, 1 is month, 2 is hour, 3 is minute, 4 is second
// byte prev_jdw[5];
// int prev_year;

// Page 3 dynamics
// 0 is pemanas, 1 is pendingin, 2 is out1, 3 is out1, 4 is out2
bool prev_outStatus[4];

// Page 4 dynamics
// 0 is tc status, 1 is tc mode
bool prev_tcStatus,
    prev_tcOperation;
byte prev_tcMode;

// Page 5 dynamics
bool prev_htMode;
//BA, BB, KP ,KI ,KD ,DS respectively
float prev_heaterKp,
    prev_heaterKi,
    prev_heaterKd,
    prev_heaterDs,
    prev_heaterBa,
    prev_heaterBb;

// Page 6 dynamics
bool prev_clMode;
//BA, BB, KP ,KI ,KD ,DS respectively
float prev_coolerKp,
    prev_coolerKi,
    prev_coolerKd,
    prev_coolerDs,
    prev_coolerBa,
    prev_coolerBb;

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

const String freeSketch = String(ESP.getFreeSketchSpace()),
             sketchSize = String(ESP.getSketchSize()),
             chipSize = String(ESP.getFlashChipSize()),
             sketchMD5 = ESP.getSketchMD5();

void setup(void)
{
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
  encoderInit();
  dht.begin();
  loadInfo();
  //75753us
  //512006us

  if (cert.append(caCert, caCertLen))
    Serial.println("Successfully loading certificate");
  else
    Serial.println("Failed loading certificate!");
  client->setTrustAnchors(&cert);
  client->setSession(&session);
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
  newHumid = dht.readHumidity();

  heaterPID.SetTunings(heaterKp, heaterKi, heaterKd);
  coolerPID.SetTunings(coolerKp, coolerKi, coolerKd);

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
      client->setX509Time(timeClient.getEpochTime());
    }
    else
    {
      Serial.println("Failed fetching NTP Clock");
      DateTime now = rtc.now();
      if (!rtc.isrunning() || now < 1577836800)
      {
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
        client->setX509Time(rtc.now().unixtime());
        Serial.printf("Initializing client x509 time RTC : %lu", rtc.now().unixtime());
      }
    }
    programStarted = false;
    t_httpUpdate_return ret = ESPhttpUpdate.update(dynamic_cast<WiFiClient &>(*client), FPSTR(espUpdater), BUILD_VERSION);
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
  Serial.println("loop");
  float *sensorBuffer = (float *)malloc(sizeof(float) * 2);
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
  if (dhtSampleCounter <= 0)
  {
    delay(5);
    sensorBuffer[1] = dht.readHumidity();
    if (!isnan(sensorBuffer[1]))
      newHumid = sensorBuffer[1];
    dhtSampleCounter = DHT_LOOP;
  }
  if (newTemp <= 0.0 || newTemp > 200.0)
  {
    newTemp = 0.0;
    ds18bFaultCounter++;
  }
  if (newHumid < 2.0 || newHumid > 100.0)
    newHumid = 0.0;
  sensorMillis = millis();
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
        doc[F("tc")] = bitRead(deviceStatus, BITPOS_TC_STATUS);
        doc[F("ht")] = bitRead(deviceStatus, BITPOS_HEATER_STATUS);
        doc[F("cl")] = bitRead(deviceStatus, BITPOS_COOLER_STATUS);

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
                if (out["setpoint"])
                {
                  float ibuffer = out["setpoint"].as<float>();
                  thermalSetPoint = ibuffer;
                  Serial.printf("Received Setpoint Update!\nthermalSetPoint : %f\n", thermalSetPoint);
                  memcpy(buffer, &ibuffer, 4);
                  eeprom.writeBytes(ADDR_THERMAL_SETPOINT, 4, buffer);
                }
                if (out["hpam"])
                {
                  float fbuffer = out["hpam"][0].as<float>(); // Kp
                  heaterKp = fbuffer;
                  memcpy(buffer, &fbuffer, 4);
                  eeprom.writeBytes(ADDR_HEATER_KP, 4, buffer);
                  fbuffer = out["hpam"][1].as<float>(); // Ki
                  heaterKi = fbuffer;
                  memcpy(buffer, &fbuffer, 4);
                  eeprom.writeBytes(ADDR_HEATER_KI, 4, buffer);
                  fbuffer = out["hpam"][2].as<float>(); // Kd
                  heaterKd = fbuffer;
                  memcpy(buffer, &fbuffer, 4);
                  eeprom.writeBytes(ADDR_HEATER_KD, 4, buffer);
                  fbuffer = out["hpam"][3].as<float>(); // Ds
                  heaterDs = fbuffer;
                  memcpy(buffer, &fbuffer, 4);
                  eeprom.writeBytes(ADDR_HEATER_DS, 4, buffer);
                  fbuffer = out["hpam"][4].as<float>(); // Ba
                  heaterBa = fbuffer;
                  memcpy(buffer, &fbuffer, 4);
                  eeprom.writeBytes(ADDR_HEATER_BA, 4, buffer);
                  fbuffer = out["hpam"][5].as<float>(); // Bb
                  heaterBb = fbuffer;
                  memcpy(buffer, &fbuffer, 4);
                  eeprom.writeBytes(ADDR_HEATER_BB, 4, buffer);
                  Serial.println(out["hmd"].as<bool>());
                  bitWrite(htclMode, BITPOS_HEATER_MODE, out["hmd"].as<bool>());
                  eeprom.writeByte(ADDR_HTCL_MODE, htclMode);
                  delay(10);
                  heaterPID.SetTunings(heaterKp, heaterKi, heaterKd);
                  Serial.printf("Received Heater Param Update!\nhHeater Mode : %s\nheaterKp : %f\nheaterKi : %f\nheaterKd : %f\nheaterDs : %f\nheaterBa : %f\nheaterBb : %f\n", (bitRead(htclMode, BITPOS_HEATER_MODE)) ? "Hysteresis" : "PID", heaterKp, heaterKi, heaterKd, heaterDs, heaterBa, heaterBb);
                }
                if (out["cpam"])
                {
                  float fbuffer = out["cpam"][0].as<float>(); // Kp
                  coolerKp = fbuffer;
                  memcpy(buffer, &fbuffer, 4);
                  eeprom.writeBytes(ADDR_COOLER_KP, 4, buffer);
                  fbuffer = out["cpam"][1].as<float>(); // Ki
                  coolerKi = fbuffer;
                  memcpy(buffer, &fbuffer, 4);
                  eeprom.writeBytes(ADDR_COOLER_KI, 4, buffer);
                  fbuffer = out["cpam"][2].as<float>(); // Kd
                  coolerKd = fbuffer;
                  memcpy(buffer, &fbuffer, 4);
                  eeprom.writeBytes(ADDR_COOLER_KD, 4, buffer);
                  fbuffer = out["cpam"][3].as<float>(); // Ds
                  coolerDs = fbuffer;
                  memcpy(buffer, &fbuffer, 4);
                  eeprom.writeBytes(ADDR_COOLER_DS, 4, buffer);
                  fbuffer = out["cpam"][4].as<float>(); // Ba
                  coolerBa = fbuffer;
                  memcpy(buffer, &fbuffer, 4);
                  eeprom.writeBytes(ADDR_COOLER_BA, 4, buffer);
                  fbuffer = out["cpam"][5].as<float>(); // Bb
                  coolerBb = fbuffer;
                  memcpy(buffer, &fbuffer, 4);
                  eeprom.writeBytes(ADDR_COOLER_BB, 4, buffer);
                  Serial.println(out["cmd"].as<bool>());
                  bitWrite(htclMode, BITPOS_COOLER_MODE, out["cmd"].as<bool>());
                  eeprom.writeByte(ADDR_HTCL_MODE, htclMode);
                  coolerPID.SetTunings(coolerKp, coolerKi, coolerKd);
                  delay(10);
                  // writeByte mode
                  Serial.printf("Received Cooler Param Update!\nCooler Mode : %s\ncoolerKp : %f\ncoolerKi : %f\ncoolerKd : %f\ncoolerDs : %f\ncoolerBa : %f\ncoolerBb : %f\n", (bitRead(htclMode, BITPOS_COOLER_MODE)) ? "Hysteresis" : "PID", coolerKp, coolerKi, coolerKd, coolerDs, coolerBa, coolerBb);
                }
                if (out["st"])
                {
                  if (!out["auxStatus1"].isNull())
                    bitWrite(deviceStatus, BITPOS_AUX1_STATUS, out["auxStatus1"].as<bool>());
                  if (!out["auxStatus2"].isNull())
                    bitWrite(deviceStatus, BITPOS_AUX2_STATUS, out["auxStatus2"].as<bool>());
                  if (!out["thStatus"].isNull())
                  {
                    bitWrite(deviceStatus, BITPOS_TC_STATUS, out["thStatus"].as<bool>());
                    eeprom.writeByte(ADDR_DEVICE_STATUS, deviceStatus);
                  }
                  if (!out["htStatus"].isNull())
                    bitWrite(deviceStatus, BITPOS_HEATER_STATUS, out["htStatus"].as<bool>());
                  if (!out["clStatus"].isNull())
                    bitWrite(deviceStatus, BITPOS_COOLER_STATUS, out["clStatus"].as<bool>());
                  delay(10);
                  Serial.printf("Received Status Update\nAux Status 1 : %d\nAux Status 2 : %d\nThermocontrol Status : %d\nHeater Status : %d\nCooler Status : %d\n", bitRead(deviceStatus, BITPOS_AUX1_STATUS), bitRead(deviceStatus, BITPOS_AUX2_STATUS), bitRead(deviceStatus, BITPOS_TC_STATUS), bitRead(deviceStatus, BITPOS_HEATER_STATUS), bitRead(deviceStatus, BITPOS_COOLER_STATUS));
                }
                if (out["tcm"])
                {
                  //
                  bitWrite(deviceStatus, BITPOS_TC_OPERATION, out["tcm"][0].as<bool>());
                  bitWrite(deviceStatus, BITPOS_TC_MODE_B0, out["tcm"][1].as<bool>());
                  bitWrite(deviceStatus, BITPOS_TC_MODE_B1, out["tcm"][2].as<bool>());
                  eeprom.writeByte(ADDR_DEVICE_STATUS, deviceStatus);
                  delay(10);
                  Serial.printf("Received Thermalcontroller Update\nTherco Operation : %s\nTherco Mode %s\n", (bitRead(deviceStatus, BITPOS_TC_OPERATION)) ? "AUTO" : "MANUAL", (bitRead(deviceStatus, BITPOS_TC_MODE_B0) == 0 && bitRead(deviceStatus, BITPOS_TC_MODE_B1) == 0) ? "HEATER" : (bitRead(deviceStatus, BITPOS_TC_MODE_B0) == 1 && bitRead(deviceStatus, BITPOS_TC_MODE_B1) == 0) ? "COOLER" : (bitRead(deviceStatus, BITPOS_TC_MODE_B0) == 1 && bitRead(deviceStatus, BITPOS_TC_MODE_B1) == 1) ? "DUAL" : "INVALID");
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
        Serial.printf("Main online routine takes : %dms\nCONT stack used at main fetch url: %d\n-------\n\n", millis() - timeTakenStart, freeStackStart - freeStackEnd);
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
  if (millis() - updateCheckMillis >= UPDATE_CHECK_INTERVAL)
  {
    Serial.println("Checking update");
    programStarted = false;
    byteWrite595(0x00);
    t_httpUpdate_return ret = ESPhttpUpdate.update(dynamic_cast<WiFiClient &>(*client), FPSTR(espUpdater), BUILD_VERSION);
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

void debugMemory()
{
  Serial.printf("Stack : %d\nHeap : %d\n", ESP.getFreeContStack(), ESP.getFreeHeap());
}

uint8_t uselessNumberParser(uint8_t progAct)
{
  switch (progAct)
  {
  case 1:
    return 2;
    break;
  case 2:
    return 3;
    break;
  case 3:
    return 0;
    break;
  case 4:
    return 1;
    break;
  case 5:
    return 4;
    break;
  case 6:
    return 2;
    break;
  case 7:
    return 3;
    break;
  case 8:
    return 0;
    break;
  case 9:
    return 1;
    break;
  case 10:
    return 4;
    break;
  }
  return 0;
}

void programScan(void)
{
  if (programStarted)
  {
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
      else if (lcdScreen == 2 || lcdScreen == 3)
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

    if (bs == ENC_DBCLICKED)
    { // Navigation
      if (lcdScreen == 1)
        lcdTransition(2);
      else if (lcdScreen == 2)
      {
        if (lcdCursor == 0)
          lcdTransition(3);
        else if (lcdCursor == 1)
          lcdTransition(4);
        else if (lcdCursor == 2)
          lcdTransition(8);
        else if (lcdCursor == 3)
          lcdTransition(7);
        else if (lcdCursor == 4)
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

    statusLED.update();
    statusBuzzer.update();
    bool statusBuffer[10];
    // 0 is heater, 1 is cooler, 2 is aux1, 3 is aux2
    // The algorithm that set PID Mode is just straight, the ifs is not there
    statusBuffer[0] = bitRead(deviceStatus, BITPOS_HEATER_STATUS);
    statusBuffer[1] = bitRead(deviceStatus, BITPOS_COOLER_STATUS);
    statusBuffer[2] = bitRead(deviceStatus, BITPOS_AUX1_STATUS);
    statusBuffer[3] = bitRead(deviceStatus, BITPOS_AUX2_STATUS);
    statusBuffer[4] = bitRead(deviceStatus, BITPOS_TC_STATUS);
    bool hcon =
        ((bitRead(deviceStatus, BITPOS_TC_MODE_B0) == 0 && bitRead(deviceStatus, BITPOS_TC_MODE_B1) == 0) ||
         (bitRead(deviceStatus, BITPOS_TC_MODE_B0) == 1 && bitRead(deviceStatus, BITPOS_TC_MODE_B1) == 1));
    bool ccon =
        ((bitRead(deviceStatus, BITPOS_TC_MODE_B0) == 1 && bitRead(deviceStatus, BITPOS_TC_MODE_B1) == 0) ||
         (bitRead(deviceStatus, BITPOS_TC_MODE_B0) == 1 && bitRead(deviceStatus, BITPOS_TC_MODE_B1) == 1));
    bool tcactive =
        (bitRead(deviceStatus, BITPOS_TC_OPERATION) == MODE_OPERATION_AUTO &&
         bitRead(deviceStatus, BITPOS_TC_STATUS) == true);

    if (tcactive && bitRead(htclMode, BITPOS_HEATER_MODE) == MODE_PID && hcon)
    {
      if (heaterPID.GetMode() == MANUAL)
        heaterPID.SetMode(AUTOMATIC);
      heaterPID.Compute();
      if (millis() - heaterWindowStart > (unsigned long)heaterDs)
      {
        //time to shift the Relay Window
        heaterWindowStart += (unsigned long)heaterDs;
      }

      float copyOutput = heaterPidOutput;
      if (copyOutput <= 0)
        copyOutput = 0.01;
      if ((unsigned long)((copyOutput * (heaterDs / 255.0))) < millis() - heaterWindowStart)
        statusBuffer[0] = MATI;
      else
        statusBuffer[0] = MURUP;
    }
    else if (tcactive && bitRead(htclMode, BITPOS_HEATER_MODE) == MODE_HYSTERESIS && hcon)
    { // PID MANUAL DOESNT IMPLY THAT HEATER IS HYSTERESIS, WE STILL NEED TO CHECK IF IT WAS ENABLED OR NOT (TC MODE, OPERATION ETC)
      if (heaterPID.GetMode() == AUTOMATIC)
        heaterPID.SetMode(MANUAL);
      if (newTemp >= thermalSetPoint + heaterBa)
        heaterHysteresis = MATI;
      else if (newTemp <= thermalSetPoint - heaterBb)
        heaterHysteresis = MURUP;
      statusBuffer[0] = heaterHysteresis;
    }

    if (bitRead(htclMode, BITPOS_COOLER_MODE) == MODE_PID && tcactive && ccon)
    {
      if (coolerPID.GetMode() == MANUAL)
        coolerPID.SetMode(AUTOMATIC);
      coolerPID.Compute();
      if (millis() - coolerWindowStart > (unsigned long)coolerDs)
      {
        //time to shift the Relay Window
        coolerWindowStart += (unsigned long)coolerDs;
      }
      float copyOutput = coolerPidOutput;
      if (copyOutput <= 0)
        copyOutput = 0.01;
      if ((unsigned long)(coolerPidOutput * (coolerDs / 255.0)) < millis() - coolerWindowStart)
        statusBuffer[1] = MATI;
      else
        statusBuffer[1] = MURUP;
    }
    else if (bitRead(htclMode, BITPOS_COOLER_MODE) == MODE_HYSTERESIS && tcactive && ccon)
    { // PID MANUAL DOESNT IMPLY THAT HEATER IS HYSTERESIS, WE STILL NEED TO CHECK IF IT WAS ENABLED OR NOT (TC MODE, OPERATION ETC)
      if (coolerPID.GetMode() == AUTOMATIC)
        coolerPID.SetMode(MANUAL);
      if (newTemp >= thermalSetPoint + coolerBa)
        coolerHysteresis = MURUP;
      else if (newTemp <= thermalSetPoint - coolerBb)
        coolerHysteresis = MATI;
      statusBuffer[1] = coolerHysteresis;
    }

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
            if (progAct[i] == 1 || progAct[i] == 6)
              statusBuffer[2] = (progAct[i] == 1) ? MURUP : MATI;
            if (progAct[i] == 2 || progAct[i] == 7)
              statusBuffer[3] = (progAct[i] == 2) ? MURUP : MATI;
            if (progAct[i] == 3 || progAct[i] == 8)
              statusBuffer[0] = (progAct[i] == 3) ? MURUP : MATI;
            if (progAct[i] == 4 || progAct[i] == 9)
              statusBuffer[1] = (progAct[i] == 4) ? MURUP : MATI;
            if (progAct[i] == 5 || progAct[i] == 10)
              statusBuffer[4] = (progAct[i] == 5) ? MURUP : MATI;
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
                if (progAct[i] < 6)
                  statusBuffer[uselessNumberParser(progAct[i])] = MURUP;
                else if (progAct[i] >= 6)
                  statusBuffer[uselessNumberParser(progAct[i])] = MATI;
                progFlag[i] = true;
              }
              else if (progFlag[i])
              {
                if (progAct[i] < 6)
                  statusBuffer[uselessNumberParser(progAct[i])] = MATI;
                else if (progAct[i] >= 6)
                  statusBuffer[uselessNumberParser(progAct[i])] = MURUP;
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
              if (progAct[i] < 6)
                statusBuffer[uselessNumberParser(progAct[i])] = MURUP;
              else if (progAct[i] >= 6)
                statusBuffer[uselessNumberParser(progAct[i])] = MATI;
              progFlag[i] = true;
            }
            else if (progFlag[i])
            {
              if (progAct[i] < 6)
                statusBuffer[uselessNumberParser(progAct[i])] = MATI;
              else if (progAct[i] >= 6)
                statusBuffer[uselessNumberParser(progAct[i])] = MURUP;
              progFlag[i] = false;
            }
          }
        }
        else if (progTrig[i] == 5 && progRB2[i][0] != 0)
        {
          if (progRB2[i][0] != 0)
          {
            if (progRB1[i][0] == 1)
              condition = (bitRead(deviceStatus, BITPOS_AUX1_STATUS) == ((progRB2[i][0] == 1) ? MURUP : MATI));
            else if (progRB1[i][0] == 2)
              condition = (bitRead(deviceStatus, BITPOS_AUX2_STATUS) == ((progRB2[i][0] == 1) ? MURUP : MATI));
            else if (progRB1[i][0] == 3)
              condition = (bitRead(deviceStatus, BITPOS_HEATER_STATUS) == ((progRB2[i][0] == 1) ? MURUP : MATI));
            else if (progRB1[i][0] == 4)
              condition = (bitRead(deviceStatus, BITPOS_COOLER_STATUS) == ((progRB2[i][0] == 1) ? MURUP : MATI));
            else if (progRB1[i][0] == 5)
              condition = (bitRead(deviceStatus, BITPOS_TC_STATUS) == ((progRB2[i][0] == 1) ? MURUP : MATI));
            else
              condition = false;
            if (condition)
            {
              if (progAct[i] == 1 || progAct[i] == 6)
                statusBuffer[2] = (progAct[i] == 1) ? MURUP : MATI;
              if (progAct[i] == 2 || progAct[i] == 7)
                statusBuffer[3] = (progAct[i] == 2) ? MURUP : MATI;
              if (progAct[i] == 3 || progAct[i] == 8)
                statusBuffer[0] = (progAct[i] == 3) ? MURUP : MATI;
              if (progAct[i] == 4 || progAct[i] == 9)
                statusBuffer[1] = (progAct[i] == 4) ? MURUP : MATI;
              if (progAct[i] == 5 || progAct[i] == 10)
                statusBuffer[4] = (progAct[i] == 5) ? MURUP : MATI;
            }
          }
        }
      }
    }

    if (!bitRead(deviceStatus, BITPOS_TC_STATUS))
    {
      statusBuffer[0] = MATI;
      statusBuffer[1] = MATI;
    }
    bitWrite595(SFT_HEATER_RELAY, statusBuffer[0]);
    bitWrite595(SFT_COOLER_RELAY, statusBuffer[1]);
    bitWrite595(SFT_AUX1_RELAY, statusBuffer[2]);
    bitWrite595(SFT_AUX2_RELAY, statusBuffer[3]);
    bitWrite(deviceStatus, BITPOS_HEATER_STATUS, statusBuffer[0]);
    bitWrite(deviceStatus, BITPOS_COOLER_STATUS, statusBuffer[1]);
    bitWrite(deviceStatus, BITPOS_AUX1_STATUS, statusBuffer[2]);
    bitWrite(deviceStatus, BITPOS_AUX2_STATUS, statusBuffer[3]);
    bitWrite(deviceStatus, BITPOS_TC_STATUS, statusBuffer[4]);
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
    lcd.setCursor(1, 0);
    lcd.printf("Suhu:%05.2f", newTemp);
    lcd.print(LCD_DEGREE); // print [degree] character
    lcd.print("C");
    lcd.setCursor(14, 0);
    lcd.printf("%s", (isWifiConnected()) ? "ONL" : "OFFL");
    lcd.setCursor(1, 1);
    lcd.printf("Hmdt:%05.1f%%", newHumid);
    lcd.setCursor(1, 2);
    lcd.printf("SP  : %04.1f", thermalSetPoint);
    lcd.print(LCD_DEGREE); // print [degree] character
    lcd.print("C");
    lcd.setCursor(0, 3);
    lcd.print(LCD_ARROW);
    lcd.print("Menu");
  }
  else if (screen == 2)
  {
    DateTime now = rtc.now();
    lcd.setCursor(0, 0);
    lcd.print(LCD_ARROW);
    lcd.printf("Output         Back");
    lcd.setCursor(1, 1);
    lcd.printf("Therco");
    lcd.setCursor(1, 2);
    lcd.printf("Program  %02d/%02d/%04d", now.day(), now.month(), now.year());
    lcd.setCursor(1, 3);
    lcd.printf("Info      %02d:%02d:%02d", now.hour(), now.minute(), now.second());
  }
  else if (screen == 3)
  {
    lcd.setCursor(0, 0);
    lcd.print(LCD_ARROW);
    lcd.printf("Pemanas:%s", (bitRead(deviceStatus, BITPOS_HEATER_STATUS) == MURUP) ? "ON" : "OFF");
    lcd.setCursor(16, 0);
    lcd.print("Back");
    lcd.setCursor(1, 1);
    lcd.printf("Pendngin:%s", (bitRead(deviceStatus, BITPOS_COOLER_STATUS) == MURUP) ? "ON" : "OFF");
    lcd.setCursor(1, 2);
    lcd.printf("Output 1:%s", (bitRead(deviceStatus, BITPOS_AUX1_STATUS) == MURUP) ? "ON" : "OFF");
    lcd.setCursor(1, 3);
    lcd.printf("Output 2:%s", (bitRead(deviceStatus, BITPOS_AUX2_STATUS) == MURUP) ? "ON" : "OFF");
  }
  else if (screen == 4)
  {
    lcd.setCursor(0, 0);
    lcd.print(LCD_ARROW);
    lcd.printf("Status :%s", (bitRead(deviceStatus, BITPOS_TC_STATUS) == MURUP) ? "ON" : "OFF");
    sprintf(lcdRowBuffer[0], "Status :%s", (bitRead(deviceStatus, BITPOS_TC_STATUS) == MURUP) ? "ON" : "OFF");
    lcd.setCursor(1, 1);
    lcd.printf("Mode   :%s",
               (bitRead(deviceStatus, BITPOS_TC_MODE_B0) == 1 && bitRead(deviceStatus, BITPOS_TC_MODE_B1) == 1) ? "Dual" : (bitRead(deviceStatus, BITPOS_TC_MODE_B0) == 0 && bitRead(deviceStatus, BITPOS_TC_MODE_B1) == 1) ? "Cool" : (bitRead(deviceStatus, BITPOS_TC_MODE_B0) == 0 && bitRead(deviceStatus, BITPOS_TC_MODE_B1) == 0) ? "Heat" : "");
    sprintf(lcdRowBuffer[1], "Mode   :%s",
            (bitRead(deviceStatus, BITPOS_TC_MODE_B0) == 1 && bitRead(deviceStatus, BITPOS_TC_MODE_B1) == 1) ? "Dual" : (bitRead(deviceStatus, BITPOS_TC_MODE_B0) == 0 && bitRead(deviceStatus, BITPOS_TC_MODE_B1) == 1) ? "Cool" : (bitRead(deviceStatus, BITPOS_TC_MODE_B0) == 0 && bitRead(deviceStatus, BITPOS_TC_MODE_B1) == 0) ? "Heat" : "");
    lcd.setCursor(1, 2);
    lcd.printf("Operasi:%s", (bitRead(deviceStatus, BITPOS_TC_OPERATION) == MODE_OPERATION_AUTO) ? "Auto" : "Manual");
    sprintf(lcdRowBuffer[2], "Operasi:%s", (bitRead(deviceStatus, BITPOS_TC_OPERATION) == MODE_OPERATION_AUTO) ? "Auto" : "Manual");
    lcd.setCursor(1, 3);
    lcd.print("Opsi Pemanas");
    sprintf(lcdRowBuffer[3], "Opsi Pemanas");
    lcd.setCursor(16, 0);
    lcd.print("Back");
  }
  else if (screen == 5 || screen == 6)
  {
    lcdCursorBackPos = 7;
    lcd.setCursor(0, 0);
    lcd.print((screen == 5) ? "Pemanas         Back" : "Pendingin       Back");
    sprintf(lcdRowBuffer[0], (screen == 5) ? "Pemanas         Back" : "Pendingin       Back");
    lcd.setCursor(0, 1);
    lcd.print(LCD_ARROW);
    lcd.printf("Mode:%s", (bitRead(htclMode, (screen == 5) ? BITPOS_HEATER_MODE : BITPOS_COOLER_MODE) == MODE_PID) ? "PID" : "HYS");
    sprintf(lcdRowBuffer[1], "Mode:%s", (bitRead(htclMode, (screen == 5) ? BITPOS_HEATER_MODE : BITPOS_COOLER_MODE) == MODE_PID) ? "PID" : "HYS");
    lcd.setCursor(1, 2);
    lcd.printf("Bts Ats:%05.2f", (screen == 5) ? heaterBa : coolerBa);
    sprintf(lcdRowBuffer[2], "Bts Ats:%05.2f", (screen == 5) ? heaterBa : coolerBa);
    lcd.setCursor(1, 3);
    lcd.printf("Bts Bwh:%05.2f", (screen == 5) ? heaterBb : coolerBb);
    sprintf(lcdRowBuffer[3], "Bts Bwh:%05.2f", (screen == 5) ? heaterBb : coolerBb);
  }
  else if (screen == 7)
  {
    lcd.setCursor(0, 0);
    lcd.printf("Soft AP:%.12s", storedSoftSSID.c_str());
    lcd.setCursor(0, 1);
    lcd.printf("Soft PW:%.12s", storedSoftWifiPass.c_str());
    lcd.setCursor(0, 2);
    lcd.print("IP:192.168.4.1"); // this guy is the same every esp, so its fine to hardcode
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
    lcd.printf("Pemicu:%s", (progTrig[progNum] == 1) ? "Suhu" : (progTrig[progNum] == 2) ? "Humidts" : (progTrig[progNum] == 3) ? "Jdwlhrn" : (progTrig[progNum] == 4) ? "TglWktu" : (progTrig[progNum] == 5) ? "Keadaan" : "null");
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
      lcd.setCursor(6, 0);
      lcd.printf("%05.2f", newTemp);
    }
    if (prev_newHumid != newHumid)
    {
      lcd.setCursor(6, 1);
      lcd.printf("%05.1f", newHumid);
    }
    if (prev_thermalSetPoint != thermalSetPoint)
    {
      lcd.setCursor(7, 2);
      lcd.printf("%05.1f", thermalSetPoint);
    }
    if (prev_wifiStatus != isWifiConnected())
    {
      lcd.setCursor(14, 0);
      lcd.printf("%s", (isWifiConnected()) ? "Online" : "Offline");
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
    if (prev_outStatus[0] != bitRead(deviceStatus, BITPOS_HEATER_STATUS))
    {
      lcd.setCursor(9, 0);
      lcd.printf("%s", (bitRead(deviceStatus, BITPOS_HEATER_STATUS) == MURUP) ? "ON" : "OFF");
    }
    if (prev_outStatus[1] != bitRead(deviceStatus, BITPOS_COOLER_STATUS))
    {
      lcd.setCursor(9, 1);
      lcd.printf("%s", (bitRead(deviceStatus, BITPOS_COOLER_STATUS) == MURUP) ? "ON" : "OFF");
    }
    if (prev_outStatus[2] != bitRead(deviceStatus, BITPOS_AUX1_STATUS))
    {
      lcd.setCursor(9, 2);
      lcd.printf("%s", (bitRead(deviceStatus, BITPOS_AUX1_STATUS) == MURUP) ? "ON" : "OFF");
    }
    if (prev_outStatus[3] != bitRead(deviceStatus, BITPOS_AUX2_STATUS))
    {
      lcd.setCursor(9, 3);
      lcd.printf("%s", (bitRead(deviceStatus, BITPOS_AUX2_STATUS) == MURUP) ? "ON" : "OFF");
    }
  }
  else if (lcdScreen == 4)
  {
    byte modeBuffer;
    bitWrite(modeBuffer, 0, bitRead(deviceStatus, BITPOS_TC_MODE_B0));
    bitWrite(modeBuffer, 1, bitRead(deviceStatus, BITPOS_TC_MODE_B1));
    if (prev_tcStatus != bitRead(deviceStatus, BITPOS_TC_STATUS))
    {
      lcd.setCursor(9, 0);
      lcd.print("   ");
      lcd.setCursor(9, 0);
      lcd.printf("%s", (bitRead(deviceStatus, BITPOS_TC_STATUS) == MURUP) ? "ON" : "OFF");
      if (lcdCursor < 4)
        sprintf(lcdRowBuffer[0], "Status :%s", (bitRead(deviceStatus, BITPOS_TC_STATUS) == MURUP) ? "ON" : "OFF");
    }
    if (prev_tcMode != modeBuffer)
    {
      lcd.setCursor(9, 1);
      lcd.printf("%s", (modeBuffer == 3) ? "Dual" : (modeBuffer == 1) ? "Cool" : (modeBuffer == 0) ? "Heat" : "");
      sprintf(lcdRowBuffer[1 - lcdRowPos], "Mode   :%s", (modeBuffer == 3) ? "Dual" : (modeBuffer == 1) ? "Cool" : (modeBuffer == 0) ? "Heat" : "");
    }
    if (prev_tcOperation != bitRead(deviceStatus, BITPOS_TC_OPERATION))
    {
      lcd.setCursor(9, 2);
      lcd.printf("%s", (bitRead(deviceStatus, BITPOS_TC_OPERATION) == MODE_OPERATION_AUTO) ? "Auto" : "Manual");
      sprintf(lcdRowBuffer[2 - lcdRowPos], "Operasi:%s", (bitRead(deviceStatus, BITPOS_TC_OPERATION) == MODE_OPERATION_AUTO) ? "Auto" : "Manual");
    }
    if (prev_lcdCursor != lcdCursor)
    {
      if (prev_lcdCursor == 3 && lcdCursor == 4 && lcdRowPos == 0)
      {
        lcdRowPos = 1;
        lcd.clear();
        memcpy(&lcdRowBuffer[0], &lcdRowBuffer[1], sizeof lcdRowBuffer[1]);
        memcpy(&lcdRowBuffer[1], &lcdRowBuffer[2], sizeof lcdRowBuffer[2]);
        memcpy(&lcdRowBuffer[2], &lcdRowBuffer[3], sizeof lcdRowBuffer[3]);
        sprintf(lcdRowBuffer[3], "Opsi Pendingin");
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
      else if (prev_lcdCursor == 1 && lcdCursor == 0 && lcdRowPos == 1)
      {
        lcdRowPos = 0;
        lcd.clear();
        memcpy(&lcdRowBuffer[3], &lcdRowBuffer[2], sizeof lcdRowBuffer[2]);
        memcpy(&lcdRowBuffer[2], &lcdRowBuffer[1], sizeof lcdRowBuffer[1]);
        memcpy(&lcdRowBuffer[1], &lcdRowBuffer[0], sizeof lcdRowBuffer[0]);
        sprintf(lcdRowBuffer[0], "Status :%s", (bitRead(deviceStatus, BITPOS_TC_STATUS) == MURUP) ? "ON" : "OFF");
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
      if (lcdCursor == 5)
        lcd.setCursor(15, 0);
      else if (lcdCursor == 4)
        lcd.setCursor(0, 3);
      else if (lcdCursor == 3)
        lcd.setCursor(0, 3 - lcdRowPos);
      else if (lcdCursor == 2)
        lcd.setCursor(0, 2 - lcdRowPos);
      else if (lcdCursor == 1)
        lcd.setCursor(0, 1 - lcdRowPos);
      else if (lcdCursor == 0)
        lcd.setCursor(0, 0);
      lcd.print(LCD_ARROW);
    }
  }
  else if (lcdScreen == 5 || lcdScreen == 6)
  {
    if (prev_lcdCursor != lcdCursor)
    {
      if (prev_lcdCursor == 2 + lcdRowPos && lcdCursor == 3 + lcdRowPos && lcdCursor != lcdCursorBackPos)
      { // Scroll down
        lcdRowPos++;
        lcd.clear();
        memcpy(&lcdRowBuffer[0], &lcdRowBuffer[1], sizeof lcdRowBuffer[1]);
        memcpy(&lcdRowBuffer[1], &lcdRowBuffer[2], sizeof lcdRowBuffer[2]);
        memcpy(&lcdRowBuffer[2], &lcdRowBuffer[3], sizeof lcdRowBuffer[3]);
        if (lcdRowPos == 1)
          sprintf(lcdRowBuffer[3], "Kp:%05.2f", (lcdScreen == 5) ? heaterKp : coolerKp);
        else if (lcdRowPos == 2)
          sprintf(lcdRowBuffer[3], "Ki:%05.2f", (lcdScreen == 5) ? heaterKi : coolerKi);
        else if (lcdRowPos == 3)
          sprintf(lcdRowBuffer[3], "Kd:%05.2f", (lcdScreen == 5) ? heaterKd : coolerKd);
        else if (lcdRowPos == 4)
          sprintf(lcdRowBuffer[3], "Durasi:%04.0f ms", (lcdScreen == 5) ? heaterDs : coolerDs);
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
        if (lcdRowPos == 0)
          sprintf(lcdRowBuffer[0], (lcdScreen == 5) ? "Pemanas" : "Pendingin");
        else if (lcdRowPos == 1)
          sprintf(lcdRowBuffer[0], "Mode:%s", (bitRead(htclMode, (lcdScreen == 5) ? BITPOS_HEATER_MODE : BITPOS_COOLER_MODE) == MODE_PID) ? "PID" : "HYS");
        else if (lcdRowPos == 2)
          sprintf(lcdRowBuffer[0], "Bts Ats:%05.2f", (lcdScreen == 5) ? heaterBa : coolerBa);
        else if (lcdRowPos == 3)
          sprintf(lcdRowBuffer[0], "Bts Bwh:%05.2f", (lcdScreen == 5) ? heaterBb : coolerBb);
        lcd.setCursor((strcmp(lcdRowBuffer[0], (lcdScreen == 5) ? "Pemanas" : "Pendingin") == 0) ? 0 : 1, 0);
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

      if (lcdRowPos != 0)
      {
        lcd.setCursor(0, 0);
        lcd.print(" ");
      }
      lcd.setCursor(0, 1);
      lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(" ");
      lcd.setCursor(0, 3);
      lcd.print(" ");
      lcd.setCursor(15, 0);
      lcd.print(" ");
      if (lcdCursor != lcdCursorBackPos)
        lcd.setCursor(0, lcdCursor - lcdRowPos + 1);
      else
        lcd.setCursor(15, 0);
      lcd.print(LCD_ARROW);
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
  prev_thermalSetPoint = thermalSetPoint;
  prev_wifiStatus = isWifiConnected();

  prev_outStatus[0] = bitRead(deviceStatus, BITPOS_HEATER_STATUS);
  prev_outStatus[1] = bitRead(deviceStatus, BITPOS_COOLER_STATUS);
  prev_outStatus[2] = bitRead(deviceStatus, BITPOS_AUX1_STATUS);
  prev_outStatus[3] = bitRead(deviceStatus, BITPOS_AUX2_STATUS);

  prev_tcStatus = bitRead(deviceStatus, BITPOS_TC_STATUS);
  prev_tcOperation = bitRead(deviceStatus, BITPOS_TC_OPERATION);
  bitWrite(prev_tcMode, 0, bitRead(deviceStatus, BITPOS_TC_MODE_B0));
  bitWrite(prev_tcMode, 1, bitRead(deviceStatus, BITPOS_TC_MODE_B1));

  prev_clMode = bitRead(htclMode, BITPOS_COOLER_MODE);
  prev_coolerKp = coolerKp;
  prev_coolerKi = coolerKi;
  prev_coolerKd = coolerKd;
  prev_coolerDs = coolerDs;
  prev_coolerBa = coolerBa;
  prev_coolerBb = coolerBb;
  prev_htMode = bitRead(htclMode, BITPOS_HEATER_MODE);
  prev_heaterKp = heaterKp;
  prev_heaterKi = heaterKi;
  prev_heaterKd = heaterKd;
  prev_heaterDs = heaterDs;
  prev_heaterBa = heaterBa;
  prev_heaterBb = heaterBb;
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
  htclMode = data[ADDR_HTCL_MODE - ADDR_DEVICE_STATUS];
  memcpy(&thermalSetPoint, &data[ADDR_THERMAL_SETPOINT - ADDR_DEVICE_STATUS], sizeof(float));
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
void fetchURL(const String &URL, const String &data, int &responseCode, String &response)
{
  unsigned long dt = millis();

  client->setX509Time(timeClient.getEpochTime());
  // client.setInsecure();
  // client.setCiphersLessSecure();
  if (!client->connect(baseUri, 443))
  {
    Serial.printf("Connecting takes %lums\n", millis() - dt);
    Serial.println("connection failed");
    responseCode = 408;
    response = "";
  }
  else
  {
    Serial.printf("Connecting takes %lums\n", millis() - dt);
    // configure traged server and url
    http.begin(dynamic_cast<WiFiClient &>(*client), URL); //HTTP
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
        response = http.getString();
      }
    }
    else
    {
      Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
    http.end();
  }
}

bool isWifiConnected()
{
  return (WiFi.status() == WL_CONNECTED);
}

bool initiateSoftAP()
{
  programStarted = false;
  lcd.clear();
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
  lcd.clear();
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
  closeClient();
  closeServer();
  closeSoftAP();
  initiateSoftAP();
  deployWebServer();

  if (responseStatus == "success" || responseStatus == "recon")
  {
    statusBuzzer.setPattern(100, buzzerSuccessPattern, sizeof(buzzerErrorPattern) / sizeof(buzzerErrorPattern[0]));
    statusBuzzer.disableAfter(3000);
  }
  else
  {
    statusBuzzer.setPattern(500, buzzerErrorPattern, sizeof(buzzerErrorPattern) / sizeof(buzzerErrorPattern[0]));
    statusBuzzer.disableAfter(3000);
  }
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
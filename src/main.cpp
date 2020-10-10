#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include "AES.h"
#include "b64.h"
#include <Wire.h>
#include <Eeprom24C04_16.h>
#include <ArduinoJson.h>
#include <html.h>

#define EEPROM_ADDRESS 0x50
#define DEVICETOKEN "te9Dz1MfFK"
#define FB 0
#define WIFISSID 1
#define WIFIPW 2
#define SOFTSSID 3
#define SOFTPW 4
#define USERNAME 5
#define IPADDRESS 6
#define GATEWAY 7

AES aes;
AES aesDecript;
HTTPClient http;
WiFiClient client;
static Eeprom24C04_16 eeprom(EEPROM_ADDRESS);
ESP8266WebServer server(80); // Create a webserver object that listens for HTTP request on port 80

String storedUsername;
String storedSSID;
String storedWifiPass;
String storedSoftSSID;
String storedSoftWifiPass;
String storedIPAddress;
byte storedFirstByte;

bool serverAvailable;
bool triedLog = false;

// The necessary encryption information: First the pre-shared key.
byte key[] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};
// The IV's always have the same size used for AES: 16 bytes
byte ivByteArray[16] = {48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63};

uint8_t getrnd();
char *multi_tok(char *input, char *delimiter);
void gen_iv(byte *iv);
String encryptData(String message);
String decryptData(String data);
void storeString(int addrOffset, const String &strToWrite);
String readString(int addrOffset);
String readFromEEPROM(byte what);
void writeToEEPROM(byte what, String strToWrite);
byte readFB();
void writeFB(byte data);
void loadInfo();
void processIPString(String *buffer, String ipdata);
bool initiateSoftAP();
bool initiateClient(String ssid, String pass);
void deployWebServer();
void closeServer();
bool closeSoftAP();
bool closeClient();
bool isWifiConnected();
String fetchURL(String URL, String payload, int &responseCode);
void writeByteEEPROM(int addr, byte data);

void pgRoot(); // function prototypes for HTTP handlers
void pgInit();
void pgRestart();
void pgReqStatus();
void pgAccInfo();
void handleNotFound();

void setup(void)
{
  Serial.begin(74880); // Start the Serial communication to send messages to the computer
  delay(10);
  Serial.println('\n');

  // pinMode(LED_BUILTIN, OUTPUT);
  delay(100);
  eeprom.initialize();
  loadInfo();
  closeClient();
  closeSoftAP();
  closeServer();
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
  Serial.print("storedIPAddress : ");
  Serial.println(storedIPAddress);

  if (readFB() == 0)
  {
    initiateSoftAP();
    deployWebServer(); // Deploy web server
  }
  else
  {
    if (!initiateClient(storedSSID, storedWifiPass)) // First try on connecting to stored SSID
    {
      if (!initiateClient(storedSSID, storedWifiPass))
        initiateSoftAP(); // Seems that SSID is invalid, initiate Soft AP instead
      deployWebServer();
    }
  }
}

unsigned long loopMillis;
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
    }
  }
  // if (!serverClosed)
  // if (storedFirstByte == 1 && millis() - loopMillis >= 1000)
  // {
  //   loopMillis = millis();
  //   if ((WiFi.status() == WL_CONNECTED))
  //   {

  //     Serial.print("[HTTP] begin...\n");

  //     WiFiClient client;
  //     http.begin(client, "http://192.168.7.65:8080/db_getLastStatus.php?token=keSvw4Hwt6");

  //     Serial.print("[HTTP] GET...\n");
  //     // start connection and send HTTP header
  //     int httpCode = http.GET();

  //     // httpCode will be negative on error
  //     if (httpCode > 0)
  //     {
  //       // HTTP header has been send and Server response header has been handled
  //       Serial.printf("[HTTP] GET... code: %d\n", httpCode);

  //       // file found at server
  //       if (httpCode == HTTP_CODE_OK)
  //       {
  //         String payload = http.getString();
  //         Serial.println(payload);
  //         if (payload == "OFF")
  //           digitalWrite(LED_BUILTIN, HIGH);
  //         else if (payload == "ON")
  //           digitalWrite(LED_BUILTIN, LOW);
  //       }
  //     }
  //     else
  //     {
  //       Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
  //     }

  //     http.end();
  //   }
  //   else
  //   {
  //     Serial.println("DISCONNECTED");
  //   }
  // }
}

String fetchURL(String URL, String data, int &responseCode)
{
  WiFiClient client;
  HTTPClient http;

  Serial.print("[HTTP] begin...\n");
  // configure traged server and url
  http.begin(client, URL); //HTTP
  http.addHeader("Content-Type", "application/json");

  Serial.print("[HTTP] POST...\n");
  // start connection and send HTTP header and body
  int httpCode = http.POST(data);
  responseCode = httpCode;

  // httpCode will be negative on error
  if (httpCode > 0)
  {
    // HTTP header has been send and Server response header has been handled
    Serial.printf("[HTTP] POST... code: %d\n", httpCode);

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

uint8_t getrnd()
{
  uint8_t really_random = *(volatile uint8_t *)0x3FF20E44;
  return really_random;
}

char *multi_tok(char *input, char *delimiter)
{
  static char *string;
  if (input != NULL)
    string = input;

  if (string == NULL)
    return string;

  char *end = strstr(string, delimiter);
  if (end == NULL)
  {
    char *temp = string;
    string = NULL;
    return temp;
  }

  char *temp = string;

  *end = '\0';
  string = end + strlen(delimiter);
  return temp;
}

// Generate a random initialization vector
void gen_iv(byte *iv)
{
  for (int i = 0; i < N_BLOCK; i++)
  {
    iv[i] = (byte)getrnd();
  }
}

// with 632 array allocation, 284 maximum of character is achieved
String encryptData(String message)
{
  char b64dataIV[64];
  byte cipher[632];
  char b64dataMessage[632];

  gen_iv(ivByteArray);
  b64_encode(b64dataIV, (char *)ivByteArray, N_BLOCK); // Encode IV B64

  aes.do_aes_encrypt((byte *)message.c_str(), message.length(), cipher, key, 128, ivByteArray); // Encrypt Message

  b64_encode(b64dataMessage, (char *)cipher, aes.get_size()); // Encode Encrypted Message

  return String(b64dataIV) + "%" + String(b64dataMessage); // Send data
}

String decryptData(String data)
{
  char decodedData[1500];
  char dataDec[1500];
  byte out[1500];
  char ivDec[64];
  char ivDec64[64];

  char *token = strtok((char *)data.c_str(), "%"); // Explode data
  byte tokenCounter = 0;
  while (token != NULL)
  {
    if (tokenCounter == 0)
    {
      strcpy(ivDec64, token);
    }
    else
    {
      strcpy(dataDec, token);
    }
    token = strtok(NULL, " ");
    tokenCounter++;
  }

  b64_decode(ivDec, ivDec64, strlen(ivDec64));

  int decodeLength = b64_decode(decodedData, dataDec, strlen(dataDec));

  Serial.println(decodeLength);
  aes.do_aes_decrypt((byte *)decodedData, decodeLength, out, key, 128, (byte *)ivDec);
  int i = 0;
  while ((out[i] < 128 && out[i] > 31) || out[i] == 194 || out[i] == 182)
    i++;
  out[i] = '\0';
  return String((char *)out);
}

void storeString(int addrOffset, const String &strToWrite)
{
  byte len = strToWrite.length();
  byte data[len];
  strToWrite.getBytes(data, len + 1);
  eeprom.writeByte(addrOffset, len);
  delay(10);
  eeprom.writeBytes(addrOffset + 1, len, data);
}

String readString(int addrOffset)
{
  int newStrLen = eeprom.readByte(addrOffset);
  byte data[newStrLen + 1];
  eeprom.readBytes(addrOffset + 1, newStrLen, data);
  data[newStrLen] = '\0';
  return String((char *)data);
}

byte readFB()
{
  storedFirstByte = eeprom.readByte(0);
  return storedFirstByte;
}

void writeFB(byte data)
{
  eeprom.writeByte(0, data);
  delay(10);
}

String readFromEEPROM(byte what)
{
  if (what == WIFISSID)
  {
    storedSSID = readString(1);
    return storedSSID;
  }
  else if (what == WIFIPW)
  {
    storedWifiPass = readString(33);
    return storedWifiPass;
  }
  else if (what == SOFTSSID)
  {
    storedSoftSSID = readString(108);
    return storedSoftSSID;
  }
  else if (what == SOFTPW)
  {
    storedSoftWifiPass = readString(140);
    return storedSoftWifiPass;
  }
  else if (what == USERNAME)
  {
    storedUsername = readString(205);
    return storedUsername;
  }
  else if (what == IPADDRESS)
  {
    storedIPAddress = readString(237);
    return storedIPAddress;
  }

  return "INVALID";
}

void writeToEEPROM(byte what, String strToWrite)
{
  unsigned int address = 1;

  if (what == WIFISSID)
  {
    storedSSID = strToWrite;
    address = 1;
  }
  else if (what == WIFIPW)
  {
    storedWifiPass = strToWrite;
    address = 33;
  }
  else if (what == SOFTSSID)
  {
    storedSoftSSID = strToWrite;
    address = 108;
  }
  else if (what == SOFTPW)
  {
    storedSoftWifiPass = strToWrite;
    address = 140;
  }
  else if (what == USERNAME)
  {
    storedUsername = strToWrite;
    address = 205;
  }
  else if (what == IPADDRESS)
  {
    storedIPAddress = strToWrite;
    address = 237;
  }
  storeString(address, strToWrite);
}

void loadInfo()
{
  storedFirstByte = readFB();
  storedSSID = readFromEEPROM(WIFISSID);
  storedWifiPass = readFromEEPROM(WIFIPW);
  storedSoftSSID = readFromEEPROM(SOFTSSID);
  storedSoftWifiPass = readFromEEPROM(SOFTPW);
  storedUsername = readFromEEPROM(USERNAME);
  storedIPAddress = readFromEEPROM(IPADDRESS);
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
  if (storedIPAddress == " ")
    storedIPAddress = "192.168.4.1";
}

void processIPString(String *buffer, String ipdata)
{
  char ipCopy[20];
  ipdata.toCharArray(ipCopy, 20);
  char *token = strtok(ipCopy, "."); // Explode data
  byte tokenCounter = 0;

  while (token != NULL)
  {
    buffer[tokenCounter] = String(token);
    token = strtok(NULL, ".");
    tokenCounter++;
  }
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
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  delay(100);
  return timeOutFlag;
}

bool initiateClient(String ssid, String pass)
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
      delay(500);
    }
    if (i == 19)
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

void pgRoot()
{
  server.send(200, "text/html", FPSTR(htmlDoc));
}

void pgRestart()
{
  delay(100);
  ESP.reset();
}

String responseStatus = "empty";
void pgAccInfo()
{
  String usrn = server.arg("usrn");
  String unpw = server.arg("unpw");
  String ssid = server.arg("ssid");
  String wfpw = server.arg("wfpw");
  Serial.printf("Username : %s\nPassword : %s\nSSID : %s\nWiFiPW : %s\n", usrn.c_str(), unpw.c_str(), ssid.c_str(), wfpw.c_str());
  if (initiateClient(ssid, wfpw))
  {
    Serial.print("[HTTP] begin...\n");
    int httpCode;
    const size_t capacity = JSON_OBJECT_SIZE(5);
    DynamicJsonDocument doc(capacity);
    String json = "";
    doc["username"] = usrn.c_str();
    doc["password"] = unpw.c_str();
    doc["devicetoken"] = String(DEVICETOKEN).c_str();
    doc["softssid"] = storedSoftSSID.c_str();
    doc["softpw"] = storedSoftWifiPass.c_str();
    serializeJson(doc, json);
    responseStatus = fetchURL("http://192.168.2.110:8080/otoma/teste.php", json, httpCode);
    if (httpCode == HTTP_CODE_OK)
    {
      if (responseStatus == "success")
      {
        writeToEEPROM(USERNAME, usrn);
        writeFB(1);
      }
    }
    else
      responseStatus = "nocon";
  }
  else
  {
    responseStatus = "invwifi";
  }
  closeClient();
  closeServer();
  closeSoftAP();
  initiateSoftAP();
  deployWebServer();
}

void pgReqStatus()
{
  const size_t capacity = JSON_OBJECT_SIZE(4);
  DynamicJsonDocument doc(capacity);
  String json = "";
  doc["usrn"] = storedUsername.c_str();
  doc["ssid"] = storedSSID.c_str();
  doc["wfpw"] = storedWifiPass.c_str();
  doc["message"] = responseStatus.c_str();
  Serial.println(storedUsername);
  Serial.println(storedSSID);
  Serial.println(storedWifiPass);
  Serial.println(responseStatus);
  serializeJson(doc, json);
  Serial.printf("Transferring JSON : %s\n", json.c_str());
  server.send(200, "application/json", json);
}

void handleNotFound()
{
  server.send(404, "text/plain", "404: Not found");
}
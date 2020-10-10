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
bool showPassword = 0;

String deviceToken = "te9Dz1MfFK";

int status = 1;

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
void loadInfo();
void processIPString(String *buffer, String ipdata);
void initiateSoftAP();
bool initiateClient(String ssid, String pass);
String fetchURL(String URL, String payload, int &responseCode);
void writeByteEEPROM(int addr, byte data);

void handleRoot(); // function prototypes for HTTP handlers
void handleLogin();
void handleSoft();
void handleRestart();
void handlePasswordVisibility();
void handleNotFound();
String htmlCode();
bool serverClosed;

void setup(void)
{
  Serial.begin(74880); // Start the Serial communication to send messages to the computer
  delay(10);
  Serial.println('\n');

  // pinMode(LED_BUILTIN, OUTPUT);
  delay(100);
  eeprom.initialize();
  loadInfo();

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
  Serial.println(FPSTR(htmlDoc));
  Serial.printf("Length : %u", strlen_P(htmlDoc));
  delay(99999999);
  if (storedFirstByte == 0)
  {
    initiateSoftAP();
  }
  else if (storedFirstByte == 1)
  {
    if (initiateClient(storedSSID, storedWifiPass))
      ;
    else
      initiateSoftAP();
  }
}

unsigned long loopMillis;
void loop(void)
{
  if (!serverClosed)
    server.handleClient(); // Listen for HTTP requests from clients
  if (storedFirstByte == 1 && millis() - loopMillis >= 1000)
  {
    loopMillis = millis();
    if ((WiFi.status() == WL_CONNECTED))
    {

      Serial.print("[HTTP] begin...\n");

      WiFiClient client;
      http.begin(client, "http://192.168.7.65:8080/db_getLastStatus.php?token=keSvw4Hwt6");

      Serial.print("[HTTP] GET...\n");
      // start connection and send HTTP header
      int httpCode = http.GET();

      // httpCode will be negative on error
      if (httpCode > 0)
      {
        // HTTP header has been send and Server response header has been handled
        Serial.printf("[HTTP] GET... code: %d\n", httpCode);

        // file found at server
        if (httpCode == HTTP_CODE_OK)
        {
          String payload = http.getString();
          Serial.println(payload);
          if (payload == "OFF")
            digitalWrite(LED_BUILTIN, HIGH);
          else if (payload == "ON")
            digitalWrite(LED_BUILTIN, LOW);
        }
      }
      else
      {
        Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
      }

      http.end();
    }
    else
    {
      Serial.println("DISCONNECTED");
    }
  }
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

void loadInfo()
{
  storedFirstByte = eeprom.readByte(0);
  storedSSID = readString(1);
  storedWifiPass = readString(33);
  storedSoftSSID = readString(108);
  storedSoftWifiPass = readString(140);
  storedUsername = readString(205);
  storedIPAddress = readString(237);
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

void initiateSoftAP()
{
  String octet[4];
  bool timeOutFlag;
  processIPString(octet, storedIPAddress);
  Serial.println();
  Serial.println(octet[0] + "." + octet[1] + "." + octet[2] + "." + octet[3]);
  IPAddress local_IP(octet[0].toInt(), octet[1].toInt(), octet[2].toInt(), octet[3].toInt());
  IPAddress gateway(octet[0].toInt(), octet[1].toInt(), octet[2].toInt(), octet[3].toInt() + 1);
  IPAddress subnet(255, 255, 255, 0);
  delay(100);
  for (int i = 0; i < 20; i++)
  {
    if (WiFi.softAPConfig(local_IP, gateway, subnet))
    {
      Serial.println("Configuring softAP");
      timeOutFlag = 0;
      break;
    }
    else
    {
      Serial.print(".");
      delay(500);
    }
    if (i == 19)
    {
      timeOutFlag = 1;
    }
  }
  Serial.println();
  delay(100);
  for (int i = 0; i < 20; i++)
  {
    if (WiFi.softAP(storedSoftSSID, storedSoftWifiPass))
    {
      Serial.println("Starting softAP");
      timeOutFlag = 0;
      break;
    }
    else
    {
      Serial.print(".");
      delay(500);
    }
    if (i == 19)
    {
      timeOutFlag = 1;
    }
  }
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  delay(100);

  server.on("/", HTTP_GET, handleRoot);                                 // Call the 'handleRoot' function when a client requests URI "/"
  server.on("/restart", HTTP_GET, handleRestart);                       // Call the 'handleRestart' function when a client requests URI "/restart"
  server.on("/passwordvisibility", HTTP_GET, handlePasswordVisibility); // Call the 'handleRestart' function when a client requests URI "/restart"
  server.on("/login", HTTP_POST, handleLogin);                          // Call the 'handleLogin' function when a POST request is made to URI "/login"
  server.on("/soft", HTTP_POST, handleSoft);                            // Call the 'handleSoft' function when a POST request is made to URI "/soft"

  server.onNotFound(handleNotFound); // When a client requests an unknown URI (i.e. something other than "/"), call function "handleNotFound"

  server.begin();
}

bool initiateClient(String ssid, String pass)
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  bool successFlag = false;
  for (int i = 0; i < 20; i++)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("Connected!");
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

void handleRoot()
{ // When URI / is requested, send a web page with a button to toggle the LED
  server.send(200, "text/html", htmlCode());
}

void handlePasswordVisibility()
{
  showPassword = !showPassword;
  delay(100);
  server.send(200, "text/html", htmlCode());
}

void handleRestart()
{
  server.send(200, "text/plain", "Merestart perangkat...");
  delay(100);
  ESP.reset();
}

String softErrorCode = "";
void handleSoft()
{
  String softssid = server.arg("softssid");
  String softpass = server.arg("softpass");
  String softip = server.arg("ip");
  bool errorFlag = 0;
  String octet[4];
  processIPString(octet, softip);
  if (octet[0] != "192")
  {
    errorFlag = 1;
    softErrorCode += "<span style=\"color: var(--failcolor);\">Oktet pertama IP Address harus \"192\"</span><br/>";
  }
  if (octet[1] != "168")
  {
    errorFlag = 1;
    softErrorCode += "<span style=\"color: var(--failcolor);\">Oktet kedua IP Address harus \"168\"</span><br/>";
  }
  if (octet[2] == "")
  {
    errorFlag = 1;
    softErrorCode += "<span style=\"color: var(--failcolor);\">Oktet ketiga IP Address tidak boleh kosong</span><br/>";
  }
  else if (octet[2].toInt() < 1 && octet[2] != "")
  {
    errorFlag = 1;
    softErrorCode += "<span style=\"color: var(--failcolor);\">Oktet ketiga IP Address tidak boleh kurang dari 1</span><br/>";
  }
  if (octet[3] == "")
  {
    errorFlag = 1;
    softErrorCode += "<span style=\"color: var(--failcolor);\">Oktet keempat IP Address tidak boleh kosong</span><br/>";
  }
  else if (octet[3].toInt() < 1 && octet[3] != "")
  {
    errorFlag = 1;
    softErrorCode += "<span style=\"color: var(--failcolor);\">Oktet keempat IP Address tidak boleh kurang dari 1</span><br/>";
  }
  if (octet[2].toInt() > 248)
  {
    errorFlag = 1;
    softErrorCode += "<span style=\"color: var(--failcolor);\">Oktet ketiga IP Address tidak boleh lebih dari \"248\"</span><br/>";
  }
  if (octet[3].toInt() > 248)
  {
    errorFlag = 1;
    softErrorCode += "<span style=\"color: var(--failcolor);\">Oktet keempat IP Address tidak boleh lebih dari \"248\"</span><br/>";
  }
  if (softssid.length() == 0)
  {
    errorFlag = 1;
    softErrorCode += "<span style=\"color: var(--failcolor);\">SSID tidak boleh kosong</span><br/>";
  }
  if (softssid.length() > 24)
  {
    softErrorCode += "<span style=\"color: var(--failcolor);\">Panjang SSID maksimum 24 karakter</span><br/>";
    errorFlag = 1;
  }
  if (softpass.length() > 32)
  {
    softErrorCode += "<span style=\"color: var(--failcolor);\">Panjang Password Wifi maksimum 32 karakter</span><br/>";
    errorFlag = 1;
  }
  if (softpass.length() != 0 && softpass.length() < 8)
  {
    softErrorCode += "<span style=\"color: var(--failcolor);\">Jumlah karakter yang diperbolehkan untuk password adalah 0 dan lebih dari sama dengan 8</span><br/>";
    errorFlag = 1;
  }
  if (errorFlag)
  {
    status = 13;
    server.send(200, "text/html", htmlCode());
    return;
  }
  Serial.println();
  Serial.print("softSSID : ");
  Serial.println(softssid);
  Serial.print("softPass : ");
  Serial.println(softpass);
  Serial.print("softIPAddress : ");
  Serial.println(softip);
  if (softssid == storedSoftSSID && softpass == storedSoftWifiPass && softip == storedIPAddress)
  {
    status = 15;
    server.send(200, "text/html", htmlCode());
  }
  if (softssid != storedSoftSSID)
  {
    storeString(108, softssid);
  }
  if (softpass != storedSoftWifiPass)
  {
    storeString(140, softpass);
  }
  if (softip != storedIPAddress)
  {
    storeString(237, softip);
  }
  loadInfo();
  status = 12;
  server.send(200, "text/html", htmlCode());
}

String errorCode;
void handleLogin()
{
  // If a POST request is made to URI /login
  bool timeOutFlag;
  String userpass = server.arg("userpass");
  String username = server.arg("username");
  String ssid = server.arg("ssid");
  String password = server.arg("wifipass");
  Serial.print("username entered : ");
  Serial.println(username);
  Serial.print("userpass entered : ");
  Serial.println(userpass);
  Serial.print("ssid entered : ");
  Serial.println(ssid);
  Serial.print("wifipass entered : ");
  Serial.println(password);
  if (ssid == "")
  {
    errorCode += "<span style=\"color: var(--failcolor);\">SSID WiFi tidak boleh kosong, isi dengan SSID atau nama WiFi yang akan anda pakai</span><br/>";
    status = 5;
  }
  if (username == "")
  {
    errorCode += "<span style=\"color: var(--failcolor);\">Username akun tidak boleh kosong</span><br/>";
    status = 5;
  }
  if (userpass == "")
  {
    errorCode += "<span style=\"color: var(--failcolor);\">Password akun tidak boleh kosong</span><br/>";
    status = 5;
  }
  if (status == 5)
  {
    server.send(200, "text/html", htmlCode());
    return;
  }
  status = 2;
  server.send(200, "text/html", htmlCode());
  delay(100);
  server.close();
  serverClosed = 1;
  while (WiFi.softAPdisconnect(true))
  {
    delay(500);
    Serial.println("breaking....");
  }
  timeOutFlag = !(initiateClient(ssid, password));
  delay(100);
  if (timeOutFlag)
  {
    status = 3;
  }
  else
  {
    ////////
    Serial.print("[HTTP] begin...\n"); // configure traged server and url
    int httpCode;
    const size_t capacity = JSON_OBJECT_SIZE(5);
    DynamicJsonDocument doc(capacity);
    String json = "";
    doc["username"] = username;
    doc["password"] = userpass;
    doc["devicetoken"] = deviceToken;
    doc["softssid"] = storedSoftSSID;
    doc["softpw"] = storedSoftWifiPass;
    serializeJson(doc, json);
    String payload = fetchURL("http://192.168.2.110:8080/otoma/teste.php", json, httpCode);
    Serial.println(payload);
    // Send HTTP POST request
    // httpCode will be negative on error
    if (httpCode > 0)
    {
      // HTTP header has been send and Server response header has been handled
      Serial.printf("[HTTP] POST... code: %d\n", httpCode);

      // file found at server
      if (httpCode == HTTP_CODE_OK)
      {
        if (payload.indexOf("CONNECTED") >= 0)
        {
          status = 4;
          eeprom.writeByte(0, 1);
          storeString(1, ssid);
          storeString(33, password);
          storeString(205, username);
          loadInfo();
        }
        else if (payload.indexOf("RECON") >= 0)
        {
          status = 11;
          eeprom.writeByte(0, 1); // set first flag
          storeString(1, ssid);
          storeString(33, password);
          storeString(205, username);
          loadInfo();
        }
        else if (payload.indexOf("USER ERROR") >= 0)
        {
          status = 7;
        }
        else if (payload.indexOf("PASS ERROR") >= 0)
          status = 8;
        else if (payload.indexOf("INVALID") >= 0)
          status = 6;
        else if (payload.indexOf("DUPLICATE") >= 0)
          status = 10;
        else
          status = 9;
      }
    }
    else
    {
      Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
      status = 9;
    }
    ////////////////////////
  }
  delay(100);
  initiateSoftAP();
  server.send(200, "text/html", htmlCode());
}

void handleNotFound()
{
  server.send(404, "text/plain", "404: Not found"); // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
}

String htmlCode()
{
  String code = "<!DOCTYPE html> <html lang=\"en\"> <head> <meta charset=\"UTF-8\" />\n";
  code += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\" />\n";
  code += "<title>Ootoma IoT Device</title>";
  code += "<style>:root {--normalcolor:#011627;--failcolor:#f51000;--successcolor: #00ad17;--main-bg-color: #e4dfda;--second-color: #6a3937;--third-color: #011627;--fourth-color: #9d79bc;--fifth-color: #2374ab;}html {font-family: Helvetica, sans-serif;display: inline-block;text-align: center;color: var(--third-color);}body {margin-top: 50px;margin-left: auto;background-color: var(--main-bg-color);}h1 {columns: #444444;margin: 50px auto 30px;}h3 {color: #444444;margin-bottom: 50px;}.isian {width: 40%;height: 24px;margin-right: 2%;}.tombolsubmit {margin-top: 2%;margin-left: -2%;width: 84%;background-color: var(--fourth-color);}.grid-container {display: grid;grid-template-columns: auto auto;padding: 10px;}.grid-item {padding: 20px;text-align: center;}.soft {width: 80%;height: 24px;margin-right: 2%;margin-bottom:10px;}</style>";

  code += "</head><body><div style=\"width: 100vw; height: 100px; margin-bottom:25px;\">";
  code += "<form action=\"/login\" method=\"POST\">";
  if (storedSSID == "")
  {
    code += "<input type=\"text\" placeholder=\"SSID WiFi\" class=\"isian\" name=\"ssid\" />";
  }
  else
  {
    code += "<input type=\"text\" placeholder=\"SSID WiFi\" class=\"isian\" name=\"ssid\" value=\"";
    code += storedSSID;
    code += "\"/>";
  }
  if (storedUsername == "")
  {
    code += "<input type=\"text\" placeholder=\"Username Akun\" class=\"isian\" name=\"username\"/><br/><br/>";
  }
  else
  {
    code += "<input type=\"text\" placeholder=\"Username Akun\" class=\"isian\" name=\"username\" value=\"";
    code += storedUsername;
    code += "\"/><br/><br/>";
  }

  if (storedWifiPass == "")
  {
    if (showPassword)
    {
      code += "<input type=\"text\" placeholder=\"Password WiFi\" class=\"isian\" name=\"wifipass\" />";
    }
    else
    {
      code += "<input type=\"password\" placeholder=\"Password WiFi\" class=\"isian\" name=\"wifipass\" />";
    }
  }
  else
  {
    if (showPassword)
    {
      code += "<input type=\"text\" placeholder=\"Password WiFi\" class=\"isian\" name=\"wifipass\" value=\"";
    }
    else
    {
      code += "<input type=\"password\" placeholder=\"Password WiFi\" class=\"isian\" name=\"wifipass\" value=\"";
    }
    code += storedWifiPass;
    code += "\"/>";
  }

  code += "<input type=\"password\" placeholder=\"Password Akun\" class=\"isian\" name=\"userpass\" />";

  code += "<input type=\"submit\" class=\"tombolsubmit\" /><br/>";

  code += "</form></div>";

  if (status == 2)
  {
    IPAddress myIP = WiFi.softAPIP();
    String IPku = myIP.toString();
    code += "<span style=\"color: var(--normalcolor);\">Mencoba menyambungkan... silahkan membuka " + IPku + " kembali setelah beberapa saat</span>";
  }
  else if (status == 3)
    code += "<span style=\"color: var(--failcolor);\">Tidak dapat tersambung ke server, Pastikan SSID dan Password WiFi anda benar dan dapat terkoneksi ke internet</span>";
  else if (status == 4)
    code += "<span style=\"color: var(--successcolor);\">Sukses menghubungkan perangkat dan akun ke server, mohon restart perangkat</span>";
  else if (status == 1)
    code += "<span style=\"color: var(--normalcolor);\">Tolong isi kredensial pada kotak yang tersedia dengan benar</span>";
  else if (status == 5)
  {
    code += errorCode;
    errorCode = "";
    status = 1;
  }
  else if (status == 6)
  {
    code += "<span style=\"color: var(--failcolor);\">Perangkat anda ILLEGAL, apabila anda tidak yakin, tolong hubungi kami di ootomaiot.com</span>";
  }
  else if (status == 7)
  {
    code += "<span style=\"color: var(--failcolor);\">Username akun yang anda masukkan salah</span>";
  }
  else if (status == 8)
  {
    code += "<span style=\"color: var(--failcolor);\">Password akun yang anda masukkan salah</span>";
  }
  else if (status == 9)
  {
    code += "<span style=\"color: var(--failcolor);\">Tidak didapatkan respon dari server, tolong coba lagi sesaat kemudian serta pastikan SSID dan Password WiFi anda benar dan dapat terkoneksi ke internet</span>";
  }
  else if (status == 10)
  {
    code += "<span style=\"color: var(--failcolor);\">Gagal menyambung, Perangkat anda sudah tersambung ke akun lain, apabila anda tidak yakin tolong hubungi kami di ootomaiot.com</span>";
  }
  else if (status == 11)
  {
    code += "<span style=\"color: var(--successcolor);\">Akun dengan username " + storedUsername + " berhasil tersambung kembali ke server</span>";
  }

  code += "<div class=\"grid-container\">";

  code += "<div class=\"grid-item\">";
  code += "<form action=\"soft\" method=\"POST\">";
  code += "<input type=\"text\" placeholder=\"SSID Perangkat\" class=\"soft\" name=\"softssid\" value=\"" + storedSoftSSID + "\"/><br />";
  if (showPassword)
  {
    code += "<input type=\"text\" placeholder=\"Password WiFi Perangkat\" class=\"soft\" name=\"softpass\" value=\"" + storedSoftWifiPass + "\"/>";
  }
  else
  {
    code += "<input type=\"password\" placeholder=\"Password WiFi Perangkat\" class=\"soft\" name=\"softpass\" value=\"" + storedSoftWifiPass + "\"/>";
  }
  code += "<input type=\"text\" placeholder=\"IP Address Perangkat\" class=\"soft\" name=\"ip\" value=\"" + storedIPAddress + "\"/>";
  code += "<input type=\"submit\" class=\"tombolsubmit\" /></form>";
  if (status == 12)
  {
    code += "<span style=\"color: var(--successcolor);\">Konfigurasi WiFi perangkat berhasil diubah, restart perangkat untuk menerapkanm perubahan</span>";
  }
  else if (status == 13)
  {
    code += softErrorCode;
    softErrorCode = "";
    status = 1;
  }
  else if (status == 14)
  {
    code += "<span style=\"color: var(--failcolor);\">Terjadi masalah saat mengubah konfigurasi Soft Access Point perangkat, tolong coba lagi setelah beberapa saat</span>";
  }
  else if (status == 15)
  {
    code += "<span style=\"color: var(--normalcolor);\">Tidak ada perubahan diterapkan</span>";
  }
  code += "</div>";
  code += "<div class=\"grid-item\">";
  code += "<form action=\"restart\" ><button class=\"tombolsubmit\">Restart Perangkat</button></form><br/><br/>";
  if (showPassword)
    code += "<form action=\"passwordvisibility\" ><button class=\"tombolsubmit\">Sembunyikan Password WiFi</button></form></div></div>";
  else
    code += "<form action=\"passwordvisibility\" ><button class=\"tombolsubmit\">Tampilkan Password WiFi</button></form></div></div>";

  code += "</body></html>";

  return code;
}
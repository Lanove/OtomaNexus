#define BUILD_VERSION "1.0.0"

#define EEPROM_ADDRESS 0x50
#define UPDATE_CHECK_INTERVAL 86400000 // ONE DAY
#define LCD_UPDATE_INTERVAL 100
#define HTTP_FETCH_INTERVAL 2000
#define SENSOR_UPDATE_INTERVAL 500
#define MAXIMUM_DISCONNECT_TIME 900000 // Maximum WiFi disconnection time or server request time out before rollback to AP Mode and reset FB

#define LCD_ARROW (char)B01111110
#define LCD_DEGREE (char)223

#define MURUP 1 // BEN GA BINGUNG, FOR SOME REASON, INVERTING OUTPUT CAUSES OSCILLATING
#define MATI 0

#define FB 0
#define WIFISSID 1
#define WIFIPW 2
#define SOFTSSID 3
#define SOFTPW 4
#define USERNAME 5
#define IPADDRESS 6
#define GATEWAY 7
#define DEVICETOKEN 8
#define USERPASS 9

#define FB_CONNECTED 0
#define FB_WIFI_ERROR1 1
#define FB_WIFI_ERROR2 2
#define FB_DS_NF1 3
#define FB_DS_NF2 4

#define DHTPIN 13
#define DHTTYPE DHT11

#define DATA595 15
#define LATCH595 2
#define CLOCK595 0

#define ENCODER_PINA 14
#define ENCODER_PINB 16
#define ENCODER_BTN 12

#define ENCODER_STEPS_PER_NOTCH 4 // Change this depending on which encoder is used

#define ENC_UP 0
#define ENC_DOWN 1
#define ENC_OPEN ClickEncoder::Open
#define ENC_CLICKED ClickEncoder::Clicked
#define ENC_DBCLICKED ClickEncoder::DoubleClicked

// QA to QH of 595, in sort
#define SFT_LED_STATUS 0
#define SFT_AUX1_RELAY 1
#define SFT_AUX2_RELAY 2
#define SFT_AUX3_RELAY 3
#define SFT_AUX4_RELAY 4
// #define SFT_HEATER_RELAY 3 DEPRECATED
// #define SFT_COOLER_RELAY 4 DEPRECATED
#define SFT_MUX_A 5
#define SFT_MUX_B 6
#define SFT_BUZZER 7

#define ONE_WIRE_BUS 13

#define DHT_LOOP 5 // Means that DHT will sample once every 5 sample of DS18B20

#define ADDR_FIRST_BYTE 0

// address 602 is 362
#define PROG_LENGTH 366
#define ADDR_DEVICE_STATUS 241
// Device status contain : aux
// 242 is deprecated int setpoint
// 243 is deprecated int setpoint
// #define ADDR_HEATER_KP 244 DEPRECATED
// #define ADDR_HEATER_KI 248 DEPRECATED
// #define ADDR_HEATER_KD 252 DEPRECATED
// #define ADDR_HEATER_DS 256 DEPRECATED
// #define ADDR_HEATER_BA 260 DEPRECATED
// #define ADDR_HEATER_BB 264 DEPRECATED
// #define ADDR_COOLER_KP 268 DEPRECATED
// #define ADDR_COOLER_KI 272 DEPRECATED
// #define ADDR_COOLER_KD 276 DEPRECATED
// #define ADDR_COOLER_DS 280 DEPRECATED
// #define ADDR_COOLER_BA 284 DEPRECATED
// #define ADDR_COOLER_BB 288 DEPRECATED
#define ADDR_PROG_TRIGGER(x) (292 + (x * 10))
#define ADDR_PROG_RB1(x) (293 + (x * 10))
#define ADDR_PROG_RB2(x) (297 + (x * 10))
#define ADDR_PROG_ACTION(x) (301 + (x * 10))
// #define ADDR_HTCL_MODE 602 DEPRECATED
// #define ADDR_THERMAL_SETPOINT 603 DEPRECATED

#define BITPOS_AUX1_STATUS 0
#define BITPOS_AUX2_STATUS 1
// #define BITPOS_TC_STATUS 2   DEPRECATED
// #define BITPOS_TC_OPERATION 3 // 0 is MANUAL MODE, 1 is AUTO MODE  DEPRECATED
// #define BITPOS_TC_MODE_B0 4   // B00 is HEATER MODE, B01 is COOLER MODE DEPRECATED
// #define BITPOS_TC_MODE_B1 5   // B11 is DUAL MODE DEPRECATED
#define BITPOS_AUX3_STATUS 6
#define BITPOS_AUX4_STATUS 7
// #define BITPOS_HEATER_MODE 0 // 0 is PID MODE, 1 is HYSTERESIS MOD DEPRECATED
// #define BITPOS_COOLER_MODE 1  DEPRECATED

// #define MODE_PID 0 DEPRECATED
// #define MODE_HYSTERESIS 1 DEPRECATED
// #define MODE_OPERATION_MANUAL 0 DEPRECATED
// #define MODE_OPERATION_AUTO 1 DEPRECATED
const char baseUri[] = "192.168.43.242";
const int httpPort = 8080;
static const char espUpdater[] PROGMEM = "http://192.168.43.242:8080/otoma/api/ESPUpdater.php";
static const char requestURL[] PROGMEM = "http://192.168.43.242:8080/otoma/api/nexusControllerRequest.php";
static const char identifyURL[] PROGMEM = "http://192.168.43.242:8080/otoma/api/identifyDevice.php";
// This is the minified file of html document to reduce flash usage
// minified is 4857 byte
// unminified is 6358 byte, which is 23.6% saving!
// Original file is testssid.php on xampp htdocs
static const char htmlDoc[] PROGMEM = R"=====(
<!DOCTYPE html><html lang="en"><head> <meta charset="UTF-8"/> <meta name="viewport" content="width=device-width, initial-scale=1.0"/> <title>Otoma Nexus Controller</title> <style>:root{--normalcolor: #011627; --failcolor: #f51000; --successcolor: #00ad17; --main-bg-color: #e4dfda; --second-color: #6a3937; --third-color: #011627; --fourth-color: #9d79bc; --fifth-color: #2374ab;}html{font-family: Helvetica, sans-serif; display: inline-block; text-align: center; color: var(--third-color);}body{margin-top: 50px; margin-left: auto; background-color: var(--main-bg-color);}h1{columns: #444444; margin: 50px auto 30px;}h3{color: #444444; margin-bottom: 50px;}.isian{width: 40%; height: 24px; margin-right: 2%;}.tombolsubmit{margin-top: 2%; margin-left: -2%; width: 84%;}</style></head><body> <h2 id="hd">Masukkan informasi</h2> <div style="width: 100vw; height: 100px; margin-bottom:25px;"> <input type="text" placeholder="SSID/Nama WiFi" class="isian" id="ssid"/> <input type="text" placeholder="Username Akun" class="isian" id="usrn"/> <br/><br/> <input type="password" placeholder="Password WiFi" class="isian" id="wfpw"/> <input type="password" placeholder="Password akun" class="isian" id="unpw"/> <br><input type="checkbox" id="spw" name="spw" value="show" onclick="showPassword();"> <label for="spw"> Tampilkan Password</label> <br><button class="tombolsubmit" onclick="sendInputInfo();">Submit</button> </div><br/><br><span style="color: var(--normalcolor);" id="ds"></span> <br><br><button onclick="restart();">Restart Kontroller</button> <br><h3 style="background-color:red;width:100vw;" id="tm"> Belum termuat </h3> <script>function requestAJAX(url, data, callback=function(){}){var xhr=new XMLHttpRequest(); xhr.open("POST", url); xhr.setRequestHeader("Content-type", "application/x-www-form-urlencoded"); xhr.send(data); xhr.onload=function(){if (xhr.status==200 && xhr.readyState==4){callback(xhr.responseText);}};}function sendInputInfo(){document.getElementById("ds").innerHTML="Mengolah dan mengirim data yang dimasukkan, kontroller akan memutuskan Access Point, tolong tunggu maksimal 30 detik.<br>Setelah Access Point kontroller ini tersedia kembali, hubungkan wifi tersebut lagi lalu masuk laman ini atau 192.168.4.1 sekali lagi untuk mendapat laporan status."; requestAJAX("accInfo", "ssid=" + document.getElementById("ssid").value + "&wfpw=" + document.getElementById("wfpw").value + "&usrn=" + document.getElementById("usrn").value + "&unpw=" + document.getElementById("unpw").value);}function restart(){requestAJAX("restart", "", function(response){document.getElementById("ds").innerHTML="Merestart perangkat dan akan memutus Access Point...";});}function showPassword(){if (document.getElementById("spw").checked){document.getElementById("unpw").type="text"; document.getElementById("wfpw").type="text";}else{document.getElementById("unpw").type="password"; document.getElementById("wfpw").type="password";}}window.onload=function(){requestAJAX("reqStatus", "", function(response){response=JSON.parse(response); document.getElementById("ssid").value=response.ssid; document.getElementById("wfpw").value=response.wfpw; document.getElementById("usrn").value=response.usrn; document.getElementById("unpw").value=response.unpw; document.getElementById("tm").style.backgroundColor="green"; document.getElementById("tm").innerHTML="Berhasil termuat"; if (response.message=="success") document.getElementById("ds").innerHTML="Berhasil menghubungkan akun anda dengan kontroller<br>restart kontroller dengan tombol yang tersedia, lalu kontroller akan dapat dioperasikan secara online di otoma.my.id"; else if (response.message=="wrongid") document.getElementById("ds").innerHTML="Username akun otoma yang anda masukkan tidak terdaftar"; else if (response.message=="wrongpw") document.getElementById("ds").innerHTML="Password akun otoma yang anda masukkan salah"; else if (response.message=="illegal") document.getElementById("ds").innerHTML="Kontroller anda ILLEGAL, apabila anda tidak yakin tolong hubungi kami di otoma.my.id"; else if (response.message=="nocon") document.getElementById("ds").innerHTML="Tidak didapatkan respon dari server, tolong coba lagi sesaat kemudian serta pastikan WiFi anda dapat terkoneksi dengan internet"; else if (response.message=="recon") document.getElementById("ds").innerHTML="Berhasil menghubungkan kembali kontroller dengan akun anda, restart kontroller dengan tombol yang tersedia, lalu kontroller akan dapat dioperasikan secara online di otoma.my.id"; else if (response.message=="used") document.getElementById("ds").innerHTML="Gagal menyambung. Perangkat anda sudah tersambung ke akun lain, mohon hubungkan kontroller ini dengan akun yang terhubung dengan kontroller ini. Apabila anda tidak yakin tolong kontak kami di otoma.my.id"; else if (response.message=="invwifi") document.getElementById("ds").innerHTML="Gagal menyambung ke WiFi, pastikan SSID dan Password WiFi anda benar lalu coba lagi"; else if (response.message=="smwrong") document.getElementById("ds").innerHTML="Terjadi kesalahan saat mengirim data ke server, mohon coba lagi sesaat kemudian";});}; </script></body></html>
)=====";

// CA Certificate of Sectigo
const uint8_t caCert[] = {

0x30, 0x82, 0x05, 0xd8, 0x30, 0x82, 0x03, 0xc0, 0xa0, 0x03, 0x02, 0x01, 
0x02, 0x02, 0x10, 0x4c, 0xaa, 0xf9, 0xca, 0xdb, 0x63, 0x6f, 0xe0, 0x1f, 
0xf7, 0x4e, 0xd8, 0x5b, 0x03, 0x86, 0x9d, 0x30, 0x0d, 0x06, 0x09, 0x2a, 
0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01, 0x0c, 0x05, 0x00, 0x30, 0x81, 
0x85, 0x31, 0x0b, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 
0x47, 0x42, 0x31, 0x1b, 0x30, 0x19, 0x06, 0x03, 0x55, 0x04, 0x08, 0x13, 
0x12, 0x47, 0x72, 0x65, 0x61, 0x74, 0x65, 0x72, 0x20, 0x4d, 0x61, 0x6e, 
0x63, 0x68, 0x65, 0x73, 0x74, 0x65, 0x72, 0x31, 0x10, 0x30, 0x0e, 0x06, 
0x03, 0x55, 0x04, 0x07, 0x13, 0x07, 0x53, 0x61, 0x6c, 0x66, 0x6f, 0x72, 
0x64, 0x31, 0x1a, 0x30, 0x18, 0x06, 0x03, 0x55, 0x04, 0x0a, 0x13, 0x11, 
0x43, 0x4f, 0x4d, 0x4f, 0x44, 0x4f, 0x20, 0x43, 0x41, 0x20, 0x4c, 0x69, 
0x6d, 0x69, 0x74, 0x65, 0x64, 0x31, 0x2b, 0x30, 0x29, 0x06, 0x03, 0x55, 
0x04, 0x03, 0x13, 0x22, 0x43, 0x4f, 0x4d, 0x4f, 0x44, 0x4f, 0x20, 0x52, 
0x53, 0x41, 0x20, 0x43, 0x65, 0x72, 0x74, 0x69, 0x66, 0x69, 0x63, 0x61, 
0x74, 0x69, 0x6f, 0x6e, 0x20, 0x41, 0x75, 0x74, 0x68, 0x6f, 0x72, 0x69, 
0x74, 0x79, 0x30, 0x1e, 0x17, 0x0d, 0x31, 0x30, 0x30, 0x31, 0x31, 0x39, 
0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x5a, 0x17, 0x0d, 0x33, 0x38, 0x30, 
0x31, 0x31, 0x38, 0x32, 0x33, 0x35, 0x39, 0x35, 0x39, 0x5a, 0x30, 0x81, 
0x85, 0x31, 0x0b, 0x30, 0x09, 0x06, 0x03, 0x55, 0x04, 0x06, 0x13, 0x02, 
0x47, 0x42, 0x31, 0x1b, 0x30, 0x19, 0x06, 0x03, 0x55, 0x04, 0x08, 0x13, 
0x12, 0x47, 0x72, 0x65, 0x61, 0x74, 0x65, 0x72, 0x20, 0x4d, 0x61, 0x6e, 
0x63, 0x68, 0x65, 0x73, 0x74, 0x65, 0x72, 0x31, 0x10, 0x30, 0x0e, 0x06, 
0x03, 0x55, 0x04, 0x07, 0x13, 0x07, 0x53, 0x61, 0x6c, 0x66, 0x6f, 0x72, 
0x64, 0x31, 0x1a, 0x30, 0x18, 0x06, 0x03, 0x55, 0x04, 0x0a, 0x13, 0x11, 
0x43, 0x4f, 0x4d, 0x4f, 0x44, 0x4f, 0x20, 0x43, 0x41, 0x20, 0x4c, 0x69, 
0x6d, 0x69, 0x74, 0x65, 0x64, 0x31, 0x2b, 0x30, 0x29, 0x06, 0x03, 0x55, 
0x04, 0x03, 0x13, 0x22, 0x43, 0x4f, 0x4d, 0x4f, 0x44, 0x4f, 0x20, 0x52, 
0x53, 0x41, 0x20, 0x43, 0x65, 0x72, 0x74, 0x69, 0x66, 0x69, 0x63, 0x61, 
0x74, 0x69, 0x6f, 0x6e, 0x20, 0x41, 0x75, 0x74, 0x68, 0x6f, 0x72, 0x69, 
0x74, 0x79, 0x30, 0x82, 0x02, 0x22, 0x30, 0x0d, 0x06, 0x09, 0x2a, 0x86, 
0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01, 0x01, 0x05, 0x00, 0x03, 0x82, 0x02, 
0x0f, 0x00, 0x30, 0x82, 0x02, 0x0a, 0x02, 0x82, 0x02, 0x01, 0x00, 0x91, 
0xe8, 0x54, 0x92, 0xd2, 0x0a, 0x56, 0xb1, 0xac, 0x0d, 0x24, 0xdd, 0xc5, 
0xcf, 0x44, 0x67, 0x74, 0x99, 0x2b, 0x37, 0xa3, 0x7d, 0x23, 0x70, 0x00, 
0x71, 0xbc, 0x53, 0xdf, 0xc4, 0xfa, 0x2a, 0x12, 0x8f, 0x4b, 0x7f, 0x10, 
0x56, 0xbd, 0x9f, 0x70, 0x72, 0xb7, 0x61, 0x7f, 0xc9, 0x4b, 0x0f, 0x17, 
0xa7, 0x3d, 0xe3, 0xb0, 0x04, 0x61, 0xee, 0xff, 0x11, 0x97, 0xc7, 0xf4, 
0x86, 0x3e, 0x0a, 0xfa, 0x3e, 0x5c, 0xf9, 0x93, 0xe6, 0x34, 0x7a, 0xd9, 
0x14, 0x6b, 0xe7, 0x9c, 0xb3, 0x85, 0xa0, 0x82, 0x7a, 0x76, 0xaf, 0x71, 
0x90, 0xd7, 0xec, 0xfd, 0x0d, 0xfa, 0x9c, 0x6c, 0xfa, 0xdf, 0xb0, 0x82, 
0xf4, 0x14, 0x7e, 0xf9, 0xbe, 0xc4, 0xa6, 0x2f, 0x4f, 0x7f, 0x99, 0x7f, 
0xb5, 0xfc, 0x67, 0x43, 0x72, 0xbd, 0x0c, 0x00, 0xd6, 0x89, 0xeb, 0x6b, 
0x2c, 0xd3, 0xed, 0x8f, 0x98, 0x1c, 0x14, 0xab, 0x7e, 0xe5, 0xe3, 0x6e, 
0xfc, 0xd8, 0xa8, 0xe4, 0x92, 0x24, 0xda, 0x43, 0x6b, 0x62, 0xb8, 0x55, 
0xfd, 0xea, 0xc1, 0xbc, 0x6c, 0xb6, 0x8b, 0xf3, 0x0e, 0x8d, 0x9a, 0xe4, 
0x9b, 0x6c, 0x69, 0x99, 0xf8, 0x78, 0x48, 0x30, 0x45, 0xd5, 0xad, 0xe1, 
0x0d, 0x3c, 0x45, 0x60, 0xfc, 0x32, 0x96, 0x51, 0x27, 0xbc, 0x67, 0xc3, 
0xca, 0x2e, 0xb6, 0x6b, 0xea, 0x46, 0xc7, 0xc7, 0x20, 0xa0, 0xb1, 0x1f, 
0x65, 0xde, 0x48, 0x08, 0xba, 0xa4, 0x4e, 0xa9, 0xf2, 0x83, 0x46, 0x37, 
0x84, 0xeb, 0xe8, 0xcc, 0x81, 0x48, 0x43, 0x67, 0x4e, 0x72, 0x2a, 0x9b, 
0x5c, 0xbd, 0x4c, 0x1b, 0x28, 0x8a, 0x5c, 0x22, 0x7b, 0xb4, 0xab, 0x98, 
0xd9, 0xee, 0xe0, 0x51, 0x83, 0xc3, 0x09, 0x46, 0x4e, 0x6d, 0x3e, 0x99, 
0xfa, 0x95, 0x17, 0xda, 0x7c, 0x33, 0x57, 0x41, 0x3c, 0x8d, 0x51, 0xed, 
0x0b, 0xb6, 0x5c, 0xaf, 0x2c, 0x63, 0x1a, 0xdf, 0x57, 0xc8, 0x3f, 0xbc, 
0xe9, 0x5d, 0xc4, 0x9b, 0xaf, 0x45, 0x99, 0xe2, 0xa3, 0x5a, 0x24, 0xb4, 
0xba, 0xa9, 0x56, 0x3d, 0xcf, 0x6f, 0xaa, 0xff, 0x49, 0x58, 0xbe, 0xf0, 
0xa8, 0xff, 0xf4, 0xb8, 0xad, 0xe9, 0x37, 0xfb, 0xba, 0xb8, 0xf4, 0x0b, 
0x3a, 0xf9, 0xe8, 0x43, 0x42, 0x1e, 0x89, 0xd8, 0x84, 0xcb, 0x13, 0xf1, 
0xd9, 0xbb, 0xe1, 0x89, 0x60, 0xb8, 0x8c, 0x28, 0x56, 0xac, 0x14, 0x1d, 
0x9c, 0x0a, 0xe7, 0x71, 0xeb, 0xcf, 0x0e, 0xdd, 0x3d, 0xa9, 0x96, 0xa1, 
0x48, 0xbd, 0x3c, 0xf7, 0xaf, 0xb5, 0x0d, 0x22, 0x4c, 0xc0, 0x11, 0x81, 
0xec, 0x56, 0x3b, 0xf6, 0xd3, 0xa2, 0xe2, 0x5b, 0xb7, 0xb2, 0x04, 0x22, 
0x52, 0x95, 0x80, 0x93, 0x69, 0xe8, 0x8e, 0x4c, 0x65, 0xf1, 0x91, 0x03, 
0x2d, 0x70, 0x74, 0x02, 0xea, 0x8b, 0x67, 0x15, 0x29, 0x69, 0x52, 0x02, 
0xbb, 0xd7, 0xdf, 0x50, 0x6a, 0x55, 0x46, 0xbf, 0xa0, 0xa3, 0x28, 0x61, 
0x7f, 0x70, 0xd0, 0xc3, 0xa2, 0xaa, 0x2c, 0x21, 0xaa, 0x47, 0xce, 0x28, 
0x9c, 0x06, 0x45, 0x76, 0xbf, 0x82, 0x18, 0x27, 0xb4, 0xd5, 0xae, 0xb4, 
0xcb, 0x50, 0xe6, 0x6b, 0xf4, 0x4c, 0x86, 0x71, 0x30, 0xe9, 0xa6, 0xdf, 
0x16, 0x86, 0xe0, 0xd8, 0xff, 0x40, 0xdd, 0xfb, 0xd0, 0x42, 0x88, 0x7f, 
0xa3, 0x33, 0x3a, 0x2e, 0x5c, 0x1e, 0x41, 0x11, 0x81, 0x63, 0xce, 0x18, 
0x71, 0x6b, 0x2b, 0xec, 0xa6, 0x8a, 0xb7, 0x31, 0x5c, 0x3a, 0x6a, 0x47, 
0xe0, 0xc3, 0x79, 0x59, 0xd6, 0x20, 0x1a, 0xaf, 0xf2, 0x6a, 0x98, 0xaa, 
0x72, 0xbc, 0x57, 0x4a, 0xd2, 0x4b, 0x9d, 0xbb, 0x10, 0xfc, 0xb0, 0x4c, 
0x41, 0xe5, 0xed, 0x1d, 0x3d, 0x5e, 0x28, 0x9d, 0x9c, 0xcc, 0xbf, 0xb3, 
0x51, 0xda, 0xa7, 0x47, 0xe5, 0x84, 0x53, 0x02, 0x03, 0x01, 0x00, 0x01, 
0xa3, 0x42, 0x30, 0x40, 0x30, 0x1d, 0x06, 0x03, 0x55, 0x1d, 0x0e, 0x04, 
0x16, 0x04, 0x14, 0xbb, 0xaf, 0x7e, 0x02, 0x3d, 0xfa, 0xa6, 0xf1, 0x3c, 
0x84, 0x8e, 0xad, 0xee, 0x38, 0x98, 0xec, 0xd9, 0x32, 0x32, 0xd4, 0x30, 
0x0e, 0x06, 0x03, 0x55, 0x1d, 0x0f, 0x01, 0x01, 0xff, 0x04, 0x04, 0x03, 
0x02, 0x01, 0x06, 0x30, 0x0f, 0x06, 0x03, 0x55, 0x1d, 0x13, 0x01, 0x01, 
0xff, 0x04, 0x05, 0x30, 0x03, 0x01, 0x01, 0xff, 0x30, 0x0d, 0x06, 0x09, 
0x2a, 0x86, 0x48, 0x86, 0xf7, 0x0d, 0x01, 0x01, 0x0c, 0x05, 0x00, 0x03, 
0x82, 0x02, 0x01, 0x00, 0x0a, 0xf1, 0xd5, 0x46, 0x84, 0xb7, 0xae, 0x51, 
0xbb, 0x6c, 0xb2, 0x4d, 0x41, 0x14, 0x00, 0x93, 0x4c, 0x9c, 0xcb, 0xe5, 
0xc0, 0x54, 0xcf, 0xa0, 0x25, 0x8e, 0x02, 0xf9, 0xfd, 0xb0, 0xa2, 0x0d, 
0xf5, 0x20, 0x98, 0x3c, 0x13, 0x2d, 0xac, 0x56, 0xa2, 0xb0, 0xd6, 0x7e, 
0x11, 0x92, 0xe9, 0x2e, 0xba, 0x9e, 0x2e, 0x9a, 0x72, 0xb1, 0xbd, 0x19, 
0x44, 0x6c, 0x61, 0x35, 0xa2, 0x9a, 0xb4, 0x16, 0x12, 0x69, 0x5a, 0x8c, 
0xe1, 0xd7, 0x3e, 0xa4, 0x1a, 0xe8, 0x2f, 0x03, 0xf4, 0xae, 0x61, 0x1d, 
0x10, 0x1b, 0x2a, 0xa4, 0x8b, 0x7a, 0xc5, 0xfe, 0x05, 0xa6, 0xe1, 0xc0, 
0xd6, 0xc8, 0xfe, 0x9e, 0xae, 0x8f, 0x2b, 0xba, 0x3d, 0x99, 0xf8, 0xd8, 
0x73, 0x09, 0x58, 0x46, 0x6e, 0xa6, 0x9c, 0xf4, 0xd7, 0x27, 0xd3, 0x95, 
0xda, 0x37, 0x83, 0x72, 0x1c, 0xd3, 0x73, 0xe0, 0xa2, 0x47, 0x99, 0x03, 
0x38, 0x5d, 0xd5, 0x49, 0x79, 0x00, 0x29, 0x1c, 0xc7, 0xec, 0x9b, 0x20, 
0x1c, 0x07, 0x24, 0x69, 0x57, 0x78, 0xb2, 0x39, 0xfc, 0x3a, 0x84, 0xa0, 
0xb5, 0x9c, 0x7c, 0x8d, 0xbf, 0x2e, 0x93, 0x62, 0x27, 0xb7, 0x39, 0xda, 
0x17, 0x18, 0xae, 0xbd, 0x3c, 0x09, 0x68, 0xff, 0x84, 0x9b, 0x3c, 0xd5, 
0xd6, 0x0b, 0x03, 0xe3, 0x57, 0x9e, 0x14, 0xf7, 0xd1, 0xeb, 0x4f, 0xc8, 
0xbd, 0x87, 0x23, 0xb7, 0xb6, 0x49, 0x43, 0x79, 0x85, 0x5c, 0xba, 0xeb, 
0x92, 0x0b, 0xa1, 0xc6, 0xe8, 0x68, 0xa8, 0x4c, 0x16, 0xb1, 0x1a, 0x99, 
0x0a, 0xe8, 0x53, 0x2c, 0x92, 0xbb, 0xa1, 0x09, 0x18, 0x75, 0x0c, 0x65, 
0xa8, 0x7b, 0xcb, 0x23, 0xb7, 0x1a, 0xc2, 0x28, 0x85, 0xc3, 0x1b, 0xff, 
0xd0, 0x2b, 0x62, 0xef, 0xa4, 0x7b, 0x09, 0x91, 0x98, 0x67, 0x8c, 0x14, 
0x01, 0xcd, 0x68, 0x06, 0x6a, 0x63, 0x21, 0x75, 0x03, 0x80, 0x88, 0x8a, 
0x6e, 0x81, 0xc6, 0x85, 0xf2, 0xa9, 0xa4, 0x2d, 0xe7, 0xf4, 0xa5, 0x24, 
0x10, 0x47, 0x83, 0xca, 0xcd, 0xf4, 0x8d, 0x79, 0x58, 0xb1, 0x06, 0x9b, 
0xe7, 0x1a, 0x2a, 0xd9, 0x9d, 0x01, 0xd7, 0x94, 0x7d, 0xed, 0x03, 0x4a, 
0xca, 0xf0, 0xdb, 0xe8, 0xa9, 0x01, 0x3e, 0xf5, 0x56, 0x99, 0xc9, 0x1e, 
0x8e, 0x49, 0x3d, 0xbb, 0xe5, 0x09, 0xb9, 0xe0, 0x4f, 0x49, 0x92, 0x3d, 
0x16, 0x82, 0x40, 0xcc, 0xcc, 0x59, 0xc6, 0xe6, 0x3a, 0xed, 0x12, 0x2e, 
0x69, 0x3c, 0x6c, 0x95, 0xb1, 0xfd, 0xaa, 0x1d, 0x7b, 0x7f, 0x86, 0xbe, 
0x1e, 0x0e, 0x32, 0x46, 0xfb, 0xfb, 0x13, 0x8f, 0x75, 0x7f, 0x4c, 0x8b, 
0x4b, 0x46, 0x63, 0xfe, 0x00, 0x34, 0x40, 0x70, 0xc1, 0xc3, 0xb9, 0xa1, 
0xdd, 0xa6, 0x70, 0xe2, 0x04, 0xb3, 0x41, 0xbc, 0xe9, 0x80, 0x91, 0xea, 
0x64, 0x9c, 0x7a, 0xe1, 0x22, 0x03, 0xa9, 0x9c, 0x6e, 0x6f, 0x0e, 0x65, 
0x4f, 0x6c, 0x87, 0x87, 0x5e, 0xf3, 0x6e, 0xa0, 0xf9, 0x75, 0xa5, 0x9b, 
0x40, 0xe8, 0x53, 0xb2, 0x27, 0x9d, 0x4a, 0xb9, 0xc0, 0x77, 0x21, 0x8d, 
0xff, 0x87, 0xf2, 0xde, 0xbc, 0x8c, 0xef, 0x17, 0xdf, 0xb7, 0x49, 0x0b, 
0xd1, 0xf2, 0x6e, 0x30, 0x0b, 0x1a, 0x0e, 0x4e, 0x76, 0xed, 0x11, 0xfc, 
0xf5, 0xe9, 0x56, 0xb2, 0x7d, 0xbf, 0xc7, 0x6d, 0x0a, 0x93, 0x8c, 0xa5, 
0xd0, 0xc0, 0xb6, 0x1d, 0xbe, 0x3a, 0x4e, 0x94, 0xa2, 0xd7, 0x6e, 0x6c, 
0x0b, 0xc2, 0x8a, 0x7c, 0xfa, 0x20, 0xf3, 0xc4, 0xe4, 0xe5, 0xcd, 0x0d, 
0xa8, 0xcb, 0x91, 0x92, 0xb1, 0x7c, 0x85, 0xec, 0xb5, 0x14, 0x69, 0x66, 
0x0e, 0x82, 0xe7, 0xcd, 0xce, 0xc8, 0x2d, 0xa6, 0x51, 0x7f, 0x21, 0xc1, 
0x35, 0x53, 0x85, 0x06, 0x4a, 0x5d, 0x9f, 0xad, 0xbb, 0x1b, 0x5f, 0x74,};

const unsigned int caCertLen = 1500;

//CA CERT
/*-----BEGIN CERTIFICATE-----
MIIGYzCCBUugAwIBAgIQFCKqkR25ovci5EQ70aJoRDANBgkqhkiG9w0BAQsFADBy
MQswCQYDVQQGEwJVUzELMAkGA1UECBMCVFgxEDAOBgNVBAcTB0hvdXN0b24xFTAT
BgNVBAoTDGNQYW5lbCwgSW5jLjEtMCsGA1UEAxMkY1BhbmVsLCBJbmMuIENlcnRp
ZmljYXRpb24gQXV0aG9yaXR5MB4XDTIwMTAyODAwMDAwMFoXDTIxMDEyNjIzNTk1
OVowFjEUMBIGA1UEAxMLb3RvbWEubXkuaWQwggEiMA0GCSqGSIb3DQEBAQUAA4IB
DwAwggEKAoIBAQDh82Yn8VN/fLGzd92mVeKjpXcQilFYdEtjADJtjBnCip8GjXtB
EpnrHGLgmSDXkzAO3ITWDJeAk2wPNzjBbVb4kddVWnfWPJw9VXpVsHOxvRGN6tFT
NaynANAxnmp5WYjQTSFhi36jq2SILbD2/aAw0KoMMMWJO6+4XAxF6Vst+sBZ2on/
FtG5CQ9ZmVTMTdVOYP6SRckKDukcgZPsVUgc6s8DSdCFH0ecu0rcyLsvuhFs4tix
wU67JlFQfw0mq7ao5pThxYVsKME0SLNo8oILZ17IF2ZCujo+TjW7q+U5GW0HNmdN
4hi0cp1xdQr55DYXWOZd5qK37KBiUkIL8axHAgMBAAGjggNPMIIDSzAfBgNVHSME
GDAWgBR+A1plQWunfgrhuJ0I6h2OHWrHZTAdBgNVHQ4EFgQU2cRgbT8oEBZeP4aN
byvAJr8YByIwDgYDVR0PAQH/BAQDAgWgMAwGA1UdEwEB/wQCMAAwHQYDVR0lBBYw
FAYIKwYBBQUHAwEGCCsGAQUFBwMCMEkGA1UdIARCMEAwNAYLKwYBBAGyMQECAjQw
JTAjBggrBgEFBQcCARYXaHR0cHM6Ly9zZWN0aWdvLmNvbS9DUFMwCAYGZ4EMAQIB
MEwGA1UdHwRFMEMwQaA/oD2GO2h0dHA6Ly9jcmwuY29tb2RvY2EuY29tL2NQYW5l
bEluY0NlcnRpZmljYXRpb25BdXRob3JpdHkuY3JsMH0GCCsGAQUFBwEBBHEwbzBH
BggrBgEFBQcwAoY7aHR0cDovL2NydC5jb21vZG9jYS5jb20vY1BhbmVsSW5jQ2Vy
dGlmaWNhdGlvbkF1dGhvcml0eS5jcnQwJAYIKwYBBQUHMAGGGGh0dHA6Ly9vY3Nw
LmNvbW9kb2NhLmNvbTCCAQUGCisGAQQB1nkCBAIEgfYEgfMA8QB2AH0+8viP/4hV
aCTCwMqeUol5K8UOeAl/LmqXaJl+IvDXAAABdW4nutgAAAQDAEcwRQIhAPyoNW/8
KQTh8WfH/h1ShkApgEA6O2+AwU+YHdWlGEIBAiA5c63/jJkgs4AvgFyF5ZQqkWSc
0Hj1gy1QvqER6dZjXwB3AJQgvB6O1Y1siHMfgosiLA3R2k1ebE+UPWHbTi9YTaLC
AAABdW4nu2kAAAQDAEgwRgIhAIVJTe6keYGw3+lwAG6VXZj+OZoryAte6czm490v
zzAwAiEA74R6VD2kCRzj8SnJtbjSAzBlo0P/23yOxVSayPHOffowgaoGA1UdEQSB
ojCBn4ILb3RvbWEubXkuaWSCEmNwYW5lbC5vdG9tYS5teS5pZIIXY3BjYWxlbmRh
cnMub3RvbWEubXkuaWSCFmNwY29udGFjdHMub3RvbWEubXkuaWSCEG1haWwub3Rv
bWEubXkuaWSCE3dlYmRpc2sub3RvbWEubXkuaWSCE3dlYm1haWwub3RvbWEubXku
aWSCD3d3dy5vdG9tYS5teS5pZDANBgkqhkiG9w0BAQsFAAOCAQEATCm5RsfRxOh9
K8znVlQOdf17yvzbAEzWxa/Tghz989rp/qFIslWFnBVHqvd5eZZV8HT28LM5qIPW
QyYT6rNTBH7D0Z0f/0rZ7VaZQqApV7hhP1Elw+zztoWqROHccrh3lvPG7MH1APTN
Hhb0L4fhDcK5m9aAS1l6+Ps2/TdzTxcE+SepEZQ5TvnVGuXOcNbn4KTuZGZyE43S
26LjmI3GMaF7OkBd2qoDLRYwhgARrJSH75Sa4/ayg+h3g+V3dVhMUYPUH744PcZP
rYCENfo+okrxMcg5hQp8TPfNXQ14Ci6/7Wrg7FKBduQ+HgMn8clNt0WgtBNGIsZc
Z/cL5eLZMQ==
-----END CERTIFICATE-----
*/

// PRIVATE KEY
/*
-----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEA4fNmJ/FTf3yxs3fdplXio6V3EIpRWHRLYwAybYwZwoqfBo17
QRKZ6xxi4Jkg15MwDtyE1gyXgJNsDzc4wW1W+JHXVVp31jycPVV6VbBzsb0RjerR
UzWspwDQMZ5qeVmI0E0hYYt+o6tkiC2w9v2gMNCqDDDFiTuvuFwMRelbLfrAWdqJ
/xbRuQkPWZlUzE3VTmD+kkXJCg7pHIGT7FVIHOrPA0nQhR9HnLtK3Mi7L7oRbOLY
scFOuyZRUH8NJqu2qOaU4cWFbCjBNEizaPKCC2deyBdmQro6Pk41u6vlORltBzZn
TeIYtHKdcXUK+eQ2F1jmXeait+ygYlJCC/GsRwIDAQABAoIBABbjoFKliM7eY+YJ
W4PRKX9ocLJVQL5UMZra10Dvs2BYyJlSVc0WxE7j9kRIXuJXv8ORpUdFyUc2J/k2
n+JAYFIX/BdibcdfJni0IwtxL5cuzwtwYTorsszMMUsFgEXAwzJOzfXiurZ701hU
QJwUv1iSmtdXTXuEd12WvYuLgzgaAQ3vgjcyAMVEuBtVudgXht5x3TAemU4mw7nP
2G6aqnrRdHPLS8A55W+6Q9ooxbG28RKk4HNjPnmIkZuh0DvVBTeqhTkT48UccNwE
6pa0sZBP5GWUFnlURs37Wxkd6upeaAl/GzQ8LzR+vUP5nakdMx9u3NV1L+WW3nOU
rGMkP7ECgYEA8N/xUAEqdIPs7cVvqZUXy2a9e8Kmf12rv8wIaxlTt8AOHoedKZRr
ZJeqsdP3ISetyp9Neagqw3E7fJ2wnbDhahoKF6cXqItAEURpVTjqPNBWp9p65v7w
2Rdc3uAnb/Aum1qH3MdE9M4Tm185JSYjQEcn5C0+/V3BXBLgCtJc49kCgYEA8CON
reBM/zAKJdF6ys2ocqKMqLdWbngixVgzqjbfb2AQqGoZy2XclKRKalxG8guSKkjc
oGLxgzMgbIBkZoch59ATjqKh9CFVqLVit+5uSjRqVttkbcfxA5h1DqenUK4p2VoK
ycon8Hkh+g/PLCvKqw881hCmNJyAVCQjcFbmnR8CgYAvFeerFv9AlYVcGWsxgDaN
iUDjD3LSlPH85RqELQhDdCSObT4Yoa24lTlF18SIU6RSZn+Tl378g4b4Xi7nNGtr
/TkZ1Iz4Yngcp/3bLewEBUnbOAcsXougNEPwQWsUI6RItBK44q8lF5+XkK6wB1t+
tiojHYEhsBEbOYlYNDT6SQKBgC88HxMzipGrhmN39/pKR7b8yEQrg+HI2bYNqmPx
1TNYJw6piBddGu1V/5k6O25C3mZY3KTjsXPIK7mL67PLIt0xLZeh389gezPrMdbS
HHmWI9Cf6Po7GOpHI8dVLPEOwQaS0opsrDx95zQDxhF/L6dEIQhOUBMLMabbUOk4
/rWrAoGBANN6fkErYB0GZHdJJP3zHJiJQoTS3BSIPFc729ZDyeFqQMU3Waye3IXW
3fsgr/o+HwrfoGBhoOBcT4x1eaMjKmVzZxIL1So6WY52Fo3+p3yvu0KWQ/bZykaJ
gfdN0ux4gW0Qp5bZjagajJq0M9S/JuCYLuzgM8VsQ7M/EBGmqS8C
-----END RSA PRIVATE KEY-----
*/

// CA BUNDLE
/*
-----BEGIN CERTIFICATE-----
MIIF8TCCA9mgAwIBAgIRAPAdS+57fKN7PAVmrAWXJFgwDQYJKoZIhvcNAQEMBQAwgYUxCzAJBgNV
BAYTAkdCMRswGQYDVQQIExJHcmVhdGVyIE1hbmNoZXN0ZXIxEDAOBgNVBAcTB1NhbGZvcmQxGjAY
BgNVBAoTEUNPTU9ETyBDQSBMaW1pdGVkMSswKQYDVQQDEyJDT01PRE8gUlNBIENlcnRpZmljYXRp
b24gQXV0aG9yaXR5MB4XDTE1MDUxODAwMDAwMFoXDTI1MDUxNzIzNTk1OVowcjELMAkGA1UEBhMC
VVMxCzAJBgNVBAgTAlRYMRAwDgYDVQQHEwdIb3VzdG9uMRUwEwYDVQQKEwxjUGFuZWwsIEluYy4x
LTArBgNVBAMTJGNQYW5lbCwgSW5jLiBDZXJ0aWZpY2F0aW9uIEF1dGhvcml0eTCCASIwDQYJKoZI
hvcNAQEBBQADggEPADCCAQoCggEBAIteAVa57GsR70jpQ56byLpTkaW9qyr6Xjo14Q1cNepSqJk0
KA9+WStIa+e010t9L4PP/osmw1l5H2Chaaday583Ie8YvZv9Qet1fLeW2V6GyyoS4qf3A+TO5gX3
QZsevNL20WZpUQzete08CyfPiI4gPeNOlY8VNMYmy/c/ZOn1MCV9zak5mz/qemkri8R9C/hWk7Zr
lsrsz9J7vUO+0/WJ2k10SSHEvfUwvLxJqWUVs9b/vx2QlJwIJbatz/zH2ftV1RnQSr9iRuUk7Y++
ZJgMalGeeoBzIKm02b9Dap4QrSugzWStQDnS4rjbwvI6o+K3FpcfHvbP3zweWOkAB2sCAwEAAaOC
AWwwggFoMB8GA1UdIwQYMBaAFLuvfgI9+qbxPISOre44mOzZMjLUMB0GA1UdDgQWBBR+A1plQWun
fgrhuJ0I6h2OHWrHZTAOBgNVHQ8BAf8EBAMCAYYwEgYDVR0TAQH/BAgwBgEB/wIBADAdBgNVHSUE
FjAUBggrBgEFBQcDAQYIKwYBBQUHAwIwIgYDVR0gBBswGTANBgsrBgEEAbIxAQICNDAIBgZngQwB
AgEwTAYDVR0fBEUwQzBBoD+gPYY7aHR0cDovL2NybC5jb21vZG9jYS5jb20vQ09NT0RPUlNBQ2Vy
dGlmaWNhdGlvbkF1dGhvcml0eS5jcmwwcQYIKwYBBQUHAQEEZTBjMDsGCCsGAQUFBzAChi9odHRw
Oi8vY3J0LmNvbW9kb2NhLmNvbS9DT01PRE9SU0FBZGRUcnVzdENBLmNydDAkBggrBgEFBQcwAYYY
aHR0cDovL29jc3AuY29tb2RvY2EuY29tMA0GCSqGSIb3DQEBDAUAA4ICAQAQn6BgCIF0oaCEeGBM
OTnaZHfvGQpyOSOUO5F9fzSLl1hOWQotaMMQQrCgeoGMe6sxMiA55CJz4N7JF12DxXUt4RFHWQGe
XcD03RJq0G0wIOizyk/fmuCnF58aL4d+61DhU/P4R9mMYPLJZWWc8NoB5rLy2AeYh983iZhVEkLJ
5C3eLb6qZJRO2S7mwtXywObp6hk+NwuJX8k6+E9HQD6vGn+i9oUBiBc2tSPquf66a0gLAiA5rsNh
65WloXPHHF9UM3NXSzaLm1so4z6xC3hcaxSnEMzl2j+66dayLR1wVLpeq31PKYkQ4DqQBMXuuY5D
ouNjWH9Ji3E+V2IjQNFdlmQiYVaflmdHh7zlACCkaOLBoIF7aHMIxG1OcHno3VXXCVy5nQqVpgzZ
2+KKVeu54eealRRMWAZBwRCqqrE64qVKSuDZyR/CoJe7Bu8ZANsCvpbx+1SPk5r6MCI2qXcmH5Qo
k+kTPUXROjVIHpgNgnDAC1ooh6F4UT+1p1ymkSIAQky5gBWAKrEtiU/3uh4YxIxZHnNJo6h7vB/3
Vk1Qn2cWp8cXSOdtVFd2bpdYW3hkpO1itAA7Bn55uFhfboTWQ7xP2zmqKPDBiQnF++MYRLflsotd
lfkjWgty92k61leL4en0YL7EUSsRrP5Is3JzyhNQcw0EdsoB4ULC1yHP+Q==
-----END CERTIFICATE-----
-----BEGIN CERTIFICATE-----
MIIFfjCCBGagAwIBAgIQZ970PvF72uJP9ZQGBtLAhDANBgkqhkiG9w0BAQwFADB7MQswCQYDVQQG
EwJHQjEbMBkGA1UECAwSR3JlYXRlciBNYW5jaGVzdGVyMRAwDgYDVQQHDAdTYWxmb3JkMRowGAYD
VQQKDBFDb21vZG8gQ0EgTGltaXRlZDEhMB8GA1UEAwwYQUFBIENlcnRpZmljYXRlIFNlcnZpY2Vz
MB4XDTA0MDEwMTAwMDAwMFoXDTI4MTIzMTIzNTk1OVowgYUxCzAJBgNVBAYTAkdCMRswGQYDVQQI
ExJHcmVhdGVyIE1hbmNoZXN0ZXIxEDAOBgNVBAcTB1NhbGZvcmQxGjAYBgNVBAoTEUNPTU9ETyBD
QSBMaW1pdGVkMSswKQYDVQQDEyJDT01PRE8gUlNBIENlcnRpZmljYXRpb24gQXV0aG9yaXR5MIIC
IjANBgkqhkiG9w0BAQEFAAOCAg8AMIICCgKCAgEAkehUktIKVrGsDSTdxc9EZ3SZKzejfSNwAHG8
U9/E+ioSj0t/EFa9n3Byt2F/yUsPF6c947AEYe7/EZfH9IY+Cvo+XPmT5jR62RRr55yzhaCCenav
cZDX7P0N+pxs+t+wgvQUfvm+xKYvT3+Zf7X8Z0NyvQwA1onrayzT7Y+YHBSrfuXjbvzYqOSSJNpD
a2K4Vf3qwbxstovzDo2a5JtsaZn4eEgwRdWt4Q08RWD8MpZRJ7xnw8outmvqRsfHIKCxH2XeSAi6
pE6p8oNGN4Tr6MyBSENnTnIqm1y9TBsoilwie7SrmNnu4FGDwwlGTm0+mfqVF9p8M1dBPI1R7Qu2
XK8sYxrfV8g/vOldxJuvRZnio1oktLqpVj3Pb6r/SVi+8Kj/9Lit6Tf7urj0Czr56ENCHonYhMsT
8dm74YlguIwoVqwUHZwK53Hrzw7dPamWoUi9PPevtQ0iTMARgexWO/bTouJbt7IEIlKVgJNp6I5M
ZfGRAy1wdALqi2cVKWlSArvX31BqVUa/oKMoYX9w0MOiqiwhqkfOKJwGRXa/ghgntNWutMtQ5mv0
TIZxMOmm3xaG4Nj/QN370EKIf6MzOi5cHkERgWPOGHFrK+ymircxXDpqR+DDeVnWIBqv8mqYqnK8
V0rSS527EPywTEHl7R09XiidnMy/s1Hap0flhFMCAwEAAaOB8jCB7zAfBgNVHSMEGDAWgBSgEQoj
PpbxB+zirynvgqV/0DCktDAdBgNVHQ4EFgQUu69+Aj36pvE8hI6t7jiY7NkyMtQwDgYDVR0PAQH/
BAQDAgGGMA8GA1UdEwEB/wQFMAMBAf8wEQYDVR0gBAowCDAGBgRVHSAAMEMGA1UdHwQ8MDowOKA2
oDSGMmh0dHA6Ly9jcmwuY29tb2RvY2EuY29tL0FBQUNlcnRpZmljYXRlU2VydmljZXMuY3JsMDQG
CCsGAQUFBwEBBCgwJjAkBggrBgEFBQcwAYYYaHR0cDovL29jc3AuY29tb2RvY2EuY29tMA0GCSqG
SIb3DQEBDAUAA4IBAQB/8lY1sG2VSk50rzribwGLh9Myl+34QNJ3UxHXxxYuxp3mSFa+gKn4vHjS
yGMXroztFjH6HxjJDsfuSHmfx8m5vMyIFeNoYdGfHUthgddWBGPCCGkm8PDlL9/ACiupBfQCWmqJ
17SEQpXj6/d2IF412cDNJQgTTHE4joewM4SRmR6R8ayeP6cdYIEsNkFUoOJGBgusG8eZNoxeoQuk
ntlCRiTFxVuBrq2goNyfNriNwh0V+oitgRA5H0TwK5/dEFQMBzSxNtEU/QcCPf9yVasn1iyBQXEp
jUH0UFcafmVgr8vFKHaYrrOoU3aL5iFSa+oh0IQOSU6IU9qSLucdCGbX
-----END CERTIFICATE-----
*/
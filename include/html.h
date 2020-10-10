// This is the minified file of html document to reduce flash usage
// minified is 5374 byte
// unminified is 7415 byte, which is 27.5% saving!
// Original file is testssid.php on xampp htdocs
static const char htmlDoc[] PROGMEM = R"=====(<!DOCTYPE html>
<html lang="en">

<head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>Otoma Nexus Controller</title>
    <style>
        :root {
            --normalcolor: #011627;
            --failcolor: #f51000;
            --successcolor: #00ad17;
            --main-bg-color: #e4dfda;
            --second-color: #6a3937;
            --third-color: #011627;
            --fourth-color: #9d79bc;
            --fifth-color: #2374ab;
        }

        html {
            font-family: Helvetica, sans-serif;
            display: inline-block;
            text-align: center;
            color: var(--third-color);
        }

        body {
            margin-top: 50px;
            margin-left: auto;
            background-color: var(--main-bg-color);
        }

        h1 {
            columns: #444444;
            margin: 50px auto 30px;
        }

        h3 {
            color: #444444;
            margin-bottom: 50px;
        }

        .isian {
            width: 40%;
            height: 24px;
            margin-right: 2%;
        }

        .tombolsubmit {
            margin-top: 2%;
            margin-left: -2%;
            width: 84%;
        }
    </style>
</head>

<body>
    <h2 id="hd">Masukkan informasi</h2>
    <div style="width: 100vw; height: 100px; margin-bottom:25px;">
        <input type="text" placeholder="SSID/Nama WiFi" class="isian" id="ssid" />
        <input type="text" placeholder="Username Akun" class="isian" id="usrn" />
        <br /><br />
        <input type="password" placeholder="Password WiFi" class="isian" id="wfpw" />
        <input type="password" placeholder="Password akun" class="isian" id="unpw" />
        <br>
        <input type="checkbox" id="spw" name="spw" value="show" onclick="showPassword();">
        <label for="spw"> Tampilkan Password</label>
        <br>
        <button class="tombolsubmit" onclick="sendInputInfo();">Submit</button>
    </div>
    <br /><br>
    <span style="color: var(--normalcolor);" id="ds"></span>
    <br>
    <br>
    <button onclick="restart();">Restart Kontroller</button>
    <br>
    <h3 style="background-color:red;" id="tm">
        Belum termuat
    </h3>
    <script>
        function requestAJAX(url, data, callback = function() {}) {
            var xhr = new XMLHttpRequest();
            xhr.open("POST", url);
            xhr.setRequestHeader("Content-type", "application/x-www-form-urlencoded");
            xhr.send(data);
            xhr.onload = function() {
                if (xhr.status == 200 && xhr.readyState == 4) {
                    callback(xhr.responseText);
                }
            };
        }

        function sendInputInfo() {
            requestAJAX("accInfo", "ssid=" + document.getElementById("ssid").value + "&wfpw=" + document.getElementById("wfpw").value + "&usrn=" + document.getElementById("usrn").value + "&unpw=" + document.getElementById("unpw").value);
        }

        function restart() {
            document.getElementById("ds").innerHTML = "Merestart perangkat...";
            requestAJAX("restart", "sure=yes");
        }

        function showPassword() {
            if (document.getElementById("spw").checked) {
                document.getElementById("unpw").type = "text";
                document.getElementById("wfpw").type = "text";
            } else {
                document.getElementById("unpw").type = "password";
                document.getElementById("wfpw").type = "password";
            }
        }

        window.onload = function() {
            requestAJAX("reqStatus", "", function(response) {
                response = JSON.parse(response);
                document.getElementById("ssid").value = response.ssid;
                document.getElementById("wfpw").value = response.wfpw;
                document.getElementById("usrn").value = response.usrn;
                document.getElementById("unpw").value = response.unpw;
                document.getElementById("tm").style.backgroundColor = "green";
                document.getElementById("tm").innerHTML = "Berhasil termuat";
                if (response.message == "success")
                    document.getElementById("ds").innerHTML = "Berhasil menghubungkan akun anda dengan kontroller<br>restart kontroller dengan tombol yang tersedia, lalu kontroller akan dapat dioperasikan secara online di otomaiot.com";
                else if (response.message == "wrongid")
                    document.getElementById("ds").innerHTML = "Username yang anda masukkan tidak terdaftar";
                else if (response.message == "wrongpw")
                    document.getElementById("ds").innerHTML = "Password yang anda masukkan salah";
                else if (response.message == "illegal")
                    document.getElementById("ds").innerHTML = "Kontroller anda ILLEGAL, apabila anda tidak yakin tolong hubungi kami di otomaiot.com";
                else if (response.message == "nocon")
                    document.getElementById("ds").innerHTML = "Tidak didapatkan respon dari server, tolong coba lagi sesaat kemudian serta pastikan WiFi anda dapat terkoneksi dengan internet";
                else if (response.message == "recon")
                    document.getElementById("ds").innerHTML = "Berhasil menghubungkan kembali kontroller dengan akun anda, restart kontroller dengan tombol yang tersedia, lalu kontroller akan dapat dioperasikan secara online di otomaiot.com";
                else if (response.message == "used")
                    document.getElementById("ds").innerHTML = "Gagal menyambung. Perangkat anda sudah tersambung ke akun lain, mohon hubungkan kontroller dengan akun yang anda gunakan sebelumnya. Apabila anda tidak yakin tolong kontak kami di otomaiot.com";
                else if (response.message == "invwifi")
                    document.getElementById("ds").innerHTML = "Gagal menyambung ke WiFi, pastikan SSID dan Password WiFi anda benar lalu coba lagi";
            });
        };
    </script>
</body>
</html>
)=====";
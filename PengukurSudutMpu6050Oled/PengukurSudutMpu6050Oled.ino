// Pengukur Sudut Kemiringan — ESP32 + MPU6050 + OLED SSD1306 128x64 + Buzzer
//
// Alat kecil pengukur sudut kemiringan: modul MPU6050 membaca percepatan
// dan kecepatan sudut, ESP32 menggabungkan keduanya dengan complementary
// filter jadi sudut pitch dan roll, lalu menampilkannya di OLED 128x64
// lengkap dengan status (DATAR / MIRING / TERLALU MIRING). Kalau sudut
// melewati ambang, buzzer berbunyi — tandanya benda sudah terlalu miring.
//
// Angka datasheet MPU6050 (InvenSense, terverifikasi 26 Nov 2026):
//   Sensor            : akselerometer 3 sumbu + giroskop 3 sumbu (6-axis IMU)
//   Alamat I2C        : 0x68 (pin AD0 ke GND)
//   Akselerometer     : rentang default +-2 g, 16384 LSB per g (16-bit ADC)
//   Giroskop          : rentang default +-250 derajat/detik,
//                       131 LSB per derajat/detik (16-bit ADC)
//   Register baca     : 0x3B (ACCEL_XOUT_H) sampai 0x48, 14 byte sekaligus
//   PWR_MGMT_1 (0x6B) : tulis 0 untuk membangunkan chip dari sleep
// Sudut dari akselerometer saja berisik saat bergerak, sudut dari giroskop
// saja melayang (drift) karena integrasi. Complementary filter menggabungkan
// keduanya: 96% giroskop (halus, jangka pendek) + 4% akselerometer
// (patokan gravitasi, jangka panjang) — hasilnya stabil sampai 0,1 derajat.
// Pitch dihitung dari sumbu X, roll dari sumbu Y; rentang +-90 derajat.
//
// Wiring:
//   MPU6050 VCC -> ESP32 3V3
//   MPU6050 GND -> ESP32 GND
//   MPU6050 SDA -> ESP32 GPIO21 (SDA I2C)
//   MPU6050 SCL -> ESP32 GPIO22 (SCL I2C)
//   OLED SSD1306 128x64 (0x3C): VCC -> ESP32 3V3, GND -> GND,
//     SDA -> GPIO21, SCL -> GPIO22 (satu bus I2C dengan MPU6050)
//   Buzzer + -> ESP32 GPIO25 (aktif HIGH)
//   Buzzer - -> ESP32 GND
//   Daya: kabel USB 5 V ke port USB ESP32
//
// Library yang dibutuhkan (Arduino IDE / arduino-cli):
//   Adafruit GFX Library 1.13.0, Adafruit SSD1306 2.5.17
//   (MPU6050 TIDAK butuh library eksternal — dibaca langsung lewat Wire.h)
//
// Compile:
//   arduino-cli compile --fqbn esp32:esp32:esp32 PengukurSudutMpu6050Oled
//
// Skema + repo: https://github.com/oerban70-cloud/pengukur-sudut-mpu6050-oled

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Konfigurasi — semua angka yang bisa diubah ada di sini, bukan di tengah kode
// ---------------------------------------------------------------------------

// Nyalakan buzzer peringatan atau tidak (false = tanpa buzzer)
#define USE_BUZZER true

// Pin bus I2C (pin default ESP32 DevKit)
const uint8_t PIN_I2C_SDA = 21;
const uint8_t PIN_I2C_SCL = 22;

// Pin buzzer aktif (HIGH = bunyi)
const uint8_t PIN_BUZZER = 25;

// Alamat I2C
const uint8_t ADDR_MPU6050 = 0x68;  // pin AD0 modul ke GND
const uint8_t ADDR_OLED    = 0x3C;  // alamat I2C SSD1306

// Ukuran OLED
const uint8_t LEBAR_OLED  = 128;  // lebar layar (piksel)
const uint8_t TINGGI_OLED = 64;   // tinggi layar (piksel)
const int8_t  RESET_OLED  = -1;   // pin reset tidak dipakai

// --- Register MPU6050 (datasheet) ---
const uint8_t REG_PWR_MGMT_1   = 0x6B;  // tulis 0 = bangun dari sleep
const uint8_t REG_ACCEL_XOUT_H = 0x3B;  // awal blok baca 14 byte
const uint8_t JUMLAH_BYTE_BACA = 14;    // accel(6) + suhu(2) + gyro(6)

// --- Skala sensor (datasheet, rentang default) ---
// Akselerometer +-2 g: 16384 LSB per g.
// Giroskop +-250 derajat/detik: 131 LSB per derajat/detik.
const float LSB_PER_G_MPU6050        = 16384.0;
const float LSB_PER_DERAJAT_PER_DETIK = 131.0;

// --- Complementary filter ---
// 0,96 = percaya giroskop (halus jangka pendek),
// 0,04 = koreksi dari akselerometer (patokan gravitasi jangka panjang).
const float BOBOT_GIRO_FILTER = 0.96;

// --- Ambang sudut (derajat) ---
// Di bawah ambang ini benda masih dianggap datar.
const float AMBANG_DATAR_DERAJAT = 5.0;
// Di atas ambang ini benda terlalu miring: buzzer bunyi.
const float AMBANG_ALARM_DERAJAT = 45.0;

// Seberapa sering sensor dibaca dan OLED diperbarui (milidetik)
const unsigned long INTERVAL_BACA_MS = 100;  // 10 kali per detik

// Durasi bunyi buzzer tiap peringatan (milidetik)
const unsigned long DURASI_BUNYI_MS = 200;

// Jumlah sampel kalibrasi giroskop saat dinyalakan (modul HARUS diam)
const uint16_t JUMLAH_SAMPEL_KALIBRASI = 200;

// ---------------------------------------------------------------------------
// Objek global
// ---------------------------------------------------------------------------

Adafruit_SSD1306 oled(LEBAR_OLED, TINGGI_OLED, &Wire, RESET_OLED);

// Offset giroskop hasil kalibrasi (derajat/detik)
float offsetGiroX = 0.0;
float offsetGiroY = 0.0;
float offsetGiroZ = 0.0;

// Sudut hasil complementary filter (derajat)
float sudutPitch = 0.0;  // miring maju-mundur (sumbu X)
float sudutRoll  = 0.0;  // miring kiri-kanan (sumbu Y)

unsigned long waktuBacaTerakhirMs = 0;
unsigned long waktuLoopTerakhirMs = 0;

// ---------------------------------------------------------------------------
// Tulis satu byte ke register MPU6050
// ---------------------------------------------------------------------------
void tulisRegisterMpu6050(uint8_t alamatRegister, uint8_t nilai) {
  Wire.beginTransmission(ADDR_MPU6050);
  Wire.write(alamatRegister);
  Wire.write(nilai);
  Wire.endTransmission();
}

// ---------------------------------------------------------------------------
// Baca akselerometer + giroskop dari MPU6050 (14 byte dari register 0x3B).
// Mengisi ax, ay, az (dalam g) dan gx, gy, gz (dalam derajat/detik).
// Mengembalikan false bila MPU6050 tidak menjawab.
// ---------------------------------------------------------------------------
bool bacaMpu6050(float &ax, float &ay, float &az,
                 float &gx, float &gy, float &gz) {
  Wire.beginTransmission(ADDR_MPU6050);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) {
    return false;  // MPU6050 tidak menjawab
  }
  Wire.requestFrom(ADDR_MPU6050, JUMLAH_BYTE_BACA);
  if (Wire.available() < JUMLAH_BYTE_BACA) {
    return false;  // data tidak lengkap
  }
  // Urutan byte: accel X, accel Y, accel Z, suhu, gyro X, gyro Y, gyro Z.
  int16_t mentahAx = (Wire.read() << 8) | Wire.read();
  int16_t mentahAy = (Wire.read() << 8) | Wire.read();
  int16_t mentahAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // suhu: dibuang, tidak dipakai
  int16_t mentahGx = (Wire.read() << 8) | Wire.read();
  int16_t mentahGy = (Wire.read() << 8) | Wire.read();
  int16_t mentahGz = (Wire.read() << 8) | Wire.read();

  ax = mentahAx / LSB_PER_G_MPU6050;
  ay = mentahAy / LSB_PER_G_MPU6050;
  az = mentahAz / LSB_PER_G_MPU6050;
  gx = mentahGx / LSB_PER_DERAJAT_PER_DETIK;
  gy = mentahGy / LSB_PER_DERAJAT_PER_DETIK;
  gz = mentahGz / LSB_PER_DERAJAT_PER_DETIK;
  return true;
}

// ---------------------------------------------------------------------------
// Kalibrasi giroskop: rata-rata 200 bacaan saat modul diam.
// WAJIB: jangan gerakkan modul selama kalibrasi (sekitar 0,2 detik).
// ---------------------------------------------------------------------------
void kalibrasiGiroskop() {
  float jumlahGx = 0.0;
  float jumlahGy = 0.0;
  float jumlahGz = 0.0;
  float ax, ay, az, gx, gy, gz;
  for (uint16_t i = 0; i < JUMLAH_SAMPEL_KALIBRASI; i++) {
    if (bacaMpu6050(ax, ay, az, gx, gy, gz)) {
      jumlahGx += gx;
      jumlahGy += gy;
      jumlahGz += gz;
    }
    delay(1);
  }
  offsetGiroX = jumlahGx / JUMLAH_SAMPEL_KALIBRASI;
  offsetGiroY = jumlahGy / JUMLAH_SAMPEL_KALIBRASI;
  offsetGiroZ = jumlahGz / JUMLAH_SAMPEL_KALIBRASI;
}

// ---------------------------------------------------------------------------
// Sudut dari akselerometer (patokan gravitasi), dalam derajat.
// Pitch = miring maju-mundur, roll = miring kiri-kanan.
// ---------------------------------------------------------------------------
void sudutDariAkselerometer(float ax, float ay, float az,
                            float &pitchAccel, float &rollAccel) {
  pitchAccel = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  rollAccel  = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;
}

// ---------------------------------------------------------------------------
// Perbarui sudut dengan complementary filter.
// dtDetik = selang waktu sejak pembaruan sebelumnya (detik).
// ---------------------------------------------------------------------------
void perbaruiSudut(float ax, float ay, float az,
                   float gx, float gy, float gz, float dtDetik) {
  float pitchAccel, rollAccel;
  sudutDariAkselerometer(ax, ay, az, pitchAccel, rollAccel);
  // Integrasi giroskop (dikurangi offset kalibrasi), lalu koreksi pelan
  // dengan sudut akselerometer supaya tidak melayang.
  sudutPitch = BOBOT_GIRO_FILTER * (sudutPitch + (gx - offsetGiroX) * dtDetik)
              + (1.0 - BOBOT_GIRO_FILTER) * pitchAccel;
  sudutRoll  = BOBOT_GIRO_FILTER * (sudutRoll + (gy - offsetGiroY) * dtDetik)
              + (1.0 - BOBOT_GIRO_FILTER) * rollAccel;
}

// ---------------------------------------------------------------------------
// Status sudut: 0 = DATAR, 1 = MIRING, 2 = TERLALU MIRING (alarm)
// ---------------------------------------------------------------------------
uint8_t statusSudut(float sudutTerbesar) {
  if (sudutTerbesar > AMBANG_ALARM_DERAJAT) return 2;
  if (sudutTerbesar > AMBANG_DATAR_DERAJAT) return 1;
  return 0;
}

const char* teksStatus(uint8_t status) {
  if (status == 2) return "TERLALU MIRING!";
  if (status == 1) return "MIRING";
  return "DATAR";
}

// ---------------------------------------------------------------------------
// Tampilkan pitch, roll, dan status di OLED.
// ---------------------------------------------------------------------------
void tampilkanOled(float sudutTerbesar, uint8_t status) {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println("SUDUT KEMIRINGAN");

  // Sudut terbesar (pitch atau roll) tampil besar di tengah.
  oled.setTextSize(3);
  oled.setCursor(0, 16);
  oled.print(sudutTerbesar, 1);
  oled.print((char)247);  // simbol derajat
  oled.setTextSize(1);
  oled.setCursor(0, 52);
  oled.print("Pitch ");
  oled.print(sudutPitch, 1);
  oled.print((char)247);
  oled.print("  Roll ");
  oled.print(sudutRoll, 1);
  oled.print((char)247);

  // Status di baris bawah, terbalik (putih di atas hitam) bila alarm.
  if (status == 2) {
    oled.fillRect(0, 42, 128, 9, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
    oled.setCursor(4, 43);
    oled.print(teksStatus(status));
    oled.setTextColor(SSD1306_WHITE);
  } else {
    oled.setCursor(4, 43);
    oled.print(teksStatus(status));
  }

  oled.display();
}

// ---------------------------------------------------------------------------
// Bunyikan buzzer sekali (200 ms) bila status alarm.
// ---------------------------------------------------------------------------
void bunyikanBuzzerBilaAlarm(uint8_t status) {
  if (!USE_BUZZER || status != 2) {
    return;
  }
  digitalWrite(PIN_BUZZER, HIGH);
  delay(DURASI_BUNYI_MS);
  digitalWrite(PIN_BUZZER, LOW);
}

// ---------------------------------------------------------------------------
// Tampilkan pesan error di OLED + Serial bila MPU6050 tidak ditemukan.
// ---------------------------------------------------------------------------
void tampilkanErrorMpu6050() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("MPU6050 TIDAK");
  oled.println("TERDETEKSI!");
  oled.println("");
  oled.println("Cek kabel SDA/SCL");
  oled.println("dan alamat 0x68.");
  oled.display();
  Serial.println("MPU6050 tidak terdeteksi di alamat 0x68.");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // Bangunkan MPU6050 dari sleep (register PWR_MGMT_1 = 0).
  tulisRegisterMpu6050(REG_PWR_MGMT_1, 0);
  delay(100);

  // Nyalakan OLED.
  if (!oled.begin(SSD1306_SWITCHCAPVCC, ADDR_OLED)) {
    Serial.println("OLED SSD1306 tidak terdeteksi di alamat 0x3C.");
    while (true) {
      delay(1000);  // OLED mati: tidak ada gunanya lanjut
    }
  }
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("Kalibrasi giro...");
  oled.println("JANGAN GERAKKAN!");
  oled.display();

  // Kalibrasi giroskop — modul harus diam.
  kalibrasiGiroskop();

  // Sudut awal dari akselerometer (modul diam = patokan awal yang benar).
  float ax, ay, az, gx, gy, gz;
  if (bacaMpu6050(ax, ay, az, gx, gy, gz)) {
    sudutDariAkselerometer(ax, ay, az, sudutPitch, sudutRoll);
  }

  waktuLoopTerakhirMs = millis();
  waktuBacaTerakhirMs = millis();
  Serial.println("Pengukur sudut siap. Letakkan di benda yang diukur.");
}

void loop() {
  unsigned long sekarangMs = millis();
  if (sekarangMs - waktuBacaTerakhirMs < INTERVAL_BACA_MS) {
    return;  // belum waktunya baca lagi
  }
  float dtDetik = (sekarangMs - waktuLoopTerakhirMs) / 1000.0;
  waktuLoopTerakhirMs = sekarangMs;
  waktuBacaTerakhirMs = sekarangMs;

  float ax, ay, az, gx, gy, gz;
  if (!bacaMpu6050(ax, ay, az, gx, gy, gz)) {
    tampilkanErrorMpu6050();
    return;
  }

  perbaruiSudut(ax, ay, az, gx, gy, gz, dtDetik);

  float sudutTerbesar = fabs(sudutPitch);
  if (fabs(sudutRoll) > sudutTerbesar) {
    sudutTerbesar = fabs(sudutRoll);
  }
  uint8_t status = statusSudut(sudutTerbesar);

  tampilkanOled(sudutTerbesar, status);
  bunyikanBuzzerBilaAlarm(status);

  Serial.print("Pitch ");
  Serial.print(sudutPitch, 1);
  Serial.print("  Roll ");
  Serial.print(sudutRoll, 1);
  Serial.print("  Status ");
  Serial.println(teksStatus(status));
}

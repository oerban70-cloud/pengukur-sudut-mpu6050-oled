# Pengukur Sudut Kemiringan — ESP32 + MPU6050 + OLED SSD1306

Rak bukumu miring, tapi seberapa miringnya? Mata tidak bisa memastikan
sudutnya — alat kecil ini bisa. Modul MPU6050 membaca kemiringan sampai
90 derajat dengan ketelitian 0,1 derajat, ESP32 menggabungkan data
akselerometer dan giroskop dengan complementary filter, lalu menampilkan
sudutnya di OLED 128x64 lengkap dengan status DATAR, MIRING, atau
TERLALU MIRING. Kalau miringnya lewat 45 derajat, buzzer langsung bunyi.

Video penjelasan (60 detik):
link menyusul setelah upload

## Komponen

| Bagian | Keterangan |
|---|---|
| ESP32 DevKit | Otak alat |
| MPU6050 | Modul IMU 6-axis (akselerometer + giroskop 3 sumbu), alamat I2C 0x68 |
| OLED SSD1306 128x64 | Tampil sudut + status (bus I2C, alamat 0x3C) |
| Buzzer aktif | Bunyi kalau sudut lewat 45 derajat (GPIO25) |
| Adaptor USB 5V 2A | Catu daya ESP32 |

## Wiring

| Dari | Ke |
|---|---|
| MPU6050 VCC | ESP32 3V3 |
| MPU6050 GND | ESP32 GND |
| MPU6050 SDA | ESP32 GPIO21 (SDA I2C) |
| MPU6050 SCL | ESP32 GPIO22 (SCL I2C) |
| OLED VCC | ESP32 3V3 |
| OLED GND | ESP32 GND |
| OLED SDA / SCL | ESP32 GPIO21 / GPIO22 (satu bus I2C dengan MPU6050) |
| Buzzer + | ESP32 GPIO25 (aktif HIGH) |
| Buzzer - | ESP32 GND |

Skema lengkap: [schematic.png](schematic.png)

## Cara kerja

1. MPU6050 dibaca langsung lewat Wire.h — tanpa library MPU6050. Sketch
   membaca 14 byte dari register 0x3B: akselerometer X/Y/Z, suhu (dibuang),
   dan giroskop X/Y/Z. Chip dibangunkan dulu dengan menulis 0 ke register
   PWR_MGMT_1 (0x6B).
2. Sudut dari akselerometer dihitung dari arah gravitasi
   (pitch = atan2(ax, akar(ay²+az²)), roll = atan2(ay, akar(ax²+az²))).
   Sudut dari giroskop didapat dengan mengintegrasi kecepatan sudut.
   Keduanya digabung dengan complementary filter: 96% giroskop (halus
   jangka pendek) + 4% akselerometer (patokan gravitasi jangka panjang) —
   hasilnya stabil sampai 0,1 derajat tanpa melayang (drift).
3. Saat dinyalakan, sketch mengkalibrasi giroskop dengan merata-rata 200
   bacaan — modul HARUS diam selama kalibrasi (sekitar 0,2 detik).
4. ESP32 membaca sensor dan memperbarui OLED tiap 100 milidetik (10 kali
   per detik). OLED menampilkan sudut terbesar (pitch atau roll) besar di
   tengah, nilai pitch dan roll di bawahnya, dan status: DATAR di bawah
   5 derajat, MIRING di atas 5 derajat, TERLALU MIRING di atas 45 derajat.
5. Kalau status TERLALU MIRING, buzzer aktif berbunyi 200 milidetik tiap
   pembacaan. Matikan dengan mengubah `USE_BUZZER` jadi false.
6. Kalau MPU6050 tidak menjawab di alamat 0x68, OLED menampilkan pesan
   "MPU6050 TIDAK TERDETEKSI!" — cek kabel SDA/SCL dan alamatnya.

## Angka acuan

- MPU6050 (InvenSense): akselerometer 3 sumbu + giroskop 3 sumbu,
  alamat I2C 0x68 (pin AD0 ke GND).
- Akselerometer rentang default +-2 g: 16384 LSB per g (ADC 16-bit).
- Giroskop rentang default +-250 derajat/detik: 131 LSB per derajat/detik.
- Pitch dan roll dari akselerometer: rentang +-90 derajat.
- Complementary filter: bobot giroskop 0,96, bobot akselerometer 0,04.

## Konfigurasi (di awal sketch)

- `USE_BUZZER` (true) — nyalakan/matikan buzzer peringatan.
- `PIN_I2C_SDA` (21) / `PIN_I2C_SCL` (22) — pin bus I2C (dipakai bersama
  MPU6050 dan OLED).
- `PIN_BUZZER` (25) — pin buzzer aktif.
- `ADDR_MPU6050` (0x68) / `ADDR_OLED` (0x3C) — alamat I2C kedua modul.
- `BOBOT_GIRO_FILTER` (0,96) — bobot giroskop di complementary filter.
- `AMBANG_DATAR_DERAJAT` (5) / `AMBANG_ALARM_DERAJAT` (45) — ambang status.
- `INTERVAL_BACA_MS` (100) — seberapa sering sensor dibaca dan OLED
  diperbarui (10 kali per detik).
- `JUMLAH_SAMPEL_KALIBRASI` (200) — sampel rata-rata kalibrasi giroskop.

## Library yang dibutuhkan

| Library | Versi | Keterangan |
|---|---|---|
| Adafruit GFX Library | 1.13.0 | Grafis dasar untuk OLED |
| Adafruit SSD1306 | 2.5.17 | Driver OLED 128x64 I2C |

Library di atas juga tersedia sebagai ZIP di folder [`libraries/`](libraries/)
— install lewat Arduino IDE: Sketch → Include Library → Add .ZIP Library.
Modul MPU6050 tidak butuh library tambahan (I2C mentah lewat Wire.h).

## Compile

```sh
arduino-cli compile --fqbn esp32:esp32:esp32 PengukurSudutMpu6050Oled
```

Terverifikasi 26 Nov 2026: sketch 330.477 byte (25% flash),
RAM 24.800 byte (7%), ESP32 core 4.0.0.

## Lisensi

MIT — lihat [LICENSE](LICENSE).

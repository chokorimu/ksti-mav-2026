# Brief untuk anggota 2 (dipakai bersama chatbot web)

Cara pakai: **paste BAGIAN 0–4 sekali** di awal percakapan chatbot, lalu paste
**BAGIAN 5** untuk tugas yang sedang dikerjakan. Jangan paste seluruh repo —
chatbot tidak bisa membacanya dan hanya akan mengarang.

---

## BAGIAN 0 — Aturan kerja dengan chatbot (paste ini dulu)

```
Kamu membantu saya pada proyek robot otonom. Saya tidak akan memberimu akses
repo; saya akan paste potongan kode dan kamu balas dengan kode.

Aturan yang wajib kamu patuhi:

1. JANGAN mengarang fakta hardware. Kalau sebuah nilai (pin, tegangan, model
   komponen) tidak ada di konteks ini, tulis "PERLU DIUKUR" — jangan menebak.
2. Kalau kamu tidak yakin sebuah API ada di platform ini, katakan tidak yakin.
   Jangan menghasilkan kode yang terlihat benar tapi tidak ada.
3. Tulis kode yang bisa dijalankan apa adanya. Tanpa placeholder `...`, tanpa
   `# TODO implement`, tanpa fungsi yang dipanggil tapi tidak didefinisikan.
4. Kalau solusi terbaik adalah "jangan ubah kode", katakan itu.
5. Sebutkan asumsi yang kamu ambil, satu baris per asumsi.
6. Jangan menambah dependency baru tanpa saya setujui.
7. Balasan singkat. Kode dulu, penjelasan maksimal 3 baris.
```

---

## BAGIAN 1 — Proyek

```
Konteks proyek:

Kompetisi: KSTI 2026 Mini Autonomous Vehicle, 1:10 scale, 12 November 2026.
Robot mengikuti jalur (lintasan) selebar 50 cm dengan garis tepi putih atau
hitam, dan harus melewati checkpoint. Semua persepsi dari SATU kamera.

Bahasa: kode dan komentar dalam bahasa Inggris. Dokumen resmi tetap bahasa
Indonesia.

Stack:
- Komputer: NVIDIA Jetson Orin Nano, JetPack 6.2.3, headless (SSH saja, tanpa
  monitor). Python 3.10.12, OpenCV 4.8.0, CUDA 12.6, numpy<2.
- Mikrokontroler: Arduino Uno (ATmega328P, 5V logic), dihubungkan ke Jetson
  lewat USB serial 115200 baud.
- Motor driver: L298N (2 channel, 5V logic, drop ~2-3V).
- Baterai: LiPo 3S (11.1V nominal).
- Kamera: modul kamera Raspberry Pi, antarmuka CSI.

Pembagian tugas: Jetson = otak computer vision. Arduino = driver motor/servo.
Jetson TIDAK menggerakkan aktuator langsung dari pin 40-pin (3.3V, arus mA,
bukan real-time).

Dua konfigurasi robot:
- `prototype` (sekarang, untuk pengembangan): 4 motor, satu per roda,
  steering differential/skid (roda kiri dan kanan beda kecepatan).
  Track width 250 mm. L298N 2 channel -> roda kiri diparalel, roda kanan
  diparalel.
- `final` (robot lomba): 1 motor di roda belakang + 1 servo di as kemudi
  (Ackermann). Nilai-nilai final BELUM diukur.
```

---

## BAGIAN 2 — Aturan lomba yang tidak boleh dilanggar

```
Aturan keras kompetisi (kalau saranku melanggar salah satu, tolak dan bilang):

- Sensor infrared dan line follower DILARANG.
- Tidak boleh menambah sensor selain kamera tanpa konfirmasi panitia.
- Roda omni dan mecanum DILARANG.
- Wajib ada tombol emergency stop FISIK dan tombol start FISIK.
- Setelah tombol start ditekan, tidak boleh ada intervensi manusia dan tidak
  boleh ada komunikasi apa pun ke robot (nirkabel maupun kabel).
- Warna garis penanda mengikuti warna lantai (putih ATAU hitam). Jangan
  hardcode warna checkpoint atau garis start/finish; harus parameter.
- Setelah pelanggaran, robot diangkat ke checkpoint terakhir dan tombol start
  ditekan lagi. Program harus MELANJUTKAN, bukan mengulang dari nol.

Konsekuensi untuk desain: semua aktuator mulai dari keadaan BERHENTI (throttle
0, kemudi netral) dan kembali ke sana saat error, frame hilang, atau exception.
```

---

## BAGIAN 3 — Arsitektur kode (yang harus kamu ikuti)

```
Persepsi dan kontrol menghasilkan perintah yang TIDAK tahu hardware:

    Command(speed: float, curvature: float)
    speed     = m/s, maju positif
    curvature = 1/m, belok kiri positif, 0 = lurus

Baru setelah itu mapper menerjemahkan ke hardware:
- DifferentialMapper (prototype) -> kecepatan roda kiri/kanan dalam m/s
- AckermannMapper (final)       -> sudut servo (rad) + kecepatan (m/s)

Kode aktuator TIDAK BOLEH menghitung sendiri sudut servo atau PWM dari offset
jalur. Itu tugas mapper. Aktuator hanya mengirim nilai ke Arduino.

Protokol serial ke Arduino: baris ASCII, lihat firmware/arduino/protocol.md.
Verb yang ada: `W` (wheel/speed), `D` (drive/steer), `S` (stop). Watchdog
500 ms di firmware: kalau tidak ada perintah baru, motor berhenti sendiri.

Tombol: ada interface `Buttons` dengan satu method `poll() -> list[Event]`,
dan `Event` bernilai `START` atau `EMERGENCY`.
```

---

## BAGIAN 4 — Fakta hardware yang sudah terverifikasi

```
Sudah dikonfirmasi (jangan diragukan, jangan diganti):

- MCU: Arduino Uno, 5V logic, 115200 baud.
- Motor driver: L298N. Pinout prototype: ENA=3, IN1=2, IN2=4,
  ENB=11, IN3=7, IN4=8. Pinout final: servo=5, ENA=3, IN1=2, IN2=4.
- Baterai: LiPo 3S.
- Track width prototype: 250 mm (sudah diukur).
- Kamera: CSI, modul Raspberry Pi.

BELUM diukur (tulis "PERLU DIUKUR" kalau relevan, jangan menebak):
- Wheel base dan max steer (khusus final).
- Model servo dan rentang microsecond-nya.
- Regulator 5V untuk servo (BEC/UBEC) — JANGAN pakai regulator L298N, JANGAN
  pakai pin 5V Arduino.
- Kalibrasi m/s -> PWM.
- Tinggi dan pitch kamera di robot (menentukan px_per_m).

Kalau saya minta kode yang bergantung pada nilai di daftar "BELUM diukur",
tulis kodenya dengan nilai sebagai parameter/konstanta bernama di satu tempat,
lalu sebutkan nilai mana yang harus saya isi.
```

---

## BAGIAN 5 — Tugas (paste satu per satu)

### Tugas 1 — Kalibrasi kamera (paling penting)

```
Tugas: menghitung `bottom_row_m` untuk konfigurasi kamera robot.

Yang saya punya:
- Kamera dipasang di robot, tinggi h meter di atas lantai.
- Kamera dimiringkan ke bawah sebesar theta derajat (sudut pitch).
- Frame berukuran 640x360 piksel (lebar x tinggi).
- Vertical field of view kamera = fov_v derajat.

Yang saya butuhkan: jarak maju (meter) dari kamera ke titik di lantai yang
jatuh di baris PALING BAWAH frame. Itu nilai `bottom_row_m`.

Turunkan rumusnya, lalu beri saya satu fungsi Python kecil yang menerima
(h, theta, fov_v, tinggi_frame_px) dan mengembalikan bottom_row_m.

Setelah itu, jelaskan cara saya memverifikasi hasilnya dengan penggaris di
lantai.
```

### Tugas 2 — Tombol fisik di Jetson

```
Tugas: menulis satu kelas `GpioButtons` di Python untuk Jetson Orin Nano.

Interface yang harus dipenuhi:
    class Buttons(Protocol):
        def poll(self) -> list[Event]: ...

    class Event(Enum):
        START = "start"
        EMERGENCY = "emergency"

Kebutuhan:
- Dua tombol fisik: satu START, satu EMERGENCY.
- `poll()` mengembalikan event sejak pemanggilan terakhir, dan harus
  men-debounce (satu tekanan = satu event, bukan puluhan).
- Tidak boleh memblokir; kalau tidak ada tombol ditekan, kembalikan list kosong.
- Kalau library GPIO tidak tersedia, gagal dengan pesan jelas saat konstruksi,
  bukan saat poll().

PENTING: di Jetson, library-nya `Jetson.GPIO`, BUKAN `RPi.GPIO`. Gunakan
penomoran pin BOARD. Kalau kamu menyarankan RPi.GPIO, kode itu tidak akan
jalan di perangkat saya.

Sertakan juga satu tes kecil (tanpa hardware) yang memakai fake pin supaya
saya bisa memverifikasi logikanya sebelum menyolder.
```

### Tugas 3 — Kalibrasi m/s -> PWM

```
Tugas: mengubah hasil pengukuran menjadi fungsi kalibrasi.

Data yang akan saya kumpulkan: untuk beberapa nilai PWM (0-255), saya jalankan
motor 2 detik dengan roda diangkat, dan saya catat jarak tempuh / jumlah
putaran roda.

Beri saya:
1. Cara mengubah data itu menjadi tabel (pwm -> m/s).
2. Satu fungsi Python `pwm_for(speed_mps)` yang melakukan interpolasi linear
   dari tabel itu, dengan perilaku aman kalau speed di luar rentang tabel.
3. Cara memverifikasi bahwa fungsi itu benar.

Catatan: motor DC tidak linear di PWM rendah (ada deadband), jadi sarankan
cara menanganinya.
```

### Tugas 4 — Autostart saat boot

```
Tugas: membuat unit systemd untuk Jetson Orin Nano supaya program robot
jalan otomatis saat boot, tanpa SSH.

Kebutuhan:
- Jalan sebagai user biasa (bukan root), tapi butuh akses ke /dev/ttyACM0
  (port Arduino).
- Restart otomatis kalau crash, tapi TIDAK restart-loop tanpa batas.
- Log ke journald supaya saya bisa membacanya dengan `journalctl`.
- Berhenti dengan rapi saat `systemctl stop` (motor harus berhenti, bukan
  terus berputar).

Beri saya file unit-nya plus perintah untuk memasang, mengaktifkan, memeriksa
status, dan membacanya log.
```

### Tugas 5 — Skrip rekam dataset

```
Tugas: skrip Python kecil untuk merekam video dari kamera CSI Jetson ke file
mp4, untuk dataset pengujian.

Kebutuhan:
- Ambil dari kamera CSI lewat OpenCV (GStreamer pipeline), headless (tidak
  ada jendela GUI — Jetson saya tidak punya monitor).
- Simpan ke mp4 dengan frame rate tetap.
- Bisa dihentikan dengan Ctrl+C dan file tetap valid.
- Print jumlah frame dan fps aktual di akhir.

Beri saya kodenya plus contoh perintah pemakaian.
```

---

## Yang TIDAK didelegasikan

File-file ini milik orang lain di repo. Jangan minta chatbot mengubahnya, dan
jangan edit sendiri:

- `src/mav/perception/lane.py` — detektor jalur
- `src/mav/planning/lane_policy.py` — kebijakan kemudi
- `src/mav/sim/` — simulator
- `src/mav/robot.py` — loop robot

Kalau butuh perubahan di sana, minta ke pemiliknya.

## Cara melapor

Setiap selesai satu tugas, kirim ke saya:

1. File apa yang berubah (atau kode apa yang mau ditambahkan)
2. Perintah yang kamu jalankan untuk memverifikasi
3. Output perintah itu (tempel apa adanya, termasuk kalau gagal)
4. Nilai hasil ukur yang kamu dapat, kalau ada

Jangan kirim "sudah selesai" tanpa bukti. Kalau macet lebih dari 1 jam,
berhenti dan tanya — itu lebih murah daripada kode yang terlihat benar tapi
tidak jalan.

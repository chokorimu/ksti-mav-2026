# KSTI 2026 – Mini Autonomous Vehicle: Aturan Lomba

Sumber: `GUIDEBOOK_PESERTA_MINI_AUTONOMOUS_VEHICLE_KSTI_2026.pdf` (Bab I–IV).
Dokumen ini adalah transkripsi terstruktur, bukan sumber resmi. Jika berbeda dengan PDF atau
pengumuman panitia, **panitia yang benar**. Poin bertanda **[A#]** punya catatan di bagian
"Ambiguitas" di bawah. Jangan menebak poin tersebut; tanyakan ke pengguna atau narahubung.

## 1. Informasi kegiatan

- Nama: Mini Autonomous Vehicle, bagian dari Robotics Competition at KSTI 2026.
- Tanggal/tempat: 12 November 2026, luring, Hall Robotika ITS, Surabaya.
- Technical Meeting: 8 November 2026 (pengumuman peserta terdaftar dan pembagian grup).
- Pendaftaran: https://its-robotics.com/ (pilih KSTI 2026, kategori Mini Autonomous Vehicle).
- Biaya: Rp150.000 (batch 1), Rp175.000 (batch 2). Tidak dapat dikembalikan kecuali lomba dibatalkan.
- Verifikasi pendaftaran paling lambat 2x24 jam sejak submisi.
- Informasi resmi hanya lewat platform pendaftaran dan media sosial resmi Robotics Competition at KSTI 2026.

## 2. Persyaratan tim

- Peserta adalah mahasiswa (bukti kartu tanda mahasiswa). Jumlah tim per instansi tidak dibatasi.
- Maksimal 5 orang per tim, 1 ketua. Setiap peserta dan robot hanya boleh di satu tim.
- Nama tim/robot tidak boleh mengandung SARA, pornografi, penghinaan, pelecehan. Nama kembar: yang administrasinya selesai lebih dulu berhak. **[A4]**

## 3. Hadiah

| Predikat | Hadiah |
|---|---|
| Juara 1 | Medali Emas + Sertifikat + uang pembinaan (tertulis `Rp4.00.000,00`) **[A3]** |
| Juara 2 | Medali Perak + Sertifikat + uang pembinaan (tertulis `Rp3.00.000,00`) |
| Juara 3 | Medali Perunggu + Sertifikat + uang pembinaan (tertulis `Rp2.00.000,00`) |

## 4. Misi

- **Autonomous Race (AR):** navigasi dari start ke finish pada lintasan yang disediakan dengan waktu secepat-cepatnya.
- **Autonomous Race Challenge (ARC):** sama seperti AR, tetapi sambil menghindari rintangan (obstacle). Opsional, hanya untuk tim yang lolos final.

## 5. Ketentuan robot

### 5.1 Umum
- Mobil bertenaga listrik skala 1:10. Bahan bakar dilarang keras.
- Minimal satu gandar kemudi (steering axle).
- Wajib ada **tombol fisik emergency stop** yang mudah dijangkau, dan **tombol lain untuk memulai navigasi**.
- Satu tim hanya boleh satu mobil/satu platform. Ganti platform dilarang; modifikasi komponen untuk perbaikan boleh.

### 5.2 Tenaga
- Harus listrik. Tidak ada batasan jumlah maupun kapasitas baterai.

### 5.3 Penggerak dan kemudi
- 4WD atau 2WD boleh.
- Dilarang roda omni, mecanum, atau konfigurasi lain yang memungkinkan gerak ke segala arah.
- "Robot harus mengimplementasi sistem penggerak diferensial." **[A1]**
- Sistem steering bebas.
- Bonus nilai untuk Ackermann steering (single-axle maupun two-axle): `STP = 0.25`. **[A1]**

### 5.4 Dimensi

| Dimensi | Minimum | Maksimum |
|---|---|---|
| Lebar jejak roda (track width, pusat roda kiri-kanan pada gandar yang sama) | 130 mm | 350 mm |
| Jarak sumbu roda (wheel base, pusat gandar depan-belakang) | 160 mm | 550 mm |
| Tinggi (dari tanah ke titik tertinggi) | – | 450 mm |
| Lebar (termasuk semua komponen/perangkat tambahan) | – | 400 mm |
| Panjang (depan ke belakang, termasuk semua komponen) | – | 600 mm |

### 5.5 Sensor
- **Dilarang:** sensor infrared dan/atau sensor line follower lainnya.
- Sensor lain boleh, selain yang dapat menelusuri garis secara langsung. **[A5]**

### 5.6 Komunikasi
- Segala bentuk komunikasi (nirkabel maupun kabel) untuk mengendalikan atau berkomunikasi dengan robot **selama kompetisi berlangsung sangat dilarang**.
- Artinya robot harus sepenuhnya otonom selama percobaan.

## 6. Lintasan

- Warna marka dipilih menyesuaikan warna lantai arena, jadi tim harus bisa bekerja dengan marka putih maupun hitam.
- Checkpoint ditunjukkan warna **hijau**; garis start/finish ditunjukkan warna **merah**. Namun checkpoint tidak harus punya marka apa pun, dan garis start/finish memakai warna berbeda dari marka jalan "namun belum tentu merah". **[A11]**
- Lebar lintasan: 50 cm ± 10%, **tidak selalu sama** sepanjang lintasan.
- Radius putar minimum (radius luar): 1,5 m ± 10%.
- Marka: dua garis tepi lurus tanpa putus (putih atau hitam); satu marka jalur putus-putus dengan warna sama dengan garis tepi.
- Lebar marka minimal 1 cm. Jarak antar segmen garis putus-putus minimal 5 cm.
- Checkpoint diletakkan di sepanjang lintasan. Peta contoh ada di tautan "Lintasan" di PDF (tautan tidak ikut terekstrak). **[A9]**

## 7. Aturan kompetisi – Autonomous Race

Ada 2 tahap: **Eliminasi** (harus capai skor minimum untuk lolos) dan **Final** (nilai murni dari kecepatan dan jumlah checkpoint). **[A9]**

1. Tim siap di area start 5 menit sebelum waktu percobaan.
2. Tidak siap saat dipanggil = kesempatan gugur.
3. 3 percobaan, masing-masing batas 5 menit. **Skor tertinggi** dari tiga percobaan dipakai.
4. Tujuan: lewati garis finish secepat mungkin dengan melewati semua checkpoint dalam 5 menit.
5. Jika 5 menit habis sebelum finish, tim wajib menghentikan mobil; poin dihitung dari progres.
6. Waktu persiapan 10 menit antar percobaan. Boleh: penyesuaian kecil, kalibrasi sensor, tinjau strategi, perbaikan mobil.
7. Checkpoint **wajib dilewati berurutan**. Jika tidak: pelanggaran; mobil dihentikan, dipindahkan ke checkpoint terakhir, lalu dijalankan lagi tanpa jeda waktu dari panitia.
8. Mobil wajib berada di jalur (lajur kanan atau kiri, selama tidak melewati garis tepi). **Keluar jalur jika 30% bagian mobil keluar lintasan.** Dihitung pelanggaran; mobil dihentikan, dipindah ke checkpoint terakhir, dijalankan lagi tanpa jeda. **[A2]**
9. Batas pelanggaran: 3 kali per percobaan. Lebih dari 3 kali = percobaan selesai. **[A6]**
10. Ada bagian mobil yang lepas/copot = pelanggaran dan percobaan langsung selesai. **[A7]**

### Hukuman (penambahan waktu tempuh ke hasil akhir)

Rumus umum: `tambahan_detik = k * T_stage / N_cp`, dengan `N_cp` = jumlah seluruh checkpoint.

| Pelanggaran | k |
|---|---|
| Tidak melewati checkpoint | 0.5 |
| Bagian mobil lepas | 0.2 |
| Keluar jalur | 0.8 |

## 8. Aturan kompetisi – Autonomous Race Challenge (ARC)

Sama dengan AR, dengan perbedaan:
- Hanya untuk tim yang lolos final; menambah nilai di final. Tim bebas mengambil atau tidak.
- **Checkpoint adalah obstacle yang harus dihindari.** Menghindari obstacle dianggap melewati checkpoint.
- Menabrak atau gagal menghindari obstacle = pelanggaran (mobil dihentikan, dipindah ke checkpoint terakhir, lanjut tanpa jeda).
- Keluar jalur jika **50%** bagian mobil keluar lintasan. **[A2]**
- Penalti: sama dengan tabel AR, tetapi baris pertama menjadi "menabrak atau tidak menghindari obstacle" (k = 0.5); bagian lepas k = 0.2; keluar jalur k = 0.8.
- Batas 3 pelanggaran per percobaan, bagian lepas mengakhiri percobaan, 3 percobaan, 10 menit persiapan: sama dengan AR.

## 9. Alur pertandingan

### Test Day
- Terbuka bagi semua tim untuk menguji mobil; tidak dinilai. Jadwal diatur panitia. **[A9]**
- Wajib patuhi protokol keselamatan, kembalikan perlengkapan ke posisi semula, dilarang merusak/memodifikasi perlengkapan kompetisi. Kerusakan dilaporkan ke penyelenggara.

### Perlombaan (eliminasi, final, challenge: alur sama)
1. Tim dipanggil sesuai urutan dan langsung mendapat 10 menit persiapan.
2. Setelah persiapan habis, langsung masuk 5 menit percobaan: tim meletakkan robot di garis start lalu **menekan tombol start**.
3. Setelah tombol start ditekan, **tim tidak boleh menyentuh robot sama sekali**. Kontak = percobaan selesai.
4. Saat terjadi pelanggaran, wasit menginstruksikan; tim segera **menekan tombol emergency**, meletakkan robot di checkpoint terakhir, lalu menekan tombol start lagi.
5. Saat wasit menyatakan percobaan selesai, tim segera menekan tombol emergency.

## 10. Sistem penilaian

```
S_AR  = (1 + max((T_stage - T_total) / T_stage, 0)) * cp
S_ARC = ((1 + max((T_stage - T_total) / T_stage, 0)) * cp) / (2 * Tcp)
Total = S_AR * (1 + STP + S_ARC)
```

- `T_stage`: batas waktu yang diberikan (5 menit per percobaan; asumsi 300 s). **[A9]**
- `T_total`: total waktu tempuh, **termasuk** tambahan waktu penalti.
- `cp`: jumlah checkpoint yang berhasil didapat. `Tcp`: total checkpoint di lintasan.
- `STP = 0.25` jika memakai Ackermann steering, selain itu 0.
- `S_ARC` hanya dihitung bila ikut challenge; jika tidak ikut, anggap 0. **[A8]**

Catatan turunan (interpretasi, bukan teks panduan):
- Bonus kecepatan: `(1 + max(...,0))` bernilai antara 1 dan 2. Jika `T_total >= T_stage`, nilainya 1, sehingga skor dasar sama dengan `cp`.
- Jumlah checkpoint berpengaruh lebih besar daripada kecepatan; konsistensi lebih penting daripada ngebut.

## 11. Ketentuan lain

- Keputusan juri dan wasit mutlak. Peserta wajib mengikuti arahan juri, wasit, panitia.
- Protes: hanya oleh satu anggota tim, ke juri, sopan, segera setelah pertandingan selesai dan sebelum pertandingan berikutnya. Tidak boleh saat pertandingan berlangsung. Protes atas keputusan wasit terkait pelanggaran dan penilaian tidak diterima; protes hanya untuk dugaan pelanggaran ketentuan robot atau prosedur.
- Etika: sportif, bersih, tertib. Panitia tidak bertanggung jawab atas kehilangan/kerusakan barang peserta.
- Hal yang belum diatur ditentukan panitia dan juri (diumumkan di Technical Meeting atau kanal resmi). Panitia boleh mengubah ketentuan; perubahan diumumkan di platform pendaftaran dan media sosial resmi.
- Dengan mendaftar, peserta dianggap menyetujui seluruh isi guidebook.

## 12. Narahubung

- Nindita: +62 896-4920-8124
- Yuke: +62 857-5577-3985

## 13. Ambiguitas dan kemungkinan salah ketik

Jika tugas bergantung pada salah satu poin ini, **jangan menebak**. Tanyakan ke pengguna atau narahubung.

- **A1 – Diferensial vs Ackermann.** 5.3c mewajibkan "sistem penggerak diferensial", tetapi 5.3e memberi bonus untuk Ackermann steering. Dugaan (belum terkonfirmasi): yang dimaksud adalah diferensial pada penggerak roda (differential gear/penggerak yang bisa menerima kecepatan roda berbeda), bukan differential steering. Konfirmasi ke panitia sebelum memutuskan arsitektur kemudi.
- **A2 – Batas keluar jalur.** 30% di AR, 50% di ARC. Bisa jadi salah ketik. Untuk keamanan, rancang agar mobil tidak keluar sama sekali (anggap 30%).
- **A3 – Bagian hadiah.** Judulnya menyebut "Humanoid Penalty Kick" (salinan dari kategori lain) dan nominal tertulis `Rp4.00.000,00` (kemungkinan Rp4.000.000 dst.).
- **A4 – "KSTI 2025"** muncul di persyaratan peserta nomor 7; kemungkinan sisa dari edisi lama.
- **A5 – Larangan sensor.** Jelas dilarang: infrared dan line follower. Kalimat berikutnya membingungkan. Guidebook tidak menyebut kamera secara eksplisit sebagai boleh/tidak, walaupun Bab I menyebut visi komputer. Status LiDAR, ultrasonik, IMU, dan encoder juga tidak disebut. Konfirmasi ke panitia sebelum bergantung pada sensor selain kamera.
- **A6 – Penalti vs batas pelanggaran.** Penalti berupa tambahan detik, tetapi lebih dari 3 pelanggaran mengakhiri percobaan. Tidak jelas apakah pelanggaran ke-4 tetap dihukum, dan bagaimana skor dihitung pada percobaan yang berakhir paksa.
- **A7 – Bagian lepas.** Dihitung pelanggaran dengan penalti (k = 0.2) sekaligus mengakhiri percobaan langsung. Keduanya tertulis.
- **A8 – Skor jika tidak finish / tidak ikut ARC.** Nilai `T_total` untuk percobaan yang tidak finish tidak didefinisikan. Dugaan: `T_total >= T_stage` sehingga bonus kecepatan 0. Bila tidak ikut ARC, `S_ARC` dianggap 0.
- **A9 – Data yang tidak ada.** Skor minimum eliminasi, jumlah checkpoint `N_cp`/`Tcp`, peta lintasan (hanya tautan), jadwal test day, tanggal batch 1/2.
- **A11 – Warna start/finish.** Ditulis merah, lalu "belum tentu merah". Jangan hardcode warna garis start/finish maupun warna checkpoint tanpa konfirmasi; rancang agar parameter warna bisa diubah.

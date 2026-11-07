# Rencana Riset – Mobil otonom 1:10 KSTI 2026

Status: draf awal. Isi bertanda **(asumsi)** harus dikonfirmasi pengguna.

## 1. Konteks dan batasan dari tim

| Hal | Keputusan |
|---|---|
| Komputer di robot | Jetson Orin Nano, diakses hanya lewat SSH, tanpa display. Versi JetPack: **belum diketahui** (cek dengan `cat /etc/nv_tegra_release` dan `apt show nvidia-jetpack`) |
| Lingkungan dev | Linux (prioritas), Windows (cadangan) |
| Model | Bisa berpindah antara versi prototype dan deploy |
| Tim | Programmer kurang paham elektrikal |
| Simulasi | Berbasis web, tetapi memakai computer vision asli. Stack dan antarmuka: **belum diketahui** |

Kompetisi 12 November 2026 dan Technical Meeting 8 November 2026. Jika tanggal ini masih berlaku, waktu sangat
sempit, jadi urutan kerja di bagian 4 memprioritaskan **robot yang andal di lintasan** di atas riset yang elegan.

## 2. Pertanyaan riset

### RQ1 – Persepsi lajur
Lantai dan marka berubah warna (marka putih atau hitam), lebar lintasan tidak konstan, garis tengah putus-putus (celah minimal 5 cm).
- Bandingkan pendekatan klasik (threshold warna/adaptif + deteksi garis/sliding window) dengan model belajar (segmentasi ringan).
- Ukur: robustness terhadap pencahayaan, bayangan, pantulan lantai, dan perubahan warna lantai; latency per frame di Jetson.
- Keluaran: keputusan metode + dataset rekaman nyata dengan label seperlunya.

### RQ2 – Checkpoint, start/finish, obstacle
Checkpoint hijau (tidak selalu bermarka), start/finish warna belum pasti, obstacle untuk ARC.
- Deteksi warna harus berparameter, bukan hardcode. Riset kalibrasi warna cepat di venue (mis. 1 menit saat persiapan).
- Obstacle: deteksi sederhana dulu (segmentasi area bebas), model deteksi objek hanya jika perlu.
- Hitung checkpoint dengan benar dan berurutan, dan tahan terhadap deteksi ganda.

### RQ3 – Perencanaan dan kontrol
- Model kinematik kemudi (bicycle/Ackermann) untuk radius putar ≥ 1,5 m dan lebar 50 cm ± 10%.
- Kandidat kontrol: pure pursuit atau Stanley untuk steering, PID untuk kecepatan. Kecepatan adaptif terhadap kelengkungan.
- Trade-off dari rumus skor: `cp` berpengaruh lebih besar daripada bonus kecepatan, sedangkan keluar jalur menambah waktu `0.8 * T_stage / N_cp`. Tentukan target kecepatan yang menjaga kemungkinan keluar jalur sangat kecil.
- **State machine** wajib: `BOOT → WAIT_START → RUN → STOPPED (emergency) → WAIT_START → RUN ...`. Setelah restart manual di checkpoint terakhir, hitungan checkpoint tidak boleh reset dan robot harus segera menemukan lajur lagi (asumsi: ditaruh di checkpoint, bisa miring).

### RQ4 – Jetson headless
- Kamera: CSI (pipeline GStreamer `nvarguscamerasrc`) vs USB (V4L2). Verifikasi build OpenCV di Jetson (`cv2.getBuildInformation()`: GStreamer, CUDA).
- Tanpa display: tidak ada `imshow`; gunakan penyimpanan frame/video, log, dan stream MJPEG hanya untuk dev. Alat kerja: SSH, tmux, VS Code Remote-SSH, rsync/git.
- Pantau performa dan panas: `tegrastats`, `nvpmodel`, `jetson_clocks`. Tetapkan power mode yang sama saat tes dan saat lomba.
- Autostart saat boot dengan service (systemd) dan menunggu tombol start fisik, karena tidak boleh ada komunikasi ke robot selama percobaan.
- Latensi end-to-end (kamera → perintah aktuator) dan jitter. Targetkan terukur, bukan perkiraan.

### RQ5 – Lingkungan dev (Linux utama, Windows cadangan)
- Dua kelas mesin: laptop x86_64 dan Jetson aarch64. Pisahkan berkas dependensi (dev vs jetson). Di Jetson, paket seperti PyTorch/TensorRT/OpenCV mengikuti versi JetPack; versi JetPack harus dicatat dulu.
- Linux: skrip setup tunggal. Windows: pakai WSL2; atur `.gitattributes` agar akhir baris LF; nama port serial dan path jangan hardcode.
- Pertimbangkan container untuk menyamakan lingkungan, dengan catatan bahwa image Jetson dan image laptop berbeda.

### RQ6 – Prototype → deploy model
- Alur: latih/uji di laptop atau sim → ekspor (mis. ONNX) → optimasi untuk Jetson (mis. TensorRT, presisi lebih rendah bila akurasi tetap) → uji paritas.
- Uji paritas: untuk set frame yang sama, selisih keluaran model prototype vs deploy harus di bawah ambang yang disepakati; ukur juga FPS/latency.
- Pemilihan profil lewat `configs/*.yaml` dan satu variabel (mis. `MAV_PROFILE`), bukan percabangan kode. Profil `deploy` menonaktifkan semua komponen jaringan dan debug.

### RQ7 – Simulasi web dan kesenjangan sim-ke-nyata
Kontrak yang disarankan: sim hanya menggantikan dua antarmuka.
```
Camera.read() -> frame           (SimCamera | RealCamera)
Actuator.set(steer, throttle)    (SimActuator | RealActuator)
```
Seluruh persepsi, perencanaan, kontrol, dan state machine memakai kode yang sama.
- Daftar kesenjangan yang diuji: pencahayaan/bayangan, motion blur, distorsi lensa, tinggi dan sudut kamera, warna lantai/marka, latency kamera dan aktuator, dinamika sebenarnya (traksi, delay servo).
- Domain randomization pada sim: warna lantai, warna marka (putih/hitam), lebar lintasan variatif, kecerahan.
- **Replay rekaman nyata** melalui pipeline yang sama sebagai tes regresi tanpa hardware.
- Parameter fisik robot (wheel base, track width, batas steering) masuk ke konfigurasi dan dipakai oleh sim maupun kontrol.

### RQ8 – Elektrikal minimum untuk programmer
Lihat bagian 3. Keluaran: `docs/hardware.md` berisi daftar komponen nyata, diagram daya dan sinyal, dan hasil cek oleh orang yang paham elektrikal.

## 3. Panduan elektrikal minimal (untuk programmer)

Ini panduan umum; konfirmasi ke orang yang paham elektrikal sebelum menyambung hardware.

1. **Pisahkan daya komputasi dan daya motor.** Motor menarik arus besar dan bisa menjatuhkan tegangan (brownout) sehingga Jetson reboot di tengah lintasan. Gunakan regulator terpisah untuk Jetson dan motor, dan **satukan ground** keduanya.
2. **Jangan menyuplai motor/servo dari pin Jetson.** Pin hanya untuk sinyal.
3. **Level logika.** Pin GPIO Jetson 3,3 V dan tidak toleran 5 V. Sinyal dari mikrokontroler 5 V butuh level shifter, atau gunakan komunikasi USB serial yang menghindari masalah ini.
4. **Emergency stop.** Aturan mewajibkan tombol fisik. Rancang agar tombol ini memutus daya motor di hardware, tidak hanya mengirim sinyal ke software yang bisa hang. Sediakan juga tombol start terpisah.
5. **Baterai.** Pakai sekering atau pelindung arus pada jalur utama, perhatikan polaritas, dan ikuti prosedur aman LiPo. Cek tegangan sebelum setiap percobaan.
6. **Aktuator.** Atur batas sudut steering dan batas throttle di software agar servo/rangka tidak dipaksa melewati batasnya.
7. **Kabel dan konektor.** Konektor yang longgar menyebabkan reset acak. Kunci kabel dan beri strain relief. Rencanakan agar tidak ada bagian yang lepas (bagian copot = percobaan selesai).
8. **Panas.** Jetson dan motor driver butuh aliran udara; uji beban panjang minimal sepanjang 5 menit percobaan.

Daftar yang harus dijawab orang elektrikal: motor dan driver yang dipakai, tegangan dan kapasitas baterai, arus puncak motor, regulator untuk Jetson, jalur e-stop (memutus apa), pin yang dipakai dan levelnya.

## 4. Fase kerja (urutan prioritas)

**P0 – Fondasi (harus ada lebih dulu)**
- Konfirmasi hal terbuka (bagian 6) dan hasil Technical Meeting.
- Akses SSH ke Jetson stabil; kamera terbaca; frame bisa disimpan.
- Aktuator aman: robot bisa berhenti andal lewat e-stop; steering/throttle bisa dikendalikan dari program dengan roda terangkat.

**P1 – Robot bisa menyelesaikan lintasan**
- Pipeline persepsi lajur → kontrol dasar; state machine dengan WAIT_START dan resume.
- Jalankan di sim dan di rekaman nyata dengan kode yang sama; profil `prototype` di Jetson.

**P2 – Keandalan**
- Deteksi checkpoint dan start/finish berparameter; kecepatan adaptif; uji pencahayaan dan warna lantai berbeda.
- Deploy: model teroptimasi, autostart, power mode tetap, tanpa jaringan.
- Uji gladi penuh dengan prosedur wasit (10 menit persiapan, start, pelanggaran, restart, emergency).

**P3 – Opsional**
- ARC (obstacle), Ackermann bila platform mendukung dan bonus `STP = 0.25` sepadan.

## 5. Risiko

| Risiko | Dampak | Mitigasi |
|---|---|---|
| Brownout/reset Jetson saat motor menyala | Percobaan gagal | Regulator terpisah, tes beban |
| Perubahan warna lantai/marka di venue | Persepsi gagal | Parameter warna, kalibrasi saat persiapan, domain randomization |
| Ambiguitas aturan (A1, A5, A11) | Desain melanggar aturan | Tanyakan panitia sebelum mengunci desain |
| Sim tidak mewakili dunia nyata | Hasil menyesatkan | Replay rekaman nyata, uji roda terangkat |
| Model deploy berbeda perilaku dari prototype | Regresi diam-diam | Uji paritas |
| Waktu terlalu singkat | Tidak sempat uji | Ikuti urutan P0 → P1 dulu |
| Bagian mobil copot | Percobaan selesai | Pengunci dan inspeksi sebelum start |

## 6. Hal terbuka

- Versi JetPack di Orin Nano (ditunda; cek langsung di Jetson). Semua instruksi instalasi dan versi library menunggu jawaban ini.
- Stack sim web dan cara sim bertukar frame/perintah dengan kode CV (WebSocket/HTTP)?
- Kamera dan platform mobil yang dipakai? Motor driver dan pengendali steering?
- Apakah robot memakai Ackermann (bonus 0.25) atau tidak?
- Hasil konfirmasi panitia untuk A1, A5, A11.
- Tanggal test day dan jumlah checkpoint/peta lintasan.

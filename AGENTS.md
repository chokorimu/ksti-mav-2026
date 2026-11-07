# AGENTS.md – Mobil otonom 1:10 untuk KSTI 2026 (Mini Autonomous Vehicle)

Bahasa kerja: Indonesia. Istilah teknis boleh tetap Inggris.

## Dokumen rujukan (baca sebelum bekerja pada topik terkait)
- Aturan lomba lengkap, rumus skor, penalti, ambiguitas: @docs/ksti-mav-rules.md
- Rencana riset, arsitektur, panduan elektrikal minimal: @docs/research-plan.md

Jika tugas menyentuh poin bertanda **[A#]** di rules, jangan menebak. Tanya pengguna.

## Lingkungan
- **Jetson Orin Nano: hanya lewat SSH, tanpa display.** Jangan pakai `cv2.imshow`, GUI, atau alat yang butuh layar. Untuk melihat hasil, simpan gambar/video atau gunakan stream MJPEG khusus mode dev.
- **Versi JetPack belum diketahui.** Jangan menebak versi PyTorch/TensorRT/OpenCV atau perintah instalasi. Minta pengguna menjalankan `cat /etc/nv_tegra_release` dan `apt show nvidia-jetpack` di Jetson, lalu catat hasilnya di `docs/hardware.md` sebelum memberi instruksi yang bergantung pada versi.
- Mode daya Orin Nano dicek dengan `sudo nvpmodel -q`. Pakai mode yang sama saat tes dan lomba, dan ukur performa dengan `tegrastats`.
- **Dev: Linux diutamakan**, Windows didukung sebagai cadangan (idealnya lewat WSL2). Tulis perintah untuk bash. Gunakan `pathlib`, jangan hardcode path atau nama port serial.
- Arsitektur CPU berbeda (laptop x86_64 vs Jetson aarch64). Dependensi dipisah per platform; jangan asumsikan wheel pip yang sama.
- Ada **simulasi berbasis web** yang memakai computer vision asli. Kode CV dan kontrol harus identik antara sim dan robot nyata; hanya lapisan kamera dan aktuator yang berbeda.

## Profil eksekusi
Pilih lewat konfigurasi, bukan dengan mengedit kode.

| Profil | Tempat | Model | Tujuan |
|---|---|---|---|
| `sim` | laptop + sim web | model prototype | uji algoritma tanpa hardware |
| `prototype` | Jetson/laptop | model prototype (debug, overlay, logging penuh) | iterasi dan pengukuran |
| `deploy` | Jetson | model teroptimasi (mis. TensorRT), tanpa debug | mode lomba |

Aturan `deploy`: tidak ada server jaringan, stream, atau ketergantungan SSH saat robot berjalan; mulai otomatis saat boot lalu menunggu tombol start fisik.

## Batasan keras dari lomba (jangan dilanggar oleh desain apa pun)
- Dilarang sensor infrared dan line follower. Jangan menambah sensor selain kamera tanpa konfirmasi panitia **[A5]**.
- Dilarang komunikasi apa pun untuk mengendalikan robot selama percobaan (nirkabel maupun kabel).
- Dilarang roda omni/mecanum. Wajib ada tombol fisik emergency stop dan tombol start.
- Warna marka (putih/hitam) mengikuti lantai; warna checkpoint dan start/finish jangan di-hardcode **[A11]**.
- Setelah start ditekan, tidak ada intervensi manusia. Setelah pelanggaran, manusia menekan emergency, memindahkan robot ke checkpoint terakhir, lalu menekan start lagi. **Program harus melanjutkan (hitungan checkpoint tidak boleh reset)**; verifikasi perilaku ini di setiap perubahan state machine.

## Aturan kerja untuk agent
1. **Asumsikan programmer kurang paham elektrikal.** Saat menyentuh GPIO, motor, servo, baterai, atau kabel, jelaskan singkat dalam bahasa sederhana: tegangan, arus, level logika, ground bersama.
2. **Jangan menebak fakta hardware** (model Jetson, tegangan baterai, motor driver, pin). Tanyakan atau minta dicek; catat jawabannya di `docs/hardware.md`.
3. **Beri peringatan eksplisit** sebelum perubahan yang berisiko merusak hardware: pin GPIO Jetson adalah 3,3 V, jangan hubungkan sinyal 5 V langsung; jangan menyuplai motor dari pin Jetson; jangan jalankan aktuator tanpa jalur emergency stop yang teruji.
4. **Safe by default:** kode aktuator harus berawal dari keadaan berhenti (throttle 0, steering netral), dan kembali ke sana saat error, kehilangan frame, atau exception.
5. **Uji tanpa hardware dulu:** sim atau rekaman video nyata, baru Jetson dengan roda terangkat, baru lintasan.
6. Perubahan model atau parameter yang memengaruhi latency harus disertai pengukuran FPS/latency di Jetson, bukan hanya di laptop.
7. Jika sebuah perintah belum ditentukan di bagian di bawah, jangan mengarang; tanyakan atau usulkan, lalu catat setelah disetujui.

## Struktur repo (rencana, ubah sesuai kenyataan)
```
AGENTS.md
docs/        ksti-mav-rules.md, research-plan.md, hardware.md
configs/     sim.yaml, prototype.yaml, deploy.yaml
src/         perception/, planning/, control/, io/ (camera, actuator: sim & real)
models/      prototype/, deploy/
scripts/     setup_dev.sh, setup_jetson.sh, run.sh
tests/
```

## Perintah (belum ditentukan, isi setelah disepakati)
- Setup dev: _TBD_
- Setup Jetson: _TBD_
- Jalankan sim: _TBD_
- Jalankan di Jetson (profil prototype/deploy): _TBD_
- Tes: _TBD_

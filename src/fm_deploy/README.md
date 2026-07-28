# `fm_deploy` — inference FM-Planner di drone nyata

Paket ini adalah versi hardware dari alur inference yang sudah berjalan di
simulasi (`RL_FM/src/fm_planner`). Perencananya **identik** — `fm_model.py`,
`fm_inference_base.py`, `fm_inference_node.py`, `min_jerk_planner.py`, dan
`esdf_ros2.py` disalin apa adanya, tanpa satu baris pun diubah. Kalau logika
perencanaan ikut diubah di sini, hasil terbang nyata tidak lagi bisa
dibandingkan dengan hasil simulasi, dan itu melemahkan klaim di paper.

Yang **baru** hanya tiga berkas:

| Berkas | Peran |
|---|---|
| `gemini2_depth_bridge_node.py` | Orbbec Gemini 2 → kontrak input model (pengganti `gz_depth_bridge_node.py`) |
| `fm_inference_real_node.py` | `FMInferenceNode` + lapisan keselamatan hardware dari `takeoff_land_node.py` |
| `launch/fm_real.launch.py` | Stack lengkap tanpa dependensi Gazebo |

`px4_sensor_reader.py` dan `mavros_only.launch.py` diambil dari `takeoff_land`
(versi serial/Jetson yang sudah terbukti terbang), bukan dari workspace sim.

---

## 1. Yang berubah dari simulasi ke dunia nyata

### 1.1 Kamera — bagian paling rawan

Di Gazebo, piksel "tidak ada obstacle" bernilai `+inf`, dan `_form_model_input`
memetakannya ke 10 m (**jauh/aman**). Gemini 2 mengembalikan **0** untuk "tidak
ada return" — dan 0 pada konvensi yang sama berarti **"obstacle menempel di
lensa"**. Penyebab nilai 0 di lapangan justru sering hal yang aman: permukaan
mengkilap, jendela, benda di luar 10 m, lubang stereo.

Kalau depth mentah diteruskan begitu saja, model akan melihat dinding rapat di
depan hidungnya sepanjang penerbangan. Karena itu bridge:

1. konversi 16UC1 mm → 32FC1 meter,
2. resize ke 640×480 dengan `INTER_NEAREST` (interpolasi linear membuat
   kedalaman "antara" tepi obstacle dan latar → obstacle hantu),
3. tambal lubang **kecil** dengan median tetangga,
4. sisa piksel invalid → `invalid_fill_m` (**default 10.0 m = jauh**),
5. clip ke `[0, 10]` m sesuai `DEPTH_NORM_MAX_M`,
6. skala intrinsik `camera_info` ikut menyesuaikan resize (kalau tidak, point
   cloud salah metrik dan obstacle di octomap jadi lebih lebar/sempit).

Verifikasi sebelum terbang — arahkan drone ke tembok ±1.5 m:

```bash
ros2 topic hz   /realsense/depth/float32     # ≥10 Hz stabil
ros2 topic echo /realsense/depth/stats       # p50_m ≈ 1.5, valid_pct tinggi
```

Kalau `valid_pct` < 20 %, node akan memperingatkan. Jangan terbang dengan
kamera yang "buta": outputnya tetap terlihat aman (semua 10 m).

### 1.2 Frame goal

Di simulasi drone selalu lahir di (0, 0) menghadap +X, jadi `goal_x:=20`
langsung benar. Di lapangan origin EKF berada di posisi & heading apa pun saat
boot. Node ini mengunci **home** (x, y, yaw) tepat sebelum ARM lalu menghitung:

```
goal = home + R(yaw_home) · [goal_dist, goal_lat]
```

Geofence juga didefinisikan di frame home, lalu dibungkus menjadi kotak sejajar
sumbu untuk tembok virtual ESDF (ESDF hanya mengenal kotak sejajar sumbu; AABB
selalu lebih longgar, jadi pengaman kerasnya adalah `max_home_dist` di
watchdog).

Konsekuensinya: **arahkan hidung drone ke arah yang Anda inginkan sebelum
menjalankan misi.**

### 1.3 Lapisan keselamatan yang tidak ada di Gazebo

| Mekanisme | Aksi |
|---|---|
| RC override (mode keluar dari OFFBOARD) | setpoint berhenti **seketika**, pilot pegang penuh |
| Link MAVROS putus saat misi | setpoint berhenti, failsafe PX4 mengambil alih |
| Geofence `max_home_dist` | abort → descent terkendali → AUTO.LAND |
| Deviasi ketinggian > `max_alt_error` | abort → mendarat |
| Baterai < ambang | abort → mendarat |
| Disarm tak terduga | abort |
| `mission_timeout_s` | mendarat |
| TAKEOFF/LANDING | menahan **XY home**, bukan posisi sesaat (di sim posisi sesaat dipakai, artinya drift EKF ikut jadi perintah) |

Verifikasi `COM_RC_OVERRIDE ∈ {2, 3}` dijalankan sebelum ARM. Tanpa itu stik RC
secara fisik tidak bisa merebut OFFBOARD, dan seluruh deteksi di atas percuma.

### 1.4 Guard perencana: default sengaja BERBEDA dari simulasi

Di sim, `use_safety_guards` default `false` karena tujuannya mengukur perilaku
mentah model (mode "fair" untuk ablasi). Di lapangan, "membiarkan model gagal"
berarti menabrak tembok sungguhan — jadi di `fm_real.launch.py` defaultnya
`true` (guard + escape aktif), `v_max` 0.5 (bukan 1.0), dan `cmd_hz` 50 (bukan
100, agar tidak membanjiri link serial).

Untuk pengukuran ablasi di dunia nyata nanti, matikan lagi secara eksplisit —
tapi lakukan hanya di ruang terbuka luas dengan net/pilot siaga.

### 1.5 Parameter PX4 tidak disentuh

Node sim menulis `EKF2_HGT_REF=1` (referensi tinggi = GPS). Untuk terbang
**indoor** dengan VIO/optical flow, itu justru merusak estimasi. Default di sini
`write_px4_params:=false`. Setel `MPC_XY_VEL_MAX` dan sejenisnya lewat
QGroundControl sesuai wahana Anda.

---

## 2. Instalasi

```bash
cd ~/drone_ws/src
cp -r fm_deploy .

# checkpoint FM (JANGAN ikut di-commit, ukurannya besar)
mkdir -p fm_deploy/model/fm
cp ~/saved_net/fm/run_20260724_190037/fm_planner_20260724_190037.onnx      fm_deploy/model/fm/
cp ~/saved_net/fm/run_20260724_190037/fm_planner_20260724_190037.onnx.data fm_deploy/model/fm/

cd ~/drone_ws
colcon build --packages-select fm_deploy
source install/setup.bash
```

Dependensi Python di Jetson: `torch` (kalau memakai `.pth`), `onnxruntime-gpu`
(kalau `.onnx`), `scipy`, `opencv-python`, `pyquaternion`, `cv_bridge`.

**Catatan backend ONNX:** file `.onnx` yang ada mengunci input `noise` pada
bentuk statis `[8, 9]`. Artinya dengan backend ONNX, `K` **harus 8**, dan
`use_anchor_sampling` (yang meminta K−1 sampel segar) tidak bisa dipakai. Kalau
butuh K lain, ekspor ulang dengan dimensi dinamis atau pakai `.pth`. Opsetnya 20,
jadi butuh onnxruntime ≥ 1.17.

---

## 3. Menjalankan

```bash
# T1 — komunikasi
ros2 launch fm_deploy mavros_only.launch.py fcu_url:=/dev/ttyTHS1:921600

# T2 — driver kamera (paket Orbbec, di luar repo ini)
ros2 launch orbbec_camera gemini2.launch.py

# T3 — persepsi + perencana
ros2 launch fm_deploy fm_real.launch.py \
    model_path:=$HOME/drone_ws/src/fm_deploy/model/fm/fm_planner_20260724_190037.onnx \
    dry_run:=true

# T4 — opsional
rviz2
```

### Tangga pengujian — jangan dilompati

| Tahap | Perintah | Propeller | Lolos bila |
|---|---|---|---|
| 1 | `dry_run:=true` | **DILEPAS** | `[INF] Replan ok` muncul; `/planner/candidates` masuk akal; `[FM] GATE two-sided` terisi |
| 2 | `dry_run:=false goal_dist:=0.0` | terpasang | takeoff → hover → land bersih di ruang kosong |
| 3 | `dry_run:=false goal_dist:=3.0` | terpasang | terbang lurus 3 m tanpa obstacle |
| 4 | `goal_dist:=5.0` + 1 obstacle | terpasang | menghindar dengan benar |

Tahap 1 tidak butuh baterai motor dan tidak akan pernah arming — inilah tempat
menemukan masalah TF, satuan depth, dan pemasangan kamera.

---

## 4. Yang WAJIB Anda ukur sendiri

**Transform kamera** (`cam_x`, `cam_y`, `cam_z`) dari pusat massa drone ke lensa
depth, dalam meter, konvensi FLU (maju+, kiri+, atas+). Default `0.10 / 0.00 /
-0.05` disalin dari SDF simulasi dan hampir pasti **tidak** cocok dengan wahana
Anda. Salah 5 cm menggeser seluruh octomap 5 cm; salah tanda menggeser obstacle
ke sisi yang keliru.

Cara cek cepat: hover di depan tembok, buka RViz, tampilkan `/projected_map` dan
TF. Dinding di peta harus berada persis di jarak yang Anda ukur dengan meteran.

`cam_roll/pitch/yaw` (−1.5708 / 0 / −1.5708) memetakan `base_link` FLU ke frame
**optik** (z ke depan, x ke kanan, y ke bawah) — ubah hanya kalau kamera dipasang
miring.

---

## 5. Diagnostik cepat

| Gejala | Periksa |
|---|---|
| `Timeout depth/ESDF` | `ros2 topic hz /realsense/depth/points`, `ros2 run tf2_ros tf2_echo odom camera_depth_frame`, apakah `/projected_map` terbit |
| `Message Filter dropping message` di octomap | TF `odom→base_link` tidak mengalir → MAVROS `local_position/pose` kosong (butuh GPS/VIO) |
| Semua replan gagal | `ros2 topic echo /realsense/depth/stats` — `valid_pct` rendah, atau `occ_min_z/occ_max_z` tidak mengapit `target_alt` |
| Drone menghindar ke sisi yang salah | transform kamera (bagian 4) |
| `[FM] GATE two-sided 0%` | wajar untuk lorong sempit; bandingkan dengan angka simulasi pada scene serupa |
| Setpoint tidak terkirim | `dry_run` masih `true`, atau `_rc_override`/`_link_lost` sudah menyala |

Log yang layak direkam untuk paper:
`[REAL] SELESAI` (jumlah replan, veto cost-gate, veto post-check) dan
`[REAL] Bimodality gate` di akhir setiap misi.

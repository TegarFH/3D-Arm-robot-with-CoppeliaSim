<img width="224" height="57" alt="image" src="https://github.com/user-attachments/assets/c1442ea4-67be-4f77-a49b-e24cefb1e3f2" />   <img width="224" height="59" alt="image" src="https://github.com/user-attachments/assets/facb690e-3daf-45b4-96b2-adfd66d22928" />





# 3D-Arm-robot-with-CoppeliaSim
Forward Kinematics — 4-DOF Robot Manipulator

Proyek ini mensimulasikan gerakan robot manipulator 4-DOF menggunakan metode *forward kinematics*. Dengan memasukkan sudut pada setiap joint, program akan menghitung ke mana posisi ujung lengan robot (*end-effector*) berada — lalu menampilkannya langsung dalam bentuk visualisasi 3D dan simulasi di CoppeliaSim.

#Desk
Salah satu tantangan klasik dalam robotika adalah: *"Jika kita tahu sudut setiap sendi robot, di mana ujung tangannya berada?"* — Itulah yang diselesaikan oleh *forward kinematics*.

Proyek ini menjawab pertanyaan tersebut menggunakan pendekatan matematis berbasis **matriks transformasi homogen 4×4**. Setiap joint dimodelkan sebagai transformasi rotasi dan translasi, lalu digabungkan secara berurutan melalui perkalian matriks untuk menghasilkan posisi akhir *end-effector*.

Seluruh perhitungan diimplementasikan dalam Python dan divalidasi langsung di simulator CoppeliaSim — dilengkapi antarmuka slider yang bisa digeser untuk melihat perubahan posisi robot secara real-time.

#Apa yang Bisa Dilakukan Program Ini

- Menghitung koordinat (X, Y, Z) *end-effector* berdasarkan sudut keempat joint
- Menampilkan struktur lengan robot dalam grafik 3D yang diperbarui secara langsung
- Mengontrol setiap joint melalui slider GUI, dengan rentang −180° hingga +180°
- Mengirimkan posisi joint ke CoppeliaSim secara otomatis via ZMQ Remote API

#Spesifikasi tinggi Arm Robot

| Link       | Panjang |
|------------|---------|
| Base       | 95 mm   |
| Shoulder   | 105 mm  |
| Elbow      | 105 mm  |
| Wrist      | 50 mm   |

#Cara Kerjanya

Posisi *end-effector* dihitung dari komposisi matriks transformasi seluruh joint:

```
⁰T₄ = ⁰T₁ · ¹T₂ · ²T₃ · ³T₄
```

Setiap `Tᵢ` merupakan gabungan rotasi dan translasi pada joint ke-*i*. Posisi akhir diambil dari kolom keempat matriks hasil perkalian tersebut. Karena perkalian matriks tidak komutatif, urutan perkalian harus mengikuti susunan fisik robot — mengubah urutannya akan menghasilkan posisi yang salah.

#Resource yang Digunakan
- Python 3.x
- NumPy — operasi matriks
- Matplotlib — visualisasi 3D
- Tkinter — antarmuka GUI
- CoppeliaSim + ZMQ Remote API — simulasi robot

#Cara Menjalankan/Running

1. Buka CoppeliaSim dan load scene robot yang sesuai.
2. Install dependensi yang dibutuhkan:
```bash
   pip install numpy matplotlib coppeliasim-zmqremoteapi-client
```
3. Jalankan programnya:
```bash
   python IK_Sim.py
```
4. Geser slider di GUI untuk mengatur sudut tiap joint — posisi *end-effector* dan visualisasi 3D akan langsung berubah mengikuti.

---

#Contoh Hasil
<img width="1543" height="809" alt="{09834454-E0D2-4199-9D74-ACC3D9DA745E}" src="https://github.com/user-attachments/assets/c6992774-681b-49f2-9156-3730d8b47180" /> <img width="1541" height="817" alt="{5A84DB4F-A0FC-4FD7-AC2D-241328C2FF3A}" src="https://github.com/user-attachments/assets/459a7bf7-1cfc-4836-9505-623a875054f5" />


Dengan konfigurasi θ₁=0°, θ₂=−90°, θ₃=26°, θ₄=−69°, diperoleh posisi *end-effector*:

| Koordinat | Nilai     |
|-----------|-----------|
| X         | 0.00 mm   |
| Y         | 235.94 mm |
| Z         | 106.93 mm |

Hasil ini menunjukkan ujung robot berada sekitar 235 mm ke arah sumbu Y dan sekitar 107 mm di atas bidang dasar — sesuai dengan yang ditampilkan di simulator.

#Keterbatasan & Pengembangan ke Depan

Program ini hanya mendukung *forward kinematics* — artinya input adalah sudut joint, bukan posisi target. Untuk pengembangan berikutnya, beberapa hal yang bisa ditambahkan antara lain *inverse kinematics*, perencanaan lintasan (*trajectory planning*), serta penanganan dinamika robot seperti torsi dan deteksi tabrakan.

#Author

**Tegar Fariz Habibi** — Institut Teknologi Sumatera (ITERA), 2025

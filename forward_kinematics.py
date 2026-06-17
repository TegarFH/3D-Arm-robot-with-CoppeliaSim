from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt

# ==================================================
# KONEKSI KE COPPELIASIM
# Menghubungkan Python dengan simulator CoppeliaSim
# ==================================================
client = RemoteAPIClient()
sim = client.getObject('sim')

# Mengambil seluruh joint yang ada pada scene
robot_joints = sim.getObjectsInTree(
    sim.handle_scene,
    sim.object_joint_type
)

# Memulai simulasi
sim.startSimulation()

# ==================================================
# PANJANG LINK ROBOT (mm)
# ==================================================
LINK_1_BASE = 95 #base
LINK_2_SHOULDER = 105 #shoulder
LINK_3_ELBOW = 105 #elbow
LINK_4_WRIST = 50 #wrist

# ==================================================
# FUNGSI TRANSFORMASI HOMOGEN
# ==================================================

# Rotasi terhadap sumbu Z
def rotation_z(theta):

    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    return np.array([
        [cos_theta, -sin_theta, 0, 0],
        [sin_theta,  cos_theta, 0, 0],
        [0,          0,         1, 0],
        [0,          0,         0, 1]
    ])


# Rotasi terhadap sumbu X
def rotation_x(theta):

    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)

    return np.array([
        [1, 0,          0,         0],
        [0, cos_theta, -sin_theta, 0],
        [0, sin_theta,  cos_theta, 0],
        [0, 0,          0,         1]
    ])


# Translasi sepanjang sumbu Z
def translation_z(distance):

    return np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, distance],
        [0, 0, 0, 1]
    ])


# ==================================================
# FORWARD KINEMATICS
# Menghitung posisi seluruh joint dan end-effector
# ==================================================
def forward_kinematics(
        theta1,
        theta2,
        theta3,
        theta4):

    # Transformasi base ke joint 1
    transform_joint1 = (
        rotation_z(theta1)
        @ translation_z(LINK_1_BASE)
    )

    # Transformasi base ke joint 2
    transform_joint2 = (
        transform_joint1
        @ rotation_x(theta2)
        @ translation_z(LINK_2_SHOULDER)
    )

    # Transformasi base ke joint 3
    transform_joint3 = (
        transform_joint2
        @ rotation_x(theta3)
        @ translation_z(LINK_3_ELBOW)
    )

    # Transformasi base ke end-effector
    transform_end_effector = (
        transform_joint3
        @ rotation_x(theta4)
        @ translation_z(LINK_4_WRIST)
    )

    # Posisi setiap joint
    base_position = np.array([0, 0, 0])

    joint1_position = transform_joint1[:3, 3]
    joint2_position = transform_joint2[:3, 3]
    joint3_position = transform_joint3[:3, 3]

    end_effector_position = (
        transform_end_effector[:3, 3]
    )

    all_joint_positions = np.array([
        base_position,
        joint1_position,
        joint2_position,
        joint3_position,
        end_effector_position
    ])

    return (
        all_joint_positions,
        end_effector_position
    )


# ==================================================
# VARIABEL GLOBAL
# Menyimpan sudut joint dan status gripper
# ==================================================
joint_angles_deg = [0, 0, 0, 0]

gripper_is_open = True


# ==================================================
# INISIALISASI VISUALISASI 3D
# ==================================================
plt.ion()

figure_robot = plt.figure()

axis_robot = figure_robot.add_subplot(
    111,
    projection='3d'
)


# ==================================================
# MENGGAMBAR SISTEM KOORDINAT XYZ
# ==================================================
def draw_coordinate_axes(axis):

    axis_length = 200

    # Sumbu X
    axis.plot(
        [0, axis_length],
        [0, 0],
        [0, 0],
        'r-',
        linewidth=2
    )

    axis.text(
        axis_length,
        0,
        0,
        "X",
        color='red'
    )

    # Sumbu Y
    axis.plot(
        [0, 0],
        [0, axis_length],
        [0, 0],
        'g-',
        linewidth=2
    )

    axis.text(
        0,
        axis_length,
        0,
        "Y",
        color='green'
    )

    # Sumbu Z
    axis.plot(
        [0, 0],
        [0, 0],
        [0, axis_length],
        'b-',
        linewidth=2
    )

    axis.text(
        0,
        0,
        axis_length,
        "Z",
        color='blue'
    )

    # Titik origin
    axis.scatter(
        0,
        0,
        0,
        color='black',
        s=30
    )


# ==================================================
# UPDATE VISUALISASI ROBOT
# ==================================================
def update_robot_plot(
        joint_positions,
        end_effector_position):

    axis_robot.clear()

    x_coordinates = joint_positions[:, 0]
    y_coordinates = joint_positions[:, 1]
    z_coordinates = joint_positions[:, 2]

    # Menggambar robot
    axis_robot.plot(
        x_coordinates,
        y_coordinates,
        z_coordinates,
        'o-',
        linewidth=3
    )

    # Marker end-effector
    axis_robot.scatter(
        end_effector_position[0],
        end_effector_position[1],
        end_effector_position[2],
        color='red',
        s=80,
        label='End Effector'
    )

    draw_coordinate_axes(axis_robot)

    axis_robot.set_xlim(-300, 300)
    axis_robot.set_ylim(-300, 300)
    axis_robot.set_zlim(0, 300)

    axis_robot.set_xlabel("X (mm)")
    axis_robot.set_ylabel("Y (mm)")
    axis_robot.set_zlabel("Z (mm)")

    axis_robot.set_title(
        "Forward Kinematics Robot Arm"
    )

    plt.draw()
    plt.pause(0.001)


# ==================================================
# UPDATE SISTEM
# 1. Mengirim sudut joint ke CoppeliaSim
# 2. Menghitung FK
# 3. Memperbarui GUI
# 4. Memperbarui visualisasi 3D
# ==================================================
def update_system(event=None):

    joint_angles_rad = np.radians(
        joint_angles_deg
    )

    # Kirim sudut joint ke simulator
    for joint_index in range(4):

        sim.setJointTargetPosition(
            robot_joints[joint_index],
            float(joint_angles_rad[joint_index])
        )

    # Hitung FK
    (
        joint_positions,
        end_effector_position
    ) = forward_kinematics(
        *joint_angles_rad
    )

    # Update koordinat end-effector
    label_position_x.config(
        text=f"X: {end_effector_position[0]:.2f} mm"
    )

    label_position_y.config(
        text=f"Y: {end_effector_position[1]:.2f} mm"
    )

    label_position_z.config(
        text=f"Z: {end_effector_position[2]:.2f} mm"
    )

    # Update grafik 3D
    update_robot_plot(
        joint_positions,
        end_effector_position
    )


# ==================================================
# CALLBACK SAAT SLIDER BERUBAH
# ==================================================
def on_slider_change(
        joint_index,
        slider_value):

    joint_angles_deg[joint_index] = float(
        slider_value
    )

    update_system()


# ==================================================
# MEMBUAT SLIDER JOINT
# ==================================================
def create_joint_slider(
        joint_index,
        joint_name,
        row_number):

    frame_slider = tk.Frame(root)

    frame_slider.grid(
        row=row_number,
        column=0
    )

    tk.Label(
        frame_slider,
        text=joint_name
    ).pack(side="left")

    slider_joint = tk.Scale(
        frame_slider,
        from_=-180,
        to=180,
        orient="horizontal",
        command=lambda value:
        on_slider_change(
            joint_index,
            value
        )
    )

    slider_joint.pack(side="left")


# ==================================================
# GUI (interface)
# ==================================================
root = tk.Tk()

root.title(
    "Robot Arm FK + 3D Visualization"
)

# Slider Joint
create_joint_slider(0, "BASE", 0)
create_joint_slider(1, "SHOULDER", 1)
create_joint_slider(2, "ELBOW", 2)
create_joint_slider(3, "WRIST", 3)

# Label posisi end-effector
label_position_x = tk.Label(
    root,
    text="X: 0 mm"
)

label_position_x.grid(
    row=5,
    column=0
)

label_position_y = tk.Label(
    root,
    text="Y: 0 mm"
)

label_position_y.grid(
    row=6,
    column=0
)

label_position_z = tk.Label(
    root,
    text="Z: 0 mm"
)

label_position_z.grid(
    row=7,
    column=0
)

# Update awal
update_system()

# Menjalankan GUI (Interface)
root.mainloop()
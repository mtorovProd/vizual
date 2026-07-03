import tkinter as tk
from tkinter import ttk, messagebox
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math

class ManipulatorVisualizer:
    def __init__(self, root):
        self.root = root
        self.root.title("Визуализатор манипулятора (6 степеней свободы)")
        self.root.geometry("900x700")
        
        # Текущие позиции суставов
        self.joints = None
        self.current_angles = [0.0] * 6
        
        self.create_widgets()
        self.reset_joints()
    
    def create_widgets(self):
        # Заголовок
        title_label = tk.Label(self.root, text="🤖 Манипулятор (6 DOF) - Прямая кинематика", 
                               font=("Arial", 16, "bold"))
        title_label.pack(pady=10)
        
        # Создаем прокручиваемый фрейм для параметров
        canvas = tk.Canvas(self.root)
        scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Углы суставов
        angles_frame = tk.LabelFrame(scrollable_frame, text="📐 Углы суставов (градусы)", 
                                     padx=10, pady=10)
        angles_frame.pack(padx=20, pady=10, fill="x")
        
        self.angle_entries = []
        for i in range(6):
            tk.Label(angles_frame, text=f"J{i+1}:").grid(row=0, column=i*2, padx=5, pady=5)
            entry = tk.Entry(angles_frame, width=8)
            entry.grid(row=0, column=i*2+1, padx=5, pady=5)
            entry.insert(0, "0")
            self.angle_entries.append(entry)
        
        # DH параметры
        dh_frame = tk.LabelFrame(scrollable_frame, text="🔧 DH-параметры", 
                                padx=10, pady=10)
        dh_frame.pack(padx=20, pady=10, fill="x")
        
        # Заголовки
        tk.Label(dh_frame, text="Параметр", font=("Arial", 9, "bold")).grid(row=0, column=0, padx=5, pady=5)
        for i in range(6):
            tk.Label(dh_frame, text=f"J{i+1}", font=("Arial", 9, "bold")).grid(row=0, column=i+1, padx=5, pady=5)
        
        # a (длина звена)
        tk.Label(dh_frame, text="a (мм):").grid(row=1, column=0, padx=5, pady=5)
        self.dh_a_entries = []
        default_a = [0, 305, 0, 0, 0, 0]
        for i in range(6):
            entry = tk.Entry(dh_frame, width=8)
            entry.grid(row=1, column=i+1, padx=5, pady=5)
            entry.insert(0, str(default_a[i]))
            self.dh_a_entries.append(entry)
        
        # d (смещение)
        tk.Label(dh_frame, text="d (мм):").grid(row=2, column=0, padx=5, pady=5)
        self.dh_d_entries = []
        default_d = [280, 0, 0, 290, 0, 100]
        for i in range(6):
            entry = tk.Entry(dh_frame, width=8)
            entry.grid(row=2, column=i+1, padx=5, pady=5)
            entry.insert(0, str(default_d[i]))
            self.dh_d_entries.append(entry)
        
        # alpha (угол закрутки)
        tk.Label(dh_frame, text="α (град):").grid(row=3, column=0, padx=5, pady=5)
        self.dh_alpha_entries = []
        default_alpha = [90, 0, 90, -90, 90, 0]
        for i in range(6):
            entry = tk.Entry(dh_frame, width=8)
            entry.grid(row=3, column=i+1, padx=5, pady=5)
            entry.insert(0, str(default_alpha[i]))
            self.dh_alpha_entries.append(entry)
        
        # theta offset
        tk.Label(dh_frame, text="θ offset (град):").grid(row=4, column=0, padx=5, pady=5)
        self.dh_theta_entries = []
        default_theta = [0, -90, 0, 0, 0, 0]
        for i in range(6):
            entry = tk.Entry(dh_frame, width=8)
            entry.grid(row=4, column=i+1, padx=5, pady=5)
            entry.insert(0, str(default_theta[i]))
            self.dh_theta_entries.append(entry)
        
        # Work Frame
        wf_frame = tk.LabelFrame(scrollable_frame, text="🌍 Рабочая система координат (Work Frame)", 
                                padx=10, pady=10)
        wf_frame.pack(padx=20, pady=10, fill="x")
        
        self.wf_entries = {}
        wf_labels = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
        wf_defaults = [0, 0, 0, 0, 0, 0]
        for i, (label, default) in enumerate(zip(wf_labels, wf_defaults)):
            tk.Label(wf_frame, text=f"{label}:").grid(row=0, column=i*2, padx=5, pady=5)
            entry = tk.Entry(wf_frame, width=8)
            entry.grid(row=0, column=i*2+1, padx=5, pady=5)
            entry.insert(0, str(default))
            self.wf_entries[label] = entry
        
        # Tool Frame
        tf_frame = tk.LabelFrame(scrollable_frame, text="🔧 Инструмент (Tool Frame)", 
                                padx=10, pady=10)
        tf_frame.pack(padx=20, pady=10, fill="x")
        
        self.tf_entries = {}
        tf_labels = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
        tf_defaults = [0, 0, 50, 0, 0, 0]
        for i, (label, default) in enumerate(zip(tf_labels, tf_defaults)):
            tk.Label(tf_frame, text=f"{label}:").grid(row=0, column=i*2, padx=5, pady=5)
            entry = tk.Entry(tf_frame, width=8)
            entry.grid(row=0, column=i*2+1, padx=5, pady=5)
            entry.insert(0, str(default))
            self.tf_entries[label] = entry
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # Параметры анимации
        anim_frame = tk.Frame(self.root)
        anim_frame.pack(pady=10)
        
        tk.Label(anim_frame, text="Шагов анимации:").pack(side="left", padx=5)
        self.frames_entry = tk.Entry(anim_frame, width=10)
        self.frames_entry.pack(side="left", padx=5)
        self.frames_entry.insert(0, "60")
        
        tk.Label(anim_frame, text="Скорость (мс):").pack(side="left", padx=5)
        self.speed_entry = tk.Entry(anim_frame, width=10)
        self.speed_entry.pack(side="left", padx=5)
        self.speed_entry.insert(0, "50")
        
        # Кнопки
        btn_frame = tk.Frame(self.root)
        btn_frame.pack(pady=10)
        
        reset_btn = tk.Button(btn_frame, text="🔄 Сбросить", 
                             command=self.reset_joints, bg="#ff9800", fg="white",
                             font=("Arial", 10, "bold"), padx=10, pady=5)
        reset_btn.pack(side="left", padx=5)
        
        start_btn = tk.Button(btn_frame, text="▶️ Запустить", 
                             command=self.start_animation, bg="#2196F3", fg="white",
                             font=("Arial", 10, "bold"), padx=10, pady=5)
        start_btn.pack(side="left", padx=5)
    
    def reset_joints(self):
        """Сбросить манипулятор в нулевое положение"""
        self.current_angles = [0.0] * 6
        for entry in self.angle_entries:
            entry.delete(0, tk.END)
            entry.insert(0, "0")
        self.calculate_and_set_joints()
    
    def calculate_forward_kinematics(self, angles, dh_params, wf_params, tf_params):
        """Расчет прямой кинематики с возвратом позиций всех суставов"""
        joints = [(0.0, 0.0, 0.0)]  # Основание
        
        # Текущая матрица трансформации
        T = np.eye(4)
        
        for i in range(6):
            theta = math.radians(angles[i] + dh_params['theta'][i])
            d = dh_params['d'][i]
            a = dh_params['a'][i]
            alpha = math.radians(dh_params['alpha'][i])
            
            # DH матрица трансформации
            T_joint = np.array([
                [math.cos(theta), -math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
                [math.sin(theta), math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
                [0, math.sin(alpha), math.cos(alpha), d],
                [0, 0, 0, 1]
            ])
            
            T = T @ T_joint
            joints.append((T[0, 3], T[1, 3], T[2, 3]))
        
        # Применяем Work Frame
        wf_T = self.create_transformation_matrix(
            wf_params['X'], wf_params['Y'], wf_params['Z'],
            wf_params['Rx'], wf_params['Ry'], wf_params['Rz']
        )
        
        # Применяем Tool Frame
        tf_T = self.create_transformation_matrix(
            tf_params['X'], tf_params['Y'], tf_params['Z'],
            tf_params['Rx'], tf_params['Ry'], tf_params['Rz']
        )
        
        # Трансформируем все суставы через Work Frame
        transformed_joints = [(0.0, 0.0, 0.0)]
        for joint in joints[1:]:
            point = np.array([joint[0], joint[1], joint[2], 1])
            transformed = wf_T @ point
            transformed_joints.append((transformed[0], transformed[1], transformed[2]))
        
        # Добавляем позицию инструмента
        tool_point = np.array([joints[-1][0], joints[-1][1], joints[-1][2], 1])
        tool_transformed = wf_T @ tf_T @ tool_point
        transformed_joints.append((tool_transformed[0], tool_transformed[1], tool_transformed[2]))
        
        return transformed_joints
    
    def create_transformation_matrix(self, x, y, z, rx, ry, rz):
        """Создание матрицы трансформации из позиции и ориентации"""
        rx_rad = math.radians(rx)
        ry_rad = math.radians(ry)
        rz_rad = math.radians(rz)
        
        # Матрицы поворота
        Rx = np.array([
            [1, 0, 0, 0],
            [0, math.cos(rx_rad), -math.sin(rx_rad), 0],
            [0, math.sin(rx_rad), math.cos(rx_rad), 0],
            [0, 0, 0, 1]
        ])
        
        Ry = np.array([
            [math.cos(ry_rad), 0, math.sin(ry_rad), 0],
            [0, 1, 0, 0],
            [-math.sin(ry_rad), 0, math.cos(ry_rad), 0],
            [0, 0, 0, 1]
        ])
        
        Rz = np.array([
            [math.cos(rz_rad), -math.sin(rz_rad), 0, 0],
            [math.sin(rz_rad), math.cos(rz_rad), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Матрица переноса
        T = np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])
        
        return T @ Rz @ Ry @ Rx
    
    def calculate_and_set_joints(self):
        """Рассчитать и установить позиции суставов"""
        try:
            angles = [float(entry.get()) for entry in self.angle_entries]
            
            dh_params = {
                'a': [float(entry.get()) for entry in self.dh_a_entries],
                'd': [float(entry.get()) for entry in self.dh_d_entries],
                'alpha': [float(entry.get()) for entry in self.dh_alpha_entries],
                'theta': [float(entry.get()) for entry in self.dh_theta_entries]
            }
            
            wf_params = {label: float(entry.get()) for label, entry in self.wf_entries.items()}
            tf_params = {label: float(entry.get()) for label, entry in self.tf_entries.items()}
            
            self.joints = self.calculate_forward_kinematics(angles, dh_params, wf_params, tf_params)
            self.current_angles = angles
            
        except ValueError:
            messagebox.showerror("Ошибка", "Неверный формат данных")
            return
    
    def draw_sphere(self, ax, center, radius, color='blue', alpha=0.6):
        """Рисует сферу в 3D"""
        u = np.linspace(0, 2 * np.pi, 20)
        v = np.linspace(0, np.pi, 20)
        
        x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
        y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
        z = center[2] + radius * np.outer(np.ones(np.size(u)), np.cos(v))
        
        ax.plot_surface(x, y, z, color=color, alpha=alpha, linewidth=0)
    
    def draw_cylinder(self, ax, start, end, radius, color='gray', alpha=0.6):
        """Рисует цилиндр между двумя точками в 3D"""
        start = np.array(start)
        end = np.array(end)
        
        direction = end - start
        length = np.linalg.norm(direction)
        
        if length == 0:
            return
        
        direction = direction / length
        
        theta = np.linspace(0, 2*np.pi, 20)
        z = np.linspace(0, length, 2)
        theta, z = np.meshgrid(theta, z)
        
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        
        z_axis = np.array([0, 0, 1])
        
        if np.allclose(direction, z_axis):
            rotation_matrix = np.eye(3)
        elif np.allclose(direction, -z_axis):
            rotation_matrix = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        else:
            v = np.cross(z_axis, direction)
            s = np.linalg.norm(v)
            c = np.dot(z_axis, direction)
            
            v = v / s
            vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
            rotation_matrix = np.eye(3) + vx + vx @ vx * ((1 - c) / (s**2))
        
        for i in range(x.shape[0]):
            for j in range(x.shape[1]):
                point = np.array([x[i, j], y[i, j], z[i, j]])
                rotated_point = rotation_matrix @ point + start
                x[i, j] = rotated_point[0]
                y[i, j] = rotated_point[1]
                z[i, j] = rotated_point[2]
        
        ax.plot_surface(x, y, z, color=color, alpha=alpha, linewidth=0)
    
    def start_animation(self):
        try:
            angles = [float(entry.get()) for entry in self.angle_entries]
            
            dh_params = {
                'a': [float(entry.get()) for entry in self.dh_a_entries],
                'd': [float(entry.get()) for entry in self.dh_d_entries],
                'alpha': [float(entry.get()) for entry in self.dh_alpha_entries],
                'theta': [float(entry.get()) for entry in self.dh_theta_entries]
            }
            
            wf_params = {label: float(entry.get()) for label, entry in self.wf_entries.items()}
            tf_params = {label: float(entry.get()) for label, entry in self.tf_entries.items()}
            
            frames = int(self.frames_entry.get())
            speed = int(self.speed_entry.get())
            
        except ValueError:
            messagebox.showerror("Ошибка", "Неверный формат данных")
            return
        
        # Начальное положение (текущие углы)
        start_joints = self.calculate_forward_kinematics(
            self.current_angles, dh_params, wf_params, tf_params
        )
        
        # Конечное положение (новые углы)
        end_joints = self.calculate_forward_kinematics(
            angles, dh_params, wf_params, tf_params
        )
        
        self.animate_to_target(start_joints, end_joints, frames, speed)
    
    def animate_to_target(self, start_joints, end_joints, frames, speed):
        """Анимация движения манипулятора"""
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')
        
        def interpolate_joints(t):
            result = []
            for i in range(len(start_joints)):
                x = start_joints[i][0] + t * (end_joints[i][0] - start_joints[i][0])
                y = start_joints[i][1] + t * (end_joints[i][1] - start_joints[i][1])
                z = start_joints[i][2] + t * (end_joints[i][2] - start_joints[i][2])
                result.append((x, y, z))
            return result
        
        def update(frame):
            ax.clear()
            
            t = frame / (frames - 1) if frames > 1 else 1.0
            current_joints = interpolate_joints(t)
            
            # Рисуем цилиндры между суставами
            for i in range(len(current_joints) - 1):
                self.draw_cylinder(ax, current_joints[i], current_joints[i+1], 
                                  radius=15, color='#607D8B', alpha=0.7)
            
            # Рисуем сферы в суставах
            for i, joint in enumerate(current_joints):
                if i == 0:
                    self.draw_sphere(ax, joint, radius=25, color='black', alpha=0.8)
                elif i == len(current_joints) - 1:
                    self.draw_sphere(ax, joint, radius=20, color='red', alpha=0.8)
                else:
                    self.draw_sphere(ax, joint, radius=20, color='#2196F3', alpha=0.7)
            
            # Подписи суставов
            for i, (x, y, z) in enumerate(current_joints):
                if i == 0:
                    label = "Основание"
                elif i == len(current_joints) - 1:
                    label = "Инструмент"
                else:
                    label = f"J{i}"
                ax.text(x + 30, y + 30, z + 30, label, fontsize=9, color='navy', weight='bold')
            
            # Настройки осей
            ax.set_xlabel('X (мм)')
            ax.set_ylabel('Y (мм)')
            ax.set_zlabel('Z (мм)')
            ax.set_title(f'Манипулятор (шаг {frame+1}/{frames})')
            
            # Пределы осей
            all_x = [p[0] for p in current_joints]
            all_y = [p[1] for p in current_joints]
            all_z = [p[2] for p in current_joints]
            
            margin = 100
            ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
            ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
            ax.set_zlim(min(all_z) - margin, max(all_z) + margin)
            
            ax.grid(True)
            
            return ax,
        
        ani = FuncAnimation(fig, update, frames=frames, interval=speed, 
                           blit=False, repeat=False)
        
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    root = tk.Tk()
    app = ManipulatorVisualizer(root)
    root.mainloop()
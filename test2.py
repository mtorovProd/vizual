import tkinter as tk
from tkinter import ttk, messagebox
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math

class ManipulatorVisualizer:
    def __init__(self, root):
        self.root = root
        self.root.title("🤖 Манипулятор (6 DOF) - Прямая и обратная кинематика")
        self.root.geometry("1500x950")
        
        # Текущие углы и состояние анимации
        self.current_angles = [0.0] * 6
        self.animating = False
        self.anim_id = None
        
        # Основной контейнер
        main_frame = tk.Frame(self.root)
        main_frame.pack(fill="both", expand=True)
        
        # Левая панель - параметры
        self.create_params_panel(main_frame)
        
        # Правая панель - визуализация
        self.create_visualization_panel(main_frame)
        
        # Инициализация
        self.reset_joints()
        self.update_visualization()
    
    def create_params_panel(self, parent):
        """Левая панель с параметрами"""
        left_frame = tk.Frame(parent, width=650)
        left_frame.pack(side="left", fill="y", padx=5, pady=5)
        left_frame.pack_propagate(False)
        
        # Заголовок
        tk.Label(left_frame, text="📊 Параметры манипулятора", 
                 font=("Arial", 14, "bold")).pack(pady=5)
        
        # Прокручиваемый фрейм для параметров
        canvas = tk.Canvas(left_frame)
        scrollbar = ttk.Scrollbar(left_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # === ОБРАТНАЯ КИНЕМАТИКА (Плоскость YZ) ===
        ik_frame = tk.LabelFrame(scrollable_frame, text="🎯 Обратная кинематика (плоскость YZ)", 
                                 padx=10, pady=10, bg="#fff3e0")
        ik_frame.pack(padx=10, pady=5, fill="x")
        
        tk.Label(ik_frame, text="Целевая точка:", font=("Arial", 10, "bold"), 
                 bg="#fff3e0").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        
        tk.Label(ik_frame, text="Y:", bg="#fff3e0").grid(row=0, column=1, padx=5, pady=5)
        self.ik_y_entry = tk.Entry(ik_frame, width=10)
        self.ik_y_entry.grid(row=0, column=2, padx=5, pady=5)
        self.ik_y_entry.insert(0, "300")
        
        tk.Label(ik_frame, text="Z:", bg="#fff3e0").grid(row=0, column=3, padx=5, pady=5)
        self.ik_z_entry = tk.Entry(ik_frame, width=10)
        self.ik_z_entry.grid(row=0, column=4, padx=5, pady=5)
        self.ik_z_entry.insert(0, "400")
        
        tk.Label(ik_frame, text="Конфигурация:", bg="#fff3e0").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.ik_config_var = tk.StringVar(value="elbow_up")
        elbow_up_rb = tk.Radiobutton(ik_frame, text="Локоть вверх", variable=self.ik_config_var, 
                                     value="elbow_up", bg="#fff3e0")
        elbow_up_rb.grid(row=1, column=1, columnspan=2, padx=5, pady=5, sticky="w")
        
        elbow_down_rb = tk.Radiobutton(ik_frame, text="Локоть вниз", variable=self.ik_config_var, 
                                       value="elbow_down", bg="#fff3e0")
        elbow_down_rb.grid(row=1, column=3, columnspan=2, padx=5, pady=5, sticky="w")
        
        ik_btn = tk.Button(ik_frame, text="🎯 Рассчитать IK", command=self.calculate_inverse_kinematics,
                          bg="#ff9800", fg="white", font=("Arial", 10, "bold"), padx=10, pady=5)
        ik_btn.grid(row=2, column=0, columnspan=5, pady=10, sticky="ew")
        
        # === УГЛЫ СУСТАВОВ (2 строки по 3) ===
        angles_frame = tk.LabelFrame(scrollable_frame, text="📐 Углы суставов (градусы)", 
                                     padx=10, pady=10)
        angles_frame.pack(padx=10, pady=5, fill="x")
        
        self.angle_entries = []
        # Первая строка: J1, J2, J3
        for i in range(3):
            tk.Label(angles_frame, text=f"J{i+1}:").grid(row=0, column=i*2, padx=5, pady=5)
            entry = tk.Entry(angles_frame, width=10)
            entry.grid(row=0, column=i*2+1, padx=5, pady=5)
            entry.insert(0, "0")
            self.angle_entries.append(entry)
        
        # Вторая строка: J4, J5, J6
        for i in range(3, 6):
            tk.Label(angles_frame, text=f"J{i+1}:").grid(row=1, column=(i-3)*2, padx=5, pady=5)
            entry = tk.Entry(angles_frame, width=10)
            entry.grid(row=1, column=(i-3)*2+1, padx=5, pady=5)
            entry.insert(0, "0")
            self.angle_entries.append(entry)
        
        # === ОПИСАНИЕ DH-ПАРАМЕТРОВ ===
        info_frame = tk.LabelFrame(scrollable_frame, text="ℹ️ Описание DH-параметров", 
                                   padx=10, pady=10, bg="#e3f2fd")
        info_frame.pack(padx=10, pady=5, fill="x")
        
        info_text = (
            "📏 a (длина звена) — расстояние между осями Z соседних суставов "
            "вдоль оси X. Определяет длину звена.\n\n"
            "📐 d (смещение) — расстояние между осями X соседних суставов "
            "вдоль оси Z. Для вращательных суставов обычно 0, для призматических — переменное.\n\n"
            "🔄 α (alpha, угол закрутки) — угол между осями Z соседних суставов "
            "вокруг оси X. Определяет взаимную ориентацию осей вращения.\n\n"
            "🎯 θ offset (смещение угла) — начальное смещение угла сустава. "
            "Добавляется к текущему углу Jn для получения полного угла θ."
        )
        
        tk.Label(info_frame, text=info_text, 
                 font=("Arial", 9), bg="#e3f2fd", 
                 justify="left", wraplength=580).pack(anchor="w")
        
        # === DH ПАРАМЕТРЫ ===
        dh_frame = tk.LabelFrame(scrollable_frame, text="🔧 DH-параметры", 
                                padx=10, pady=10)
        dh_frame.pack(padx=10, pady=5, fill="x")
        
        tk.Label(dh_frame, text="Параметр", font=("Arial", 9, "bold")).grid(row=0, column=0, padx=3, pady=3)
        for i in range(6):
            tk.Label(dh_frame, text=f"J{i+1}", font=("Arial", 9, "bold")).grid(row=0, column=i+1, padx=3, pady=3)
        
        # a (длина звена)
        tk.Label(dh_frame, text="a (мм):", font=("Arial", 9)).grid(row=1, column=0, padx=3, pady=3, sticky="e")
        self.dh_a_entries = []
        default_a = [0, 305, 0, 0, 0, 0]
        for i in range(6):
            entry = tk.Entry(dh_frame, width=7)
            entry.grid(row=1, column=i+1, padx=3, pady=3)
            entry.insert(0, str(default_a[i]))
            self.dh_a_entries.append(entry)
        
        # d (смещение)
        tk.Label(dh_frame, text="d (мм):", font=("Arial", 9)).grid(row=2, column=0, padx=3, pady=3, sticky="e")
        self.dh_d_entries = []
        default_d = [280, 0, 0, 290, 0, 100]
        for i in range(6):
            entry = tk.Entry(dh_frame, width=7)
            entry.grid(row=2, column=i+1, padx=3, pady=3)
            entry.insert(0, str(default_d[i]))
            self.dh_d_entries.append(entry)
        
        # alpha (угол закрутки)
        tk.Label(dh_frame, text="α (град):", font=("Arial", 9)).grid(row=3, column=0, padx=3, pady=3, sticky="e")
        self.dh_alpha_entries = []
        default_alpha = [90, 0, 90, -90, 90, 0]
        for i in range(6):
            entry = tk.Entry(dh_frame, width=7)
            entry.grid(row=3, column=i+1, padx=3, pady=3)
            entry.insert(0, str(default_alpha[i]))
            self.dh_alpha_entries.append(entry)
        
        # theta offset
        tk.Label(dh_frame, text="θ offset:", font=("Arial", 9)).grid(row=4, column=0, padx=3, pady=3, sticky="e")
        self.dh_theta_entries = []
        default_theta = [0, -90, 0, 0, 0, 0]
        for i in range(6):
            entry = tk.Entry(dh_frame, width=7)
            entry.grid(row=4, column=i+1, padx=3, pady=3)
            entry.insert(0, str(default_theta[i]))
            self.dh_theta_entries.append(entry)
        
        # === WORK FRAME ===
        wf_frame = tk.LabelFrame(scrollable_frame, text="🌍 Рабочая система координат (Work Frame)", 
                                padx=10, pady=10)
        wf_frame.pack(padx=10, pady=5, fill="x")
        
        self.wf_entries = {}
        wf_labels = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
        wf_defaults = [0, 0, 0, 0, 0, 0]
        for i, (label, default) in enumerate(zip(wf_labels, wf_defaults)):
            tk.Label(wf_frame, text=f"{label}:").grid(row=0, column=i*2, padx=3, pady=3)
            entry = tk.Entry(wf_frame, width=7)
            entry.grid(row=0, column=i*2+1, padx=3, pady=3)
            entry.insert(0, str(default))
            self.wf_entries[label] = entry
        
        # === TOOL FRAME ===
        tf_frame = tk.LabelFrame(scrollable_frame, text="🔧 Инструмент (Tool Frame)", 
                                padx=10, pady=10)
        tf_frame.pack(padx=10, pady=5, fill="x")
        
        self.tf_entries = {}
        tf_labels = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
        tf_defaults = [0, 0, 50, 0, 0, 0]
        for i, (label, default) in enumerate(zip(tf_labels, tf_defaults)):
            tk.Label(tf_frame, text=f"{label}:").grid(row=0, column=i*2, padx=3, pady=3)
            entry = tk.Entry(tf_frame, width=7)
            entry.grid(row=0, column=i*2+1, padx=3, pady=3)
            entry.insert(0, str(default))
            self.tf_entries[label] = entry
        
        # === ИНФОРМАЦИЯ О ПОЗИЦИИ ===
        pos_frame = tk.LabelFrame(scrollable_frame, text="📍 Текущая позиция инструмента", 
                                 padx=10, pady=10)
        pos_frame.pack(padx=10, pady=5, fill="x")
        
        self.pos_labels = {}
        pos_labels = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
        for i, label in enumerate(pos_labels):
            tk.Label(pos_frame, text=f"{label}:").grid(row=i//3, column=(i%3)*2, padx=3, pady=3, sticky="e")
            lbl = tk.Label(pos_frame, text="0.00", font=("Arial", 9, "bold"), fg="#2196F3")
            lbl.grid(row=i//3, column=(i%3)*2+1, padx=3, pady=3, sticky="w")
            self.pos_labels[label] = lbl
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # === ПАРАМЕТРЫ АНИМАЦИИ ===
        anim_frame = tk.Frame(left_frame)
        anim_frame.pack(pady=5, fill="x", padx=10)
        
        tk.Label(anim_frame, text="Шагов:").pack(side="left", padx=5)
        self.frames_entry = tk.Entry(anim_frame, width=8)
        self.frames_entry.pack(side="left", padx=5)
        self.frames_entry.insert(0, "30")
        
        tk.Label(anim_frame, text="Скорость (мс):").pack(side="left", padx=5)
        self.speed_entry = tk.Entry(anim_frame, width=8)
        self.speed_entry.pack(side="left", padx=5)
        self.speed_entry.insert(0, "50")
        
        # === КНОПКИ УПРАВЛЕНИЯ ===
        btn_frame1 = tk.Frame(left_frame)
        btn_frame1.pack(pady=5, fill="x", padx=10)
        
        self.reset_btn = tk.Button(btn_frame1, text="🔄 Сбросить", 
                             command=self.reset_joints, bg="#ff9800", fg="white",
                             font=("Arial", 10, "bold"), padx=10, pady=5)
        self.reset_btn.pack(side="left", padx=5, expand=True, fill="x")
        
        self.update_btn = tk.Button(btn_frame1, text="⚡ Мгновенно", 
                             command=self.update_visualization, bg="#4CAF50", fg="white",
                             font=("Arial", 10, "bold"), padx=10, pady=5)
        self.update_btn.pack(side="left", padx=5, expand=True, fill="x")
        
        btn_frame2 = tk.Frame(left_frame)
        btn_frame2.pack(pady=5, fill="x", padx=10)
        
        self.start_btn = tk.Button(btn_frame2, text="▶️ Анимация", 
                             command=self.start_animation, bg="#2196F3", fg="white",
                             font=("Arial", 10, "bold"), padx=10, pady=5)
        self.start_btn.pack(side="left", expand=True, fill="x")
    
    def create_visualization_panel(self, parent):
        """Правая панель с 3D визуализацией"""
        right_frame = tk.Frame(parent)
        right_frame.pack(side="right", fill="both", expand=True, padx=5, pady=5)
        
        # Matplotlib figure
        self.fig = Figure(figsize=(8, 8), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
    
    def calculate_inverse_kinematics(self):
        """Расчет обратной кинематики для плоскости YZ"""
        try:
            target_y = float(self.ik_y_entry.get())
            target_z = float(self.ik_z_entry.get())
            config = self.ik_config_var.get()
            
            # Получаем DH параметры
            dh_params = {
                'a': [float(entry.get()) for entry in self.dh_a_entries],
                'd': [float(entry.get()) for entry in self.dh_d_entries],
                'alpha': [float(entry.get()) for entry in self.dh_alpha_entries],
                'theta': [float(entry.get()) for entry in self.dh_theta_entries]
            }
            
            # Для плоскости YZ используем геометрический подход
            # J1 = 0 (работаем в плоскости YZ)
            # J2 и J3 вычисляются геометрически
            
            d1 = dh_params['d'][0]  # Высота первого сустава
            a2 = dh_params['a'][1]  # Длина второго звена
            a3 = dh_params['a'][2]  # Длина третьего звена
            d4 = dh_params['d'][3]  # Смещение четвертого сустава
            
            # Целевая точка относительно основания
            y_target = target_y
            z_target = target_z - d1
            
            # Расстояние до цели
            r = math.sqrt(y_target**2 + z_target**2)
            
            # Проверка достижимости
            max_reach = a2 + a3
            min_reach = abs(a2 - a3)
            
            if r > max_reach:
                messagebox.showerror("Ошибка", f"Точка недостижима! Максимальная досягаемость: {max_reach:.2f} мм")
                return
            
            if r < min_reach:
                messagebox.showerror("Ошибка", f"Точка слишком близко! Минимальная досягаемость: {min_reach:.2f} мм")
                return
            
            # Угол J3 (по теореме косинусов)
            cos_J3 = (a2**2 + a3**2 - r**2) / (2 * a2 * a3)
            cos_J3 = max(-1.0, min(1.0, cos_J3))  # Ограничение диапазона
            
            if config == "elbow_up":
                J3 = math.acos(cos_J3)
            else:  # elbow_down
                J3 = -math.acos(cos_J3)
            
            # Угол J2
            alpha = math.atan2(z_target, y_target)
            beta = math.atan2(a3 * math.sin(J3), a2 + a3 * math.cos(J3))
            J2 = alpha - beta
            
            # Преобразуем в градусы и учитываем theta offset
            J2_deg = math.degrees(J2) - dh_params['theta'][1]
            J3_deg = math.degrees(J3) - dh_params['theta'][2]
            
            # Обновляем углы
            new_angles = [0.0, J2_deg, J3_deg, 0.0, 0.0, 0.0]
            
            # Обновляем поля ввода
            for i, entry in enumerate(self.angle_entries):
                entry.delete(0, tk.END)
                entry.insert(0, f"{new_angles[i]:.2f}")
            
            self.current_angles = new_angles
            self.update_visualization()
            
            messagebox.showinfo("Успех", f"Обратная кинематика рассчитана!\nJ2: {J2_deg:.2f}°, J3: {J3_deg:.2f}°")
            
        except ValueError as e:
            messagebox.showerror("Ошибка", f"Неверный формат данных: {e}")
        except Exception as e:
            messagebox.showerror("Ошибка", f"Произошла ошибка: {e}")
    
    def get_params(self):
        """Получить все параметры из полей ввода"""
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
            
            return angles, dh_params, wf_params, tf_params
        except ValueError as e:
            messagebox.showerror("Ошибка", f"Неверный формат данных: {e}")
            return None
    
    def calculate_forward_kinematics(self, angles, dh_params, wf_params, tf_params):
        """Расчет прямой кинематики"""
        joints = [(0.0, 0.0, 0.0)]
        
        T = np.eye(4)
        
        for i in range(6):
            theta = math.radians(angles[i] + dh_params['theta'][i])
            d = dh_params['d'][i]
            a = dh_params['a'][i]
            alpha = math.radians(dh_params['alpha'][i])
            
            T_joint = np.array([
                [math.cos(theta), -math.sin(theta) * math.cos(alpha), math.sin(theta) * math.sin(alpha), a * math.cos(theta)],
                [math.sin(theta), math.cos(theta) * math.cos(alpha), -math.cos(theta) * math.sin(alpha), a * math.sin(theta)],
                [0, math.sin(alpha), math.cos(alpha), d],
                [0, 0, 0, 1]
            ])
            
            T = T @ T_joint
            joints.append((T[0, 3], T[1, 3], T[2, 3]))
        
        # Work Frame
        wf_T = self.create_transformation_matrix(
            wf_params['X'], wf_params['Y'], wf_params['Z'],
            wf_params['Rx'], wf_params['Ry'], wf_params['Rz']
        )
        
        # Tool Frame
        tf_T = self.create_transformation_matrix(
            tf_params['X'], tf_params['Y'], tf_params['Z'],
            tf_params['Rx'], tf_params['Ry'], tf_params['Rz']
        )
        
        # Трансформируем через Work Frame
        transformed_joints = [(0.0, 0.0, 0.0)]
        for joint in joints[1:]:
            point = np.array([joint[0], joint[1], joint[2], 1])
            transformed = wf_T @ point
            transformed_joints.append((transformed[0], transformed[1], transformed[2]))
        
        # Позиция инструмента
        tool_point = np.array([joints[-1][0], joints[-1][1], joints[-1][2], 1])
        tool_transformed = wf_T @ tf_T @ tool_point
        transformed_joints.append((tool_transformed[0], tool_transformed[1], tool_transformed[2]))
        
        return transformed_joints
    
    def create_transformation_matrix(self, x, y, z, rx, ry, rz):
        """Создание матрицы трансформации"""
        rx_rad = math.radians(rx)
        ry_rad = math.radians(ry)
        rz_rad = math.radians(rz)
        
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
        
        T = np.array([
            [1, 0, 0, x],
            [0, 1, 0, y],
            [0, 0, 1, z],
            [0, 0, 0, 1]
        ])
        
        return T @ Rz @ Ry @ Rx
    
    def draw_manipulator(self, joints):
        """Отрисовка манипулятора"""
        self.ax.clear()
        
        # Рисуем звенья (линии)
        for i in range(len(joints) - 1):
            self.ax.plot([joints[i][0], joints[i+1][0]], 
                        [joints[i][1], joints[i+1][1]], 
                        [joints[i][2], joints[i+1][2]], 
                        'o-', color='#607D8B', linewidth=4, markersize=0)
        
        # Суставы - точки
        for i, joint in enumerate(joints):
            if i == 0:
                color = 'black'
                size = 120
            elif i == len(joints) - 1:
                color = 'red'
                size = 100
            else:
                color = '#2196F3'
                size = 80
            
            self.ax.scatter([joint[0]], [joint[1]], [joint[2]], 
                          color=color, s=size, alpha=0.8, edgecolors='white', linewidth=1.5)
        
        # Подписи
        for i, (x, y, z) in enumerate(joints):
            if i == 0:
                label = "Base"
            elif i == len(joints) - 1:
                label = "Tool"
            else:
                label = f"J{i}"
            self.ax.text(x + 20, y + 20, z + 20, label, fontsize=9, 
                        color='navy', weight='bold')
        
        # Настройки осей
        self.ax.set_xlabel('X (мм)')
        self.ax.set_ylabel('Y (мм)')
        self.ax.set_zlabel('Z (мм)')
        
        # Пределы осей
        all_x = [p[0] for p in joints]
        all_y = [p[1] for p in joints]
        all_z = [p[2] for p in joints]
        
        margin = 100
        self.ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        self.ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        self.ax.set_zlim(min(all_z) - margin, max(all_z) + margin)
        
        self.ax.grid(True, alpha=0.3)
        
        self.canvas.draw()
    
    def update_position_labels(self, joints):
        """Обновление меток с позицией"""
        if len(joints) >= 2:
            tool = joints[-1]
            self.pos_labels['X'].config(text=f"{tool[0]:.2f}")
            self.pos_labels['Y'].config(text=f"{tool[1]:.2f}")
            self.pos_labels['Z'].config(text=f"{tool[2]:.2f}")
            self.pos_labels['Rx'].config(text="0.00")
            self.pos_labels['Ry'].config(text="0.00")
            self.pos_labels['Rz'].config(text="0.00")
    
    def update_visualization(self):
        """Мгновенное обновление визуализации"""
        result = self.get_params()
        if result is None:
            return
        
        angles, dh_params, wf_params, tf_params = result
        joints = self.calculate_forward_kinematics(angles, dh_params, wf_params, tf_params)
        self.current_angles = angles
        
        self.draw_manipulator(joints)
        self.update_position_labels(joints)
    
    def reset_joints(self):
        """Сброс углов в нулевое положение"""
        for entry in self.angle_entries:
            entry.delete(0, tk.END)
            entry.insert(0, "0")
        self.current_angles = [0.0] * 6
        self.update_visualization()
    
    def start_animation(self):
        """Запуск анимации с интерполяцией углов"""
        if self.animating:
            self.animating = False
            if self.anim_id:
                self.root.after_cancel(self.anim_id)
            self.start_btn.config(text="▶️ Анимация", bg="#2196F3")
            return
        
        result = self.get_params()
        if result is None:
            return
        
        angles, dh_params, wf_params, tf_params = result
        
        try:
            frames = int(self.frames_entry.get())
            speed = int(self.speed_entry.get())
        except ValueError:
            messagebox.showerror("Ошибка", "Неверный формат данных")
            return
        
        # Сохраняем начальные и конечные УГЛЫ (не позиции!)
        self.animating = True
        self.start_btn.config(text="⏸ Стоп", bg="#f44336")
        self.current_frame = 0
        self.total_frames = frames
        self.start_angles = self.current_angles.copy()
        self.end_angles = angles
        self.dh_params = dh_params
        self.wf_params = wf_params
        self.tf_params = tf_params
        self.speed = speed
        
        self.animate_step()
    
    def animate_step(self):
        """Один шаг анимации с интерполяцией углов"""
        if not self.animating or self.current_frame >= self.total_frames:
            self.animating = False
            self.start_btn.config(text="▶️ Анимация", bg="#2196F3")
            self.current_angles = self.end_angles
            return
        
        t = self.current_frame / (self.total_frames - 1) if self.total_frames > 1 else 1.0
        
        # Интерполяция УГЛОВ (не позиций!)
        current_angles = []
        for i in range(6):
            angle = self.start_angles[i] + t * (self.end_angles[i] - self.start_angles[i])
            current_angles.append(angle)
        
        # Пересчитываем прямую кинематику для промежуточных углов
        current_joints = self.calculate_forward_kinematics(
            current_angles, self.dh_params, self.wf_params, self.tf_params
        )
        
        self.draw_manipulator(current_joints)
        self.update_position_labels(current_joints)
        
        self.current_frame += 1
        self.anim_id = self.root.after(self.speed, self.animate_step)

if __name__ == "__main__":
    root = tk.Tk()
    app = ManipulatorVisualizer(root)
    root.mainloop()
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
        self.root.geometry("1500x1000")
        
        # Текущие углы и состояние анимации
        self.current_angles = [0.0] * 6
        self.animating = False
        self.anim_id = None
        self.target_point = None
        
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
    
    def clamp_angle(self, angle, joint_index):
        """Ограничение угла в пределах min/max для конкретного сустава"""
        min_angle = float(self.limit_min_entries[joint_index].get())
        max_angle = float(self.limit_max_entries[joint_index].get())
        return max(min_angle, min(max_angle, angle))
    
    def clamp_angles(self, angles):
        """Ограничение всех углов в пределах min/max"""
        clamped = []
        for i, angle in enumerate(angles):
            clamped.append(self.clamp_angle(angle, i))
        return clamped
    
    def get_limits(self):
        """Получить текущие ограничения углов"""
        try:
            mins = [float(entry.get()) for entry in self.limit_min_entries]
            maxs = [float(entry.get()) for entry in self.limit_max_entries]
            return mins, maxs
        except ValueError:
            return [-180] * 6, [180] * 6
    
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
                 bg="#fff3e0").grid(row=0, column=0, columnspan=6, padx=5, pady=5, sticky="w")
        
        tk.Label(ik_frame, text="Y:", bg="#fff3e0", font=("Arial", 9)).grid(row=1, column=0, padx=5, pady=5, sticky="e")
        self.ik_y_entry = tk.Entry(ik_frame, width=10)
        self.ik_y_entry.grid(row=1, column=1, padx=5, pady=5, sticky="w")
        self.ik_y_entry.insert(0, "50")
        
        tk.Label(ik_frame, text="Z:", bg="#fff3e0", font=("Arial", 9)).grid(row=1, column=2, padx=5, pady=5, sticky="e")
        self.ik_z_entry = tk.Entry(ik_frame, width=10)
        self.ik_z_entry.grid(row=1, column=3, padx=5, pady=5, sticky="w")
        self.ik_z_entry.insert(0, "60")
        
        tk.Label(ik_frame, text="Точность (мм):", bg="#fff3e0", font=("Arial", 9)).grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="e")
        self.ik_tolerance_entry = tk.Entry(ik_frame, width=10)
        self.ik_tolerance_entry.grid(row=2, column=2, padx=5, pady=5, sticky="w")
        self.ik_tolerance_entry.insert(0, "0.5")
        
        tk.Label(ik_frame, text="Макс. итераций:", bg="#fff3e0", font=("Arial", 9)).grid(row=2, column=3, padx=5, pady=5, sticky="e")
        self.ik_max_iter_entry = tk.Entry(ik_frame, width=10)
        self.ik_max_iter_entry.grid(row=2, column=4, padx=5, pady=5, sticky="w")
        self.ik_max_iter_entry.insert(0, "100")
        
        ik_btn = tk.Button(ik_frame, text="🎯 Рассчитать IK и анимировать", 
                          command=self.calculate_and_animate_ik,
                          bg="#ff9800", fg="white", font=("Arial", 10, "bold"), padx=10, pady=5)
        ik_btn.grid(row=3, column=0, columnspan=6, pady=10, sticky="ew")
        
        # === УГЛЫ СУСТАВОВ (2 строки по 3) ===
        angles_frame = tk.LabelFrame(scrollable_frame, text="📐 Углы суставов (градусы)", 
                                     padx=10, pady=10)
        angles_frame.pack(padx=10, pady=5, fill="x")
        
        self.angle_entries = []
        for i in range(3):
            tk.Label(angles_frame, text=f"J{i+1}:").grid(row=0, column=i*2, padx=5, pady=5)
            entry = tk.Entry(angles_frame, width=10)
            entry.grid(row=0, column=i*2+1, padx=5, pady=5)
            entry.insert(0, "0")
            self.angle_entries.append(entry)
        
        for i in range(3, 6):
            tk.Label(angles_frame, text=f"J{i+1}:").grid(row=1, column=(i-3)*2, padx=5, pady=5)
            entry = tk.Entry(angles_frame, width=10)
            entry.grid(row=1, column=(i-3)*2+1, padx=5, pady=5)
            entry.insert(0, "0")
            self.angle_entries.append(entry)
        
        # === ОГРАНИЧЕНИЯ УГЛОВ ===
        limits_frame = tk.LabelFrame(scrollable_frame, text="🚧 Ограничения углов (градусы)", 
                                     padx=10, pady=10, bg="#fce4ec")
        limits_frame.pack(padx=10, pady=5, fill="x")
        
        tk.Label(limits_frame, text="Сустав", font=("Arial", 9, "bold"), bg="#fce4ec").grid(row=0, column=0, padx=5, pady=3)
        tk.Label(limits_frame, text="Min", font=("Arial", 9, "bold"), bg="#fce4ec").grid(row=0, column=1, padx=5, pady=3)
        tk.Label(limits_frame, text="Max", font=("Arial", 9, "bold"), bg="#fce4ec").grid(row=0, column=2, padx=5, pady=3)
        tk.Label(limits_frame, text="Текущий", font=("Arial", 9, "bold"), bg="#fce4ec").grid(row=0, column=3, padx=5, pady=3)
        tk.Label(limits_frame, text="Статус", font=("Arial", 9, "bold"), bg="#fce4ec").grid(row=0, column=4, padx=5, pady=3)
        
        # Стандартные ограничения для промышленного манипулятора
        default_mins = [-170, -60, -180, -180, -120, -360]
        default_maxs = [170, 85, 75, 180, 120, 360]
        
        self.limit_min_entries = []
        self.limit_max_entries = []
        self.limit_status_labels = []
        
        for i in range(6):
            row = i + 1
            tk.Label(limits_frame, text=f"J{i+1}", font=("Arial", 9, "bold"), bg="#fce4ec").grid(row=row, column=0, padx=5, pady=3)
            
            min_entry = tk.Entry(limits_frame, width=8)
            min_entry.grid(row=row, column=1, padx=5, pady=3)
            min_entry.insert(0, str(default_mins[i]))
            self.limit_min_entries.append(min_entry)
            
            max_entry = tk.Entry(limits_frame, width=8)
            max_entry.grid(row=row, column=2, padx=5, pady=3)
            max_entry.insert(0, str(default_maxs[i]))
            self.limit_max_entries.append(max_entry)
            
            # Текущее значение (только для отображения)
            current_lbl = tk.Label(limits_frame, text="0.00", font=("Arial", 9), 
                                   bg="#fce4ec", fg="#2196F3")
            current_lbl.grid(row=row, column=3, padx=5, pady=3)
            
            # Статус (OK / LIMIT)
            status_lbl = tk.Label(limits_frame, text="✓ OK", font=("Arial", 9, "bold"), 
                                 bg="#fce4ec", fg="#4CAF50")
            status_lbl.grid(row=row, column=4, padx=5, pady=3)
            self.limit_status_labels.append((current_lbl, status_lbl))
        
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
            "Добавляется к текущему углу Jn для получения полного угла θ.\n\n"
            "🚧 Ограничения — мин/макс значения углов. При выходе за пределы "
            "угол автоматически обрезается до допустимого диапазона."
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
        
        tk.Label(dh_frame, text="a (мм):", font=("Arial", 9)).grid(row=1, column=0, padx=3, pady=3, sticky="e")
        self.dh_a_entries = []
        default_a = [100, 0, 100, 0, 100, 0]
        for i in range(6):
            entry = tk.Entry(dh_frame, width=7)
            entry.grid(row=1, column=i+1, padx=3, pady=3)
            entry.insert(0, str(default_a[i]))
            self.dh_a_entries.append(entry)
        
        tk.Label(dh_frame, text="d (мм):", font=("Arial", 9)).grid(row=2, column=0, padx=3, pady=3, sticky="e")
        self.dh_d_entries = []
        default_d = [0, 60, 0, -60, 0, 60]
        for i in range(6):
            entry = tk.Entry(dh_frame, width=7)
            entry.grid(row=2, column=i+1, padx=3, pady=3)
            entry.insert(0, str(default_d[i]))
            self.dh_d_entries.append(entry)
        
        tk.Label(dh_frame, text="α (град):", font=("Arial", 9)).grid(row=3, column=0, padx=3, pady=3, sticky="e")
        self.dh_alpha_entries = []
        default_alpha = [90, 0, 90, -90, 90, 0]
        for i in range(6):
            entry = tk.Entry(dh_frame, width=7)
            entry.grid(row=3, column=i+1, padx=3, pady=3)
            entry.insert(0, str(default_alpha[i]))
            self.dh_alpha_entries.append(entry)
        
        tk.Label(dh_frame, text="θ offset:", font=("Arial", 9)).grid(row=4, column=0, padx=3, pady=3, sticky="e")
        self.dh_theta_entries = []
        default_theta = [0, 0, 0, 0, 0, 0]
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
        tf_defaults = [0, 0, -30, 0, 0, 0]
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
        
        self.fig = Figure(figsize=(8, 8), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
    
    def update_limits_display(self, angles):
        """Обновление отображения текущих углов и статусов ограничений"""
        for i in range(6):
            current_lbl, status_lbl = self.limit_status_labels[i]
            current_lbl.config(text=f"{angles[i]:.2f}")
            
            try:
                min_angle = float(self.limit_min_entries[i].get())
                max_angle = float(self.limit_max_entries[i].get())
                
                if angles[i] <= min_angle or angles[i] >= max_angle:
                    status_lbl.config(text="⚠ LIMIT", fg="#f44336")
                    current_lbl.config(fg="#f44336")
                elif angles[i] < min_angle + 5 or angles[i] > max_angle - 5:
                    status_lbl.config(text="⚠ Близко", fg="#ff9800")
                    current_lbl.config(fg="#ff9800")
                else:
                    status_lbl.config(text="✓ OK", fg="#4CAF50")
                    current_lbl.config(fg="#2196F3")
            except ValueError:
                status_lbl.config(text="? Ошибка", fg="#9e9e9e")
    
    def calculate_inverse_kinematics_numerical(self, target_pos, dh_params, wf_params, tf_params, 
                                                tolerance=0.5, max_iterations=100):
        """Численный метод обратной кинематики с использованием Якобиана"""
        current_angles = self.current_angles.copy()
        clamped_count = 0
        
        for iteration in range(max_iterations):
            # Применяем ограничения к углам
            current_angles = self.clamp_angles(current_angles)
            
            # Вычисляем текущую позицию
            joints = self.calculate_forward_kinematics(current_angles, dh_params, wf_params, tf_params)
            current_pos = np.array(joints[-1])
            
            # Ошибка
            error = target_pos - current_pos
            error_magnitude = np.linalg.norm(error)
            
            print(f"[IK] Итерация {iteration+1}: ошибка = {error_magnitude:.2f} мм, углы = {[f'{a:.2f}' for a in current_angles]}")
            
            # Проверка сходимости
            if error_magnitude < tolerance:
                print(f"[IK] Сходимость достигнута за {iteration+1} итераций")
                return current_angles, True
            
            # Вычисляем Якобиан численным дифференцированием
            J = np.zeros((3, 6))
            delta = 0.01
            
            for i in range(6):
                angles_plus = current_angles.copy()
                angles_plus[i] += delta
                # Проверяем, не выходит ли за пределы
                angles_plus[i] = self.clamp_angle(angles_plus[i], i)
                
                joints_plus = self.calculate_forward_kinematics(angles_plus, dh_params, wf_params, tf_params)
                pos_plus = np.array(joints_plus[-1])
                
                J[:, i] = (pos_plus - current_pos) / math.radians(delta)
            
            # Псевдообращение Якобиана
            J_pinv = np.linalg.pinv(J)
            
            # Обновление углов
            delta_angles = J_pinv @ error
            new_angles = current_angles + np.degrees(delta_angles)
            
            # Ограничение углов
            new_angles = self.clamp_angles(new_angles)
            
            # Подсчёт сработавших ограничений
            for i in range(6):
                if new_angles[i] != current_angles[i] + np.degrees(delta_angles[i]):
                    clamped_count += 1
            
            current_angles = new_angles
        
        print(f"[IK] Не достигнута сходимость за {max_iterations} итераций (ограничений: {clamped_count})")
        return current_angles, False
    
    def calculate_and_animate_ik(self):
        """Рассчитать IK и запустить анимацию"""
        try:
            target_y = float(self.ik_y_entry.get())
            target_z = float(self.ik_z_entry.get())
            tolerance = float(self.ik_tolerance_entry.get())
            max_iter = int(self.ik_max_iter_entry.get())
            
            dh_params = {
                'a': [float(entry.get()) for entry in self.dh_a_entries],
                'd': [float(entry.get()) for entry in self.dh_d_entries],
                'alpha': [float(entry.get()) for entry in self.dh_alpha_entries],
                'theta': [float(entry.get()) for entry in self.dh_theta_entries]
            }
            
            wf_params = {label: float(entry.get()) for label, entry in self.wf_entries.items()}
            tf_params = {label: float(entry.get()) for label, entry in self.tf_entries.items()}
            
            target_pos = np.array([0.0, target_y, target_z])
            self.target_point = target_pos
            
            solved_angles, success = self.calculate_inverse_kinematics_numerical(
                target_pos, dh_params, wf_params, tf_params, tolerance, max_iter
            )
            
            # Финальное ограничение углов
            solved_angles = self.clamp_angles(solved_angles)
            
            if success:
                for i, entry in enumerate(self.angle_entries):
                    entry.delete(0, tk.END)
                    entry.insert(0, f"{solved_angles[i]:.2f}")
                
                self.animate_to_angles(solved_angles, dh_params, wf_params, tf_params)
                
                # Проверка, были ли сработавшие ограничения
                mins, maxs = self.get_limits()
                warnings = []
                for i in range(6):
                    if abs(solved_angles[i] - mins[i]) < 0.01 or abs(solved_angles[i] - maxs[i]) < 0.01:
                        warnings.append(f"J{i+1} на пределе ({solved_angles[i]:.2f}°)")
                
                msg = f"Обратная кинематика рассчитана!\nУглы:\n" + "\n".join([f"J{i+1}: {solved_angles[i]:.2f}°" for i in range(6)])
                if warnings:
                    msg += "\n\n⚠️ ВНИМАНИЕ:\n" + "\n".join(warnings)
                    messagebox.showwarning("Успех (с ограничениями)", msg)
                else:
                    messagebox.showinfo("Успех", msg)
            else:
                final_error = np.linalg.norm(target_pos - np.array(self.calculate_forward_kinematics(solved_angles, dh_params, wf_params, tf_params)[-1]))
                messagebox.showwarning("Предупреждение", 
                    f"Не достигнута требуемая точность.\n"
                    f"Возможно, точка недостижима с учётом ограничений углов.\n"
                    f"Последняя ошибка: {final_error:.2f} мм")
        
        except ValueError as e:
            messagebox.showerror("Ошибка", f"Неверный формат данных: {e}")
        except Exception as e:
            messagebox.showerror("Ошибка", f"Произошла ошибка: {e}")
            import traceback
            traceback.print_exc()
    
    def animate_to_angles(self, target_angles, dh_params, wf_params, tf_params):
        """Анимация к целевым углам"""
        try:
            frames = int(self.frames_entry.get())
            speed = int(self.speed_entry.get())
        except ValueError:
            messagebox.showerror("Ошибка", "Неверный формат данных")
            return
        
        self.animating = True
        self.start_btn.config(text="⏸ Стоп", bg="#f44336")
        self.current_frame = 0
        self.total_frames = frames
        self.start_angles = self.current_angles.copy()
        self.end_angles = target_angles
        self.dh_params = dh_params
        self.wf_params = wf_params
        self.tf_params = tf_params
        self.speed = speed
        
        self.animate_step()
    
    def get_params(self):
        """Получить все параметры из полей ввода"""
        try:
            angles = [float(entry.get()) for entry in self.angle_entries]
            
            # Применяем ограничения к углам
            angles = self.clamp_angles(angles)
            
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
        
        wf_T = self.create_transformation_matrix(
            wf_params['X'], wf_params['Y'], wf_params['Z'],
            wf_params['Rx'], wf_params['Ry'], wf_params['Rz']
        )
        
        tf_T = self.create_transformation_matrix(
            tf_params['X'], tf_params['Y'], tf_params['Z'],
            tf_params['Rx'], tf_params['Ry'], tf_params['Rz']
        )
        
        transformed_joints = [(0.0, 0.0, 0.0)]
        for joint in joints[1:]:
            point = np.array([joint[0], joint[1], joint[2], 1])
            transformed = wf_T @ point
            transformed_joints.append((transformed[0], transformed[1], transformed[2]))
        
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
        
        for i in range(len(joints) - 1):
            self.ax.plot([joints[i][0], joints[i+1][0]], 
                        [joints[i][1], joints[i+1][1]], 
                        [joints[i][2], joints[i+1][2]], 
                        'o-', color='#607D8B', linewidth=4, markersize=0)
        
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
        
        if self.target_point is not None:
            self.ax.scatter([self.target_point[0]], [self.target_point[1]], [self.target_point[2]], 
                          color='green', s=200, marker='*', alpha=0.6, edgecolors='darkgreen', linewidth=2)
            self.ax.text(self.target_point[0] + 20, self.target_point[1] + 20, self.target_point[2] + 20, 
                        "Цель", fontsize=10, color='green', weight='bold')
        
        for i, (x, y, z) in enumerate(joints):
            if i == 0:
                label = "Base"
            elif i == len(joints) - 1:
                label = "Tool"
            else:
                label = f"J{i}"
            self.ax.text(x + 20, y + 20, z + 20, label, fontsize=9, 
                        color='navy', weight='bold')
        
        self.ax.set_xlabel('X (мм)')
        self.ax.set_ylabel('Y (мм)')
        self.ax.set_zlabel('Z (мм)')
        
        all_x = [p[0] for p in joints]
        all_y = [p[1] for p in joints]
        all_z = [p[2] for p in joints]
        
        if self.target_point is not None:
            all_x.append(self.target_point[0])
            all_y.append(self.target_point[1])
            all_z.append(self.target_point[2])
        
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
        self.update_limits_display(angles)
    
    def reset_joints(self):
        """Сброс углов в нулевое положение"""
        for entry in self.angle_entries:
            entry.delete(0, tk.END)
            entry.insert(0, "0")
        self.current_angles = [0.0] * 6
        self.target_point = None
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
        
        # Интерполяция УГЛОВ
        current_angles = []
        for i in range(6):
            angle = self.start_angles[i] + t * (self.end_angles[i] - self.start_angles[i])
            # Применяем ограничения к промежуточным углам
            angle = self.clamp_angle(angle, i)
            current_angles.append(angle)
        
        current_joints = self.calculate_forward_kinematics(
            current_angles, self.dh_params, self.wf_params, self.tf_params
        )
        
        self.draw_manipulator(current_joints)
        self.update_position_labels(current_joints)
        self.update_limits_display(current_angles)
        
        self.current_frame += 1
        self.anim_id = self.root.after(self.speed, self.animate_step)

if __name__ == "__main__":
    root = tk.Tk()
    app = ManipulatorVisualizer(root)
    root.mainloop()
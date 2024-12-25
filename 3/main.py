import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, PathPatch
from matplotlib.animation import FuncAnimation
from matplotlib.path import Path

# Параметры системы
m1 = 5.0    # масса бруса 1 (кг)
m2 = 1.0    # масса цилиндра 2 (кг)
R = 0.3     # радиус выточки (м)
r = 0.04    # радиус цилиндра (м)
c = 25.0    # жесткость пружины (Н/м)
k = 5.0     # коэффициент сопротивления линейного движения (Н⋅с/м)
k_rot = 0.05  # коэффициент сопротивления вращательного движения (Н⋅м⋅с/рад)
F0 = 1.0    # амплитуда внешней силы (Н)
p = 0.4     # частота внешней силы (1/с)
g = 9.81    # ускорение свободного падения (м/с²)

# Начальные условия
z0 = 0.1        # начальное положение (м)
z_dot0 = 0.0    # начальная скорость (м/с)
phi0 = -np.pi/2  # начальный угол (рад)
phi_dot0 = 0.0  # начальная угловая скорость (рад/с)

def system_equations(t, y):
    """
    Система дифференциальных уравнений
    y = [z, z_dot, phi, phi_dot]
    """
    z, z_dot, phi, phi_dot = y
    
    # Внешние силы
    F_ext = F0 * np.sin(p * t)
    
    # Уравнения с учетом вращательного трения
    # (m1 + m2)z̈ - m2(R-r)[φ̈cosφ - φ̇²sinφ] + kż + cz = F0sin(pt)
    # z̈cosφ + (3/2)(R-r)φ̈ + gsinφ + k_rot*φ̇/(m2*(R-r)) = 0
    
    A = np.array([
        [m1 + m2, -m2*(R-r)*np.cos(phi)],
        [np.cos(phi), 1.5*(R-r)]
    ])
    
    b = np.array([
        [m2*(R-r)*phi_dot**2*np.sin(phi) - k*z_dot - c*z + F_ext],
        [-g*np.sin(phi) - k_rot*phi_dot/(m2*(R-r))]
    ])
    
    try:
        z_ddot, phi_ddot = np.linalg.solve(A, b).flatten()
    except np.linalg.LinAlgError:
        z_ddot, phi_ddot = 0, 0  # Запасной вариант для особой матрицы
        
    return [z_dot, z_ddot, phi_dot, phi_ddot]

# Параметры времени
t_start = 0
t_end = 10
dt = 0.05
t_eval = np.arange(t_start, t_end, dt)

# Решаем систему дифференциальных уравнений
solution = solve_ivp(
    system_equations,
    [t_start, t_end],
    [z0, z_dot0, phi0, phi_dot0],
    t_eval=t_eval,
    method='RK45'
)

# Извлекаем решения
t = solution.t
z = solution.y[0]
z_dot = solution.y[1]
phi = solution.y[2]
phi_dot = solution.y[3]

def calculate_forces(t_idx):
    """Вычисляем все силы в системе в момент времени t_idx"""
    # Получаем текущее состояние
    z_current = z[t_idx]
    z_dot_current = z_dot[t_idx]
    phi_current = phi[t_idx]
    phi_dot_current = phi_dot[t_idx]
    t_current = t[t_idx]
    
    # Внешние силы
    F_ext = F0 * np.sin(p * t_current)
    F_spring = -c * z_current
    F_damp = -k * z_dot_current
    
    # Получаем ускорения
    _, z_ddot, _, phi_ddot = system_equations(
        t_current, 
        [z_current, z_dot_current, phi_current, phi_dot_current]
    )
    
    # Силы трения и нормальная реакция
    F_tr = m2 * (z_ddot * np.cos(phi_current) + 
                 (R-r) * phi_ddot + 
                 g * np.sin(phi_current))
    
    P = m2 * (-z_ddot * np.sin(phi_current) + 
              (R-r) * phi_dot_current**2 + 
              g * np.cos(phi_current))
    
    # Нормальная сила
    N = ((m1 + m2)*g + 
         m2*(R-r)*(phi_dot_current*np.sin(phi_current) + 
                   phi_ddot*np.cos(phi_current)))
    
    return F_ext, F_spring, F_damp, F_tr, P, N

def create_block_with_cutout(x_pos):
    # Создаем контур бруса с выточкой
    verts = [
        # Начинаем с нижнего левого угла, идем против часовой стрелки
        (x_pos - block_width/2, -block_height/2),  # Нижний левый угол
        (x_pos - block_width/2, block_height/2),   # Верхний левый угол
        (x_pos - R, block_height/2),              # Левый край выточки
        (x_pos - R, cutout_center_y),             # Начало выточки
    ]
    codes = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO]
    
    # Добавляем полукруглую выточку
    theta = np.linspace(-np.pi, 0, 30)
    arc_x = x_pos + R * np.cos(theta)
    arc_y = cutout_center_y + R * np.sin(theta)
    
    for i in range(len(theta)):
        verts.append((arc_x[i], arc_y[i]))
        codes.append(Path.LINETO)
    
    # Завершаем контур бруса
    verts.extend([
        (x_pos + R, block_height/2),              # Правый край выточки
        (x_pos + block_width/2, block_height/2),   # Верхний правый угол
        (x_pos + block_width/2, -block_height/2),  # Нижний правый угол
        (x_pos - block_width/2, -block_height/2),  # Возврат к началу
    ])
    codes.extend([Path.LINETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY])
    
    return Path(verts, codes)

fig = plt.figure(figsize=(10, 6))
ax1 = fig.add_subplot(111)

ax1.set_xlim(-1, 1)
ax1.set_ylim(-0.5, 0.5)
ax1.set_aspect('equal')
ax1.grid(True)
ax1.set_title('Механическая система')

fig_forces = plt.figure(figsize=(10, 8))
ax_forces = []
force_names = ['Сила пружины', 'Сила сопротивления', 'Сила трения']
force_colors = ['blue', 'green', 'purple']

for i, (name, color) in enumerate(zip(force_names, force_colors)):
    ax = fig_forces.add_subplot(3, 1, i+1)
    ax.grid(True)
    ax.set_xlim(t_start, t_end)
    ax.set_ylim(-10, 10)
    if i == 2:
        ax.set_xlabel('Время (с)')
    ax.set_ylabel(name + ' (Н)')
    ax_forces.append(ax)

fig_forces.tight_layout()

ax1.axhline(y=0, color='k', linestyle='-', alpha=0.2)
ax1.axvline(x=0, color='k', linestyle='-', alpha=0.2)

block_width = 0.7
block_height = 0.4
cutout_center_y = 0.15
wall_width = 0.05
wall_height = 0.6

block = PathPatch(create_block_with_cutout(0), facecolor='lightgray', 
                 edgecolor='black')
cylinder = Circle((0, 0), r, fill=True, color='blue', alpha=0.6)
spring = plt.Line2D([], [], color='red', linewidth=2)
wall = Rectangle((-0.8 - wall_width, -wall_height/2), wall_width, wall_height,
                facecolor='gray', edgecolor='black')

force_lines = {}
for name, color, ax in zip(force_names, force_colors, ax_forces):
    line, = ax.plot([], [], color=color)
    force_lines[name] = line

def animate(frame):
    z_pos = z[frame]
    block.set_path(create_block_with_cutout(z_pos))
    
    phi_current = phi[frame]
    x_cylinder = z_pos + (R-r) * np.sin(phi_current)
    y_cylinder = cutout_center_y - (R-r) * np.cos(phi_current)
    cylinder.center = (x_cylinder, y_cylinder)
    
    spring_x = np.linspace(-0.8, z_pos-block_width/2, 20)
    spring_y = 0.02 * np.sin(4*np.pi*np.linspace(0, 1, 20))
    spring.set_data(spring_x, spring_y)
    
    current_t = t[:frame+1]
    force_lines['Сила пружины'].set_data(current_t, -c*z[:frame+1])
    force_lines['Сила сопротивления'].set_data(current_t, -k*z_dot[:frame+1])
    force_lines['Сила трения'].set_data(current_t, [calculate_forces(i)[3] for i in range(frame+1)])
    
    return (block, cylinder, spring, *force_lines.values())

ax1.add_patch(wall)
ax1.add_patch(block)
ax1.add_patch(cylinder)
ax1.add_line(spring)

anim = FuncAnimation(fig, animate, frames=len(t), interval=50, blit=True)

plt.show()
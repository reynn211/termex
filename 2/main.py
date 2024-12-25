import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Arc, PathPatch
from matplotlib.animation import FuncAnimation
from matplotlib.path import Path
import matplotlib.patches as patches

# Параметры системы
R = 0.3   # радиус выреза
r = 0.04   # радиус цилиндра

# Параметры времени
t_start = 0
t_end = 10
dt = 0.05
t = np.arange(t_start, t_end, dt)

# Предварительный расчет всех положений и сил
def z(t):
    """Положение блока 1"""
    return 0.2 * np.sin(2*t) * np.exp(-0.1*t)  # Уменьшенная амплитуда с 0.5 до 0.2

def phi(t):
    """Угловое положение цилиндра 2"""
    # Отображение колебаний между -π и 0 (нижняя половина окружности)
    return -np.pi/2 * (1 + np.cos(2*t))

# Calculate positions
z_positions = z(t)
phi_positions = phi(t)

# Create figure and axis
fig, ax = plt.subplots(figsize=(8, 4))  # Reduced figure size
ax.set_xlim(-1, 1)  # Reduced plot width
ax.set_ylim(-0.5, 0.5)  # Reduced plot height
ax.set_aspect('equal')
ax.grid(True)
ax.set_title('Механическая система')

# Create static elements
ax.axhline(y=0, color='k', linestyle='-', alpha=0.2)
ax.axvline(x=0, color='k', linestyle='-', alpha=0.2)

block_width = 0.7
block_height = 0.4
cutout_center_y = 0.2
wall_width = 0.05  # Width of the wall
wall_height = 0.6  # Height of the wall

def create_block_with_cutout(x_pos):
    # Рисуем контур блока сегментами, избегая область выреза
    verts = [
        # Начинаем с нижнего левого угла, идем против часовой стрелки
        (x_pos - block_width/2, -block_height/2),  # Нижний левый угол
        (x_pos - block_width/2, block_height/2),   # Верхний левый угол
        (x_pos - R, block_height/2),              # Левый край выреза
        (x_pos - R, cutout_center_y),             # Начало выреза
    ]
    codes = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO]
    
    # Add the semicircular cutout
    theta = np.linspace(-np.pi, 0, 30)
    arc_x = x_pos + R * np.cos(theta)
    arc_y = cutout_center_y + R * np.sin(theta)
    
    for i in range(len(theta)):
        verts.append((arc_x[i], arc_y[i]))
        codes.append(Path.LINETO)
    
    # Complete the block outline
    verts.extend([
        (x_pos + R, block_height/2),              # Right edge of cutout
        (x_pos + block_width/2, block_height/2),   # Top right
        (x_pos + block_width/2, -block_height/2),  # Bottom right
        (x_pos - block_width/2, -block_height/2),  # Back to start
    ])
    codes.extend([Path.LINETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY])
    
    return Path(verts, codes)

# Initialize objects
block = PathPatch(create_block_with_cutout(0), facecolor='lightgray', edgecolor='black')
cylinder = Circle((0, 0), r, fill=True, color='blue', alpha=0.6)
spring = plt.Line2D([], [], color='red', linewidth=2)
wall = Rectangle((-0.8 - wall_width, -wall_height/2), wall_width, wall_height, 
                facecolor='gray', edgecolor='black')

def animate(frame):
    # Обновление положения блока
    z_pos = z_positions[frame]
    block.set_path(create_block_with_cutout(z_pos))
    
    # Обновление положения цилиндра с учетом качения
    phi_current = phi_positions[frame]
    # Расчет положения по полукруговой траектории (только нижняя половина)
    x_cylinder = z_pos + (R-r) * np.cos(phi_current)
    y_cylinder = cutout_center_y + (R-r) * np.sin(phi_current)
    cylinder.center = (x_cylinder, y_cylinder)
    
    # Обновление пружины
    spring_x = np.linspace(-0.8, z_pos-block_width/2, 20)  # Уменьшенная длина пружины
    spring_y = 0.02 * np.sin(4*np.pi*np.linspace(0, 1, 20))  # Уменьшенная амплитуда пружины
    spring.set_data(spring_x, spring_y)
    
    return block, cylinder, spring

# Add objects to axis
ax.add_patch(wall)  # Add wall first so it's behind the spring
ax.add_patch(block)
ax.add_patch(cylinder)
ax.add_line(spring)

# Create animation
anim = FuncAnimation(fig, animate, frames=len(t), interval=50, blit=True)

plt.show()

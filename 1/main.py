import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sympy as sp

# Определяем символьную переменную времени
t = sp.Symbol('t')

# Задаем уравнения в полярных координатах
r = 1 + sp.sin(5*t)
phi = t + 0.3*sp.sin(30*t)

# Преобразование в декартовы координаты
x = r * sp.cos(phi)
y = r * sp.sin(phi)

# Находим производные для скорости
vx = sp.diff(x, t)
vy = sp.diff(y, t)

# Находим производные для ускорения
ax = sp.diff(vx, t)
ay = sp.diff(vy, t)

# Добавляем вычисление тангенциального ускорения
v_magnitude = sp.sqrt(vx**2 + vy**2)
tangential_acc = sp.diff(v_magnitude, t)

# Единичный вектор скорости
v_unit_x = vx / v_magnitude
v_unit_y = vy / v_magnitude

# Компоненты тангенциального ускорения
tang_acc_x = tangential_acc * v_unit_x
tang_acc_y = tangential_acc * v_unit_y

# Преобразуем символьные выражения в функции numpy
x_f = sp.lambdify(t, x, 'numpy')
y_f = sp.lambdify(t, y, 'numpy')
vx_f = sp.lambdify(t, vx, 'numpy')
vy_f = sp.lambdify(t, vy, 'numpy')
ax_f = sp.lambdify(t, ax, 'numpy')
ay_f = sp.lambdify(t, ay, 'numpy')
tang_acc_x_f = sp.lambdify(t, tang_acc_x, 'numpy')
tang_acc_y_f = sp.lambdify(t, tang_acc_y, 'numpy')

# Создаем фигуру и оси
fig, ax = plt.subplots(figsize=(10, 10))
ax.set_aspect('equal')

# Генерируем точки для построения траектории
t_points = np.linspace(0, 4*np.pi, 1000)
x_points = x_f(t_points)
y_points = y_f(t_points)

# Рисуем траекторию
ax.plot(x_points, y_points, 'b-', alpha=0.3)

# Создаем объекты для анимации
point, = ax.plot([], [], 'ro')

# Устанавливаем пределы осей
margin = 0.5
ax.set_xlim(np.min(x_points) - margin, np.max(x_points) + margin)
ax.set_ylim(np.min(y_points) - margin, np.max(y_points) + margin)

# Добавляем легенду
plt.arrow([], [], [], [], color='g', label='Скорость')
plt.arrow([], [], [], [], color='r', label='Ускорение')
plt.arrow([], [], [], [], color='blue', label='Тангенциальное ускорение')
plt.plot([], [], 'ro', label='Точка')
plt.legend()

# Функция инициализации анимации
def init():
    point.set_data([], [])
    velocity_arrow = ax.arrow(0, 0, 0, 0, color='g', width=0.02, head_width=0.1, head_length=0.1)
    acceleration_arrow = ax.arrow(0, 0, 0, 0, color='r', width=0.02, head_width=0.1, head_length=0.1)
    tangential_arrow = ax.arrow(0, 0, 0, 0, color='blue', width=0.02, head_width=0.1, head_length=0.1)
    return point, velocity_arrow, acceleration_arrow, tangential_arrow

# Функция обновления кадров анимации
def animate(frame):
    # Текущее время
    current_t = t_points[frame]
    
    # Текущие координаты
    current_x = x_f(current_t)
    current_y = y_f(current_t)
    
    # Текущие компоненты скорости и ускорения
    current_vx = vx_f(current_t)
    current_vy = vy_f(current_t)
    current_ax = ax_f(current_t)
    current_ay = ay_f(current_t)
    
    # Обновляем положение точки
    point.set_data([current_x], [current_y])
    
    # Создаем новые стрелки с обновленными параметрами
    velocity_arrow = ax.arrow(current_x, current_y, 
                            current_vx/25, current_vy/25,  # масштабируем длину векторов
                            color='g', width=0.02, 
                            head_width=0.1, head_length=0.1)
    
    acceleration_arrow = ax.arrow(current_x, current_y, 
                                current_ax/500, current_ay/500,  # масштабируем длину векторов
                                color='r', width=0.02, 
                                head_width=0.1, head_length=0.1)
    
    # Добавляем вычисление компонент тангенциального ускорения
    current_tang_acc_x = tang_acc_x_f(current_t)
    current_tang_acc_y = tang_acc_y_f(current_t)
    
    tangential_arrow = ax.arrow(current_x, current_y, 
                               current_tang_acc_x/500, current_tang_acc_y/500,  # масштабируем длину векторов
                               color='blue', width=0.02, 
                               head_width=0.1, head_length=0.1)
    
    return point, velocity_arrow, acceleration_arrow, tangential_arrow

# Создаем анимацию
anim = FuncAnimation(fig, animate, init_func=init, frames=len(t_points),
                    interval=20, blit=True) # увеличить интервал для замедления анимации

plt.grid(True)
plt.title('Движение точки по траектории')
plt.xlabel('x')
plt.ylabel('y')
plt.show()

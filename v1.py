"""
FLL ROBOT - с остановкой миссии по CENTER
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Button, Color
from pybricks.tools import wait

hub = PrimeHub()

# Отключаем стандартную остановку - будем сами обрабатывать
hub.system.set_stop_button(None)

print("Загрузка...")

left_motor = Motor(Port.A)
right_motor = Motor(Port.E, Direction.COUNTERCLOCKWISE)
motor_b = Motor(Port.B)

print("OK!")


# ═══════════════════════════════════════════════════════════════════════════════
#                         ПРОВЕРКА ОСТАНОВКИ
# ═══════════════════════════════════════════════════════════════════════════════

class StopMission(Exception):
    """Исключение для остановки миссии"""
    pass


def check_stop():
    """Проверяет нажата ли CENTER - если да, останавливает миссию"""
    if Button.CENTER in hub.buttons.pressed():
        # Останавливаем моторы
        left_motor.brake()
        right_motor.brake()
        motor_b.brake()
        # Ждём отпускания кнопки
        while hub.buttons.pressed():
            wait(20)
        # Выбрасываем исключение чтобы выйти из миссии
        raise StopMission()


# ═══════════════════════════════════════════════════════════════════════════════
#                         ФУНКЦИИ ДВИЖЕНИЯ
# ═══════════════════════════════════════════════════════════════════════════════

def map_value(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def get_trapezoid_speed(progress, total, accel, decel, min_speed, max_speed, end_speed):
    if progress < accel:
        return map_value(progress, 0, accel, min_speed, max_speed)
    elif progress > total - decel:
        return map_value(progress, total - decel, total, max_speed, end_speed)
    else:
        return max_speed


def gyro_straight(distance_degrees, speed=300, gain=3.0):
    hub.imu.reset_heading(0)
    left_motor.reset_angle(0)
    
    while abs(left_motor.angle()) < distance_degrees:
        check_stop()  # Проверка остановки
        
        heading = hub.imu.heading()
        correction = heading * -1 * gain
        left_motor.run(speed - correction)
        right_motor.run(speed + correction)
        wait(10)
    
    left_motor.brake()
    right_motor.brake()


def gyro_straight_accel(distance_degrees, accel=200, decel=200, 
                        min_speed=100, max_speed=800, end_speed=100, gain=3.0):
    hub.imu.reset_heading(0)
    left_motor.reset_angle(0)
    
    while abs(left_motor.angle()) < distance_degrees:
        check_stop()  # Проверка остановки
        
        progress = abs(left_motor.angle())
        speed = get_trapezoid_speed(progress, distance_degrees, accel, decel, min_speed, max_speed, end_speed)
        heading = hub.imu.heading()
        correction = heading * -1 * gain
        left_motor.run(speed - correction)
        right_motor.run(speed + correction)
        wait(10)
    
    left_motor.brake()
    right_motor.brake()


def gyro_back(distance_degrees, speed=300, gain=3.0):
    hub.imu.reset_heading(0)
    left_motor.reset_angle(0)
    
    while left_motor.angle() > -distance_degrees:
        check_stop()  # Проверка остановки
        
        heading = hub.imu.heading()
        correction = heading * gain
        left_motor.run(-speed + correction)
        right_motor.run(-speed - correction)
        wait(10)
    
    left_motor.brake()
    right_motor.brake()


def gyro_back_accel(distance_degrees, accel=200, decel=200,
                    min_speed=100, max_speed=800, end_speed=100, gain=3.0):
    hub.imu.reset_heading(0)
    left_motor.reset_angle(0)
    
    while left_motor.angle() > -distance_degrees:
        check_stop()  # Проверка остановки
        
        progress = abs(left_motor.angle())
        speed = get_trapezoid_speed(progress, distance_degrees, accel, decel, min_speed, max_speed, end_speed)
        heading = hub.imu.heading()
        correction = heading * gain
        left_motor.run(-speed + correction)
        right_motor.run(-speed - correction)
        wait(10)
    
    left_motor.brake()
    right_motor.brake()


def gyro_turn(target_angle, accuracy=2):
    hub.imu.reset_heading(0)
    
    while True:
        check_stop()  # Проверка остановки
        
        current = hub.imu.heading()
        error = target_angle - current
        
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360
        
        if abs(error) <= accuracy:
            break
        
        if abs(error) > 30:
            speed = 300
        else:
            speed = abs(error) * 2.5
            if speed < 35:
                speed = 35
        
        if error > 0:
            left_motor.run(-speed)
            right_motor.run(speed)
        else:
            left_motor.run(speed)
            right_motor.run(-speed)
        
        wait(10)
    
    left_motor.stop()
    right_motor.stop()
    wait(50)


def drift(duration_ms, turn_rate=0.5, speed=300, backward=False):
    direction = -1 if backward else 1
    
    if turn_rate >= 0:
        left_speed = direction * speed
        right_speed = direction * speed * (1 - turn_rate)
    else:
        left_speed = direction * speed * (1 + turn_rate)
        right_speed = direction * speed
    
    left_motor.run(left_speed)
    right_motor.run(right_speed)
    
    # Проверка остановки во время drift
    timer = 0
    while timer < duration_ms:
        check_stop()
        wait(10)
        timer += 10
    
    left_motor.brake()
    right_motor.brake()


def rotate(motor, target_angle, min_speed=50, max_speed=1000):
    motor.reset_angle(0)
    total_distance = abs(target_angle)
    direction = 1 if target_angle > 0 else -1
    
    while True:
        check_stop()  # Проверка остановки
        
        current_angle = motor.angle()
        
        if (direction == 1 and current_angle >= target_angle) or \
           (direction == -1 and current_angle <= target_angle):
            break
        
        traveled = abs(current_angle)
        if total_distance > 0:
            progress = traveled / total_distance
            speed = min_speed + (max_speed - min_speed) * progress
        else:
            speed = min_speed
        
        motor.run(direction * speed)
        wait(10)
    
    motor.hold()


# ═══════════════════════════════════════════════════════════════════════════════
#                              МИССИИ
# ═══════════════════════════════════════════════════════════════════════════════

def mission_1():
    gyro_straight_accel(1440, accel=200, decel=300, min_speed=100, max_speed=1000, end_speed=80, gain=5.0)
    gyro_turn(-50, accuracy=3)
    gyro_straight_accel(340, accel=200, decel=300, min_speed=100, max_speed=1000, end_speed=80, gain=5.0)
    gyro_turn(6, accuracy=3)
    gyro_turn(-5, accuracy=3)
    rotate(motor_b, -70, min_speed=50, max_speed=1000)
    gyro_back(200, speed=100, gain=1.0)
    gyro_turn(50, accuracy=3)
    gyro_back_accel(650, accel=100, decel=100, min_speed=80, max_speed=700, end_speed=60)
    gyro_turn(85, accuracy=3)
    gyro_back(150, speed=1000, gain=5.0)
    drift(700, -3, 300, True)
    wait(300)
    gyro_turn(100    , accuracy=3)
    gyro_straight_accel(600, accel=200, decel=300, min_speed=100, max_speed=700, end_speed=80, gain=5.0)
    gyro_turn(-110, accuracy=3)
    gyro_straight_accel(600, accel=20, decel=30, min_speed=100, max_speed=700, end_speed=80, gain=5.0)
    rotate(motor_b, 60, min_speed=50, max_speed=1000)
    wait(500)
    gyro_back(770, speed=1000, gain=5.0)


def mission_2():
    gyro_straight_accel(1400, accel=200, decel=300, min_speed=100, max_speed=700, end_speed=80, gain=4.0)
    gyro_turn(5, accuracy=2)
    gyro_back_accel(1400, accel=100, decel=150, min_speed=80, max_speed=700, end_speed=60)


def mission_3():
    gyro_straight_accel(1400, accel=200, decel=300, min_speed=60, max_speed=500, end_speed=20, gain=6.0)
    # gyro_turn(180, accuracy=3)
    # gyro_straight(500, speed=400, gain=3.0)


def mission_4():
    pass


def mission_5():
    pass


def mission_6():
    pass


def mission_7():
    pass


def mission_8():
    pass


missions = [mission_1, mission_2, mission_3, mission_4, mission_5, mission_6, mission_7, mission_8]
current = 0


# ═══════════════════════════════════════════════════════════════════════════════
#                           МЕНЮ
# ═══════════════════════════════════════════════════════════════════════════════

def show_num():
    num = current + 1
    if num <= 9:
        hub.display.char(str(num))
    else:
        hub.display.char(str(num % 10))


hub.light.on(Color.BLUE)
show_num()
hub.speaker.beep(800, 100)

while True:
    # Ждём отпускания кнопок
    while hub.buttons.pressed():
        wait(20)
    
    # Ждём нажатия
    pressed = set()
    while not pressed:
        pressed = hub.buttons.pressed()
        wait(20)
    
    wait(100)
    
    if Button.LEFT in pressed:
        current = (current - 1) % len(missions)
        show_num()
        
    elif Button.RIGHT in pressed:
        current = (current + 1) % len(missions)
        show_num()
        
    elif Button.CENTER in pressed:
        hub.light.on(Color.GREEN)
        hub.speaker.beep(600, 100)
        
        try:
            missions[current]()
            # Успех
            hub.speaker.beep(1000, 200)
        except StopMission:
            # Остановлено пользователем - оранжевый сигнал
            hub.light.on(Color.ORANGE)
            hub.speaker.beep(500, 300)
            wait(300)
        except:
            # Другая ошибка
            hub.speaker.beep(200, 500)
        
        hub.light.on(Color.BLUE)
        show_num()
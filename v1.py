"""
FLL ROBOT - с остановкой миссии по CENTER
"""

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Direction, Button, Color
from pybricks.tools import wait
from pybricks.pupdevices import ColorSensor

hub = PrimeHub()

# Отключаем стандартную остановку - будем сами обрабатывать
hub.system.set_stop_button(None)

print("Загрузка...")

left_motor = Motor(Port.A)
right_motor = Motor(Port.E, Direction.COUNTERCLOCKWISE)
motor_b = Motor(Port.B)
motor_f = Motor(Port.F)

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

def gyro_straight_with_motor(distance_degrees, motor, motor_angle, 
                              speed=300, gain=3.0, motor_speed=500):
    """
    Едет прямо И одновременно вращает мотор
    
    Args:
        distance_degrees: Дистанция движения (градусы колёс)
        motor: Какой мотор вращать (motor_b или другой)
        motor_angle: На сколько градусов повернуть мотор
        speed: Скорость движения
        gain: Коэффициент гироскопа
        motor_speed: Скорость вращения мотора
    
    Пример:
        gyro_straight_with_motor(500, motor_b, 180)  # Едет и поднимает руку
    """
    hub.imu.reset_heading(0)
    left_motor.reset_angle(0)
    
    # Запускаем мотор БЕЗ ожидания (работает в фоне)
    motor.run_angle(motor_speed, motor_angle, wait=False)
    
    while abs(left_motor.angle()) < distance_degrees:
        check_stop()
        
        heading = hub.imu.heading()
        correction = heading * -1 * gain
        left_motor.run(speed - correction)
        right_motor.run(speed + correction)
        wait(10)
    
    left_motor.brake()
    right_motor.brake()
    
    # Ждём пока мотор закончит (если ещё не закончил)
    while not motor.done():
        check_stop()
        wait(10)


def gyro_back_with_motor(distance_degrees, motor, motor_angle,
                          speed=300, gain=3.0, motor_speed=500):
    """
    Едет назад И одновременно вращает мотор
    
    Пример:
        gyro_back_with_motor(500, motor_b, -90)  # Едет назад и опускает руку
    """
    hub.imu.reset_heading(0)
    left_motor.reset_angle(0)
    
    motor.run_angle(motor_speed, motor_angle, wait=False)
    
    while left_motor.angle() > -distance_degrees:
        check_stop()
        
        heading = hub.imu.heading()
        correction = heading * gain
        left_motor.run(-speed + correction)
        right_motor.run(-speed - correction)
        wait(10)
    
    left_motor.brake()
    right_motor.brake()
    
    while not motor.done():
        check_stop()
        wait(10)


def gyro_turn_with_motor(target_angle, motor, motor_angle,
                          accuracy=2, motor_speed=500):
    """
    Поворачивает И одновременно вращает мотор
    
    Пример:
        gyro_turn_with_motor(90, motor_b, 180)  # Поворот + поднять руку
    """
    hub.imu.reset_heading(0)
    
    motor.run_angle(motor_speed, motor_angle, wait=False)
    
    while True:
        check_stop()
        
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
    
    while not motor.done():
        check_stop()
        wait(10)


def move_both_motors(motor1, angle1, motor2, angle2, speed1=500, speed2=500):
    """
    Вращает ДВА мотора одновременно
    
    Args:
        motor1: Первый мотор
        angle1: Угол первого мотора
        motor2: Второй мотор  
        angle2: Угол второго мотора
        speed1, speed2: Скорости
    
    Пример:
        move_both_motors(motor_b, 180, motor_f, -90)  # Оба мотора сразу
    """
    motor1.run_angle(speed1, angle1, wait=False)
    motor2.run_angle(speed2, angle2, wait=False)
    
    # Ждём пока оба закончат
    while not motor1.done() or not motor2.done():
        check_stop()
        wait(10)


def start_motor(motor, angle, speed=500):
    """
    Запускает мотор в фоне (не ждёт завершения)
    
    Пример:
        start_motor(motor_b, 180)
        gyro_straight(500)  # Едет пока мотор крутится
        wait_motor(motor_b)  # Ждёт завершения
    """
    motor.run_angle(speed, angle, wait=False)


def wait_motor(motor):
    """Ждёт пока мотор закончит вращение"""
    while not motor.done():
        check_stop()
        wait(10)


# ═══════════════════════════════════════════════════════════════════════════════
#                      ВЫРАВНИВАНИЕ ПО ЛИНИИ
# ═══════════════════════════════════════════════════════════════════════════════

sensor_left = ColorSensor(Port.D) 
sensor_right = ColorSensor(Port.C)
BLACK = 15    
WHITE = 70

def align_two_sensors(sensor_left, sensor_right, speed=100, timeout=3000):
    """
    Выравнивание по линии двумя датчиками
    Чёрное = меньше 20
    """
    BLACK_THRESHOLD = 20
    
    left_count = 0
    right_count = 0
    CONFIRM_COUNT = 3
    
    left_done = False
    right_done = False
    timer = 0
    
    while timer < timeout:
        check_stop()
        
        left_val = sensor_left.reflection()
        right_val = sensor_right.reflection()
        
        if left_val < BLACK_THRESHOLD:
            left_count += 1
            if left_count >= CONFIRM_COUNT:
                left_done = True
        else:
            left_count = 0
        
        if right_val < BLACK_THRESHOLD:
            right_count += 1
            if right_count >= CONFIRM_COUNT:
                right_done = True
        else:
            right_count = 0
        
        if left_done and right_done:
            break
        
        if left_done:
            left_motor.brake()
        else:
            left_motor.run(speed)
        
        if right_done:
            right_motor.brake()
        else:
            right_motor.run(speed)
        
        wait(10)
        timer += 10
    
    left_motor.brake()
    right_motor.brake()


def align_two_sensors_back(sensor_left, sensor_right):
    """
    Выравнивание по линии НАЗАД двумя датчиками
    """
    left_on_line = False
    right_on_line = False
    timer = 0
    
    while timer < timeout:
        check_stop()
        
        left_val = sensor_left.reflection()
        right_val = sensor_right.reflection()
        
        if left_val < BLACK + 10:
            left_on_line = True
        
        if right_val < BLACK + 10:
            right_on_line = True
        
        if left_on_line and right_on_line:
            break
        
        if left_on_line:
            left_motor.brake()
        else:
            left_motor.run(-speed)
        
        if right_on_line:
            right_motor.brake()
        else:
            right_motor.run(-speed)
        
        wait(10)
        timer += 10
    
    left_motor.brake()
    right_motor.brake()


# ═══════════════════════════════════════════════════════════════════════════════
#                              МИССИИ
# ═══════════════════════════════════════════════════════════════════════════════

def mission_1():
    gyro_straight_accel(1400, accel=200, decel=300, min_speed=100, max_speed=1000, end_speed=80, gain=5.0)
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
    gyro_turn(100, accuracy=3)
    gyro_straight_accel(600, accel=200, decel=300, min_speed=100, max_speed=700, end_speed=80, gain=5.0)
    gyro_turn(-105, accuracy=3)
    gyro_straight_accel(600, accel=20, decel=30, min_speed=100, max_speed=700, end_speed=80, gain=5.0)
    rotate(motor_b, 60, min_speed=50, max_speed=1000)
    wait(500)
    gyro_back(470, speed=1000, gain=5.0)
    gyro_turn(40, accuracy=3)
    gyro_back(470, speed=1000, gain=5.0)






def mission_2():
    pass


def mission_3():
    # gyro_straight_accel(1100, accel=200, decel=200, min_speed=150, max_speed=650, end_speed=80, gain=5.0)
    # gyro_back_accel(160, accel=100, decel=100, min_speed=80, max_speed=800, end_speed=60)
    # gyro_turn(-40, accuracy=2)
    # gyro_straight_accel(170, accel=200, decel=140, min_speed=150, max_speed=700, end_speed=80, gain=5.0)
    # drift(450, -3, 900, False)
    # wait(100)
    # gyro_turn(-65, accuracy=2)
    # gyro_straight_accel(230, accel=100, decel=100, min_speed=150, max_speed=900, end_speed=80, gain=5.0)
    # gyro_turn(45, accuracy=2)
    # gyro_straight(80, 600, 5.0)
    # align_two_sensors(sensor_left, sensor_right)
    # #///////////////////////
    # wait(200)
    # gyro_back(50, 900, 4.0)
    # gyro_turn(25)
    # gyro_back(170, 900, 4.0)
    # rotate(motor_f, 90, 50, 700)
    # wait(100)
    # gyro_turn(30, 2)
    # gyro_straight(50, 400, 4.0)
    # gyro_turn(-45)
    # gyro_straight(140, 900, 4.0)
    # align_two_sensors(sensor_left, sensor_right)
    #////////////////////////
    wait(200)
    gyro_turn(25)
    gyro_straight_accel(300, accel=100, decel=150, min_speed=80, max_speed=700, end_speed=60)


    gyro_turn(-55, 2)
    gyro_back(190, 400, 4.0)
    rotate(motor_b, 3000, 700, 1000)
    rotate(motor_b, -500, 900, 1000)


    gyro_turn(-70, 2)
    gyro_straight_accel(100, accel=20, decel=20, min_speed=80, max_speed=900, end_speed=60)


    
    
    


def mission_4():
    rotate(motor_b, -36, min_speed=50, max_speed=100)
    gyro_straight_accel(195, accel=200, decel=100, min_speed=100, max_speed=1000, end_speed=80, gain=5.0)
    gyro_turn(-45, accuracy=2)
    gyro_straight_accel(490, accel=200, decel=100, min_speed=100, max_speed=600, end_speed=80, gain=5.0)
    rotate(motor_b, 70, min_speed=50, max_speed=1000)
    gyro_back(420, speed=500, gain=5.0)
    wait(200)
    # rotate(motor_b, -90, min_speed=50, max_speed=700)
    # wait(200)
    # gyro_back(420, speed=500, gain=5.0)
    rotate(motor_b, -90, min_speed=50, max_speed=700)
    gyro_straight_accel(80, accel=200, decel=100, min_speed=100, max_speed=500, end_speed=80, gain=5.0)

    gyro_straight_accel(230, accel=20, decel=10, min_speed=100, max_speed=1000, end_speed=80, gain=5.0)
    gyro_turn(-20, accuracy=2)
    gyro_turn(20, accuracy=2)
    
    


def mission_5():
    gyro_straight_accel(810, accel=40, decel=100, min_speed=100, max_speed=700, end_speed=80, gain=5.0)
    # Tap-tap-tap
    rotate(motor_b, 120, min_speed=50, max_speed=1000)
    wait(200)
    rotate(motor_b, -120, min_speed=50, max_speed=500)
    wait(200)
    rotate(motor_b, 120, min_speed=50, max_speed=1000)
    wait(200)
    rotate(motor_b, -120, min_speed=50, max_speed=500)
    wait(200)
    rotate(motor_b, 120, min_speed=50, max_speed=1000)
    wait(200)
    rotate(motor_b, -120, min_speed=50, max_speed=500)


    gyro_back(120, speed=1000, gain=5.0)
    gyro_turn(-90, accuracy=2)
    drift(850, -0.8, 900, False)
    wait(200)
    gyro_straight_accel(550, accel=40, decel=100, min_speed=100, max_speed=900, end_speed=80, gain=5.0)
    rotate(motor_b, 150, min_speed=50, max_speed=1000)
    wait(200)
    gyro_turn(-30, accuracy=2)
    gyro_turn(100, accuracy=2)

    



    # gyro_back_accel(1400, accel=100, decel=150, min_speed=80, max_speed=700, end_speed=60)




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

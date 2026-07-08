from machine import Pin, PWM, ADC, UART
import time

# =====================================================
# V2 Competition Framework
# 五路 + 编码器 + IMU
# =====================================================

# -----------------------------
# PWM
# -----------------------------
PWM_FREQ = 20000

# 左电机
pwm_m1_in1 = PWM(Pin(13), freq=PWM_FREQ, duty=0)
pwm_m1_in2 = PWM(Pin(15), freq=PWM_FREQ, duty=0)

# 右电机
pwm_m2_in1 = PWM(Pin(14), freq=PWM_FREQ, duty=0)
pwm_m2_in2 = PWM(Pin(25), freq=PWM_FREQ, duty=0)

# =====================================================
# UART
# RX22 <- S3 TX45
# TX23 -> S3 RX46
# =====================================================

uart = UART(
    2,
    baudrate=115200,
    tx=Pin(23),
    rx=Pin(22)
)

# =====================================================
# IMU  参数
# =====================================================
rx_buffer = b""
imu_online = False

imu_seq = 0
imu_gyro = 0.0
imu_yaw = 0.0

last_recv_time = time.ticks_ms()

IMU_TIMEOUT = 100

# =============================
# IMU YAW Stabilize
# =============================

IMU_GYRO_K = 0.02

YAW_K = 0.05

yaw_ref = 0
yaw_locked = False
TURN_ANGLE = 90

# =====================================================
# Line PID  参数
# =====================================================

THRESHOLD = 1000

WEIGHTS = [3, 2, 0, -2, -3]

KP = 35
KI = 0.1
KD = 140

last_error = 0.0
integral = 0.0
turn_strength = 0.0

last_turn = 0
lost_start = None

# =====================================================
# Dynamic Speed  参数
# =====================================================

base_speed = 68

# =====================================================
# Speed PID  参数
# =====================================================

KP_S = 1.6
KI_S = 0.13
KD_S = 0.05

last_e_l = 0
last_e_r = 0

int_l = 0
int_r = 0

# =====================================================
# Encoder
# =====================================================

enc_l = Pin(16, Pin.IN)
enc_r = Pin(18, Pin.IN)

encoder_l = 0
encoder_r = 0

last_time_l = 0
last_time_r = 0


def enc_l_irq(pin):
    global encoder_l
    global last_time_l

    now = time.ticks_us()

    if time.ticks_diff(now, last_time_l) > 300:
        encoder_l += 1
        last_time_l = now


def enc_r_irq(pin):
    global encoder_r
    global last_time_r

    now = time.ticks_us()

    if time.ticks_diff(now, last_time_r) > 300:
        encoder_r += 1
        last_time_r = now


enc_l.irq(trigger=Pin.IRQ_RISING, handler=enc_l_irq)
enc_r.irq(trigger=Pin.IRQ_RISING, handler=enc_r_irq)


def get_speed_l():
    global encoder_l

    speed = encoder_l
    encoder_l = 0

    return speed


def get_speed_r():
    global encoder_r

    speed = encoder_r
    encoder_r = 0

    return speed


# =====================================================
# ADC
# =====================================================

adc1 = ADC(Pin(27))
adc2 = ADC(Pin(33))
adc3 = ADC(Pin(32))
adc4 = ADC(Pin(35))
adc5 = ADC(Pin(34))

for adc in (adc1, adc2, adc3, adc4, adc5):

    adc.atten(ADC.ATTN_11DB)

    adc.width(ADC.WIDTH_12BIT)


# =====================================================
# Motor
# =====================================================

def set_motor1_speed(speed):

    speed = max(-100, min(100, speed))

    if speed > 0:
        pwm_m1_in1.duty(int(speed * 1023 / 100))
        pwm_m1_in2.duty(0)

    elif speed < 0:
        pwm_m1_in1.duty(0)
        pwm_m1_in2.duty(int(-speed * 1023 / 100))

    else:
        pwm_m1_in1.duty(0)
        pwm_m1_in2.duty(0)


def set_motor2_speed(speed):

    speed = max(-100, min(100, speed))

    if speed > 0:
        pwm_m2_in1.duty(int(speed * 1023 / 100))
        pwm_m2_in2.duty(0)

    elif speed < 0:
        pwm_m2_in1.duty(0)
        pwm_m2_in2.duty(int(-speed * 1023 / 100))

    else:
        pwm_m2_in1.duty(0)
        pwm_m2_in2.duty(0)


def stop():

    set_motor1_speed(0)
    set_motor2_speed(0)


# =====================================================
# IMU Receive
# =====================================================

def receive_imu():

    global imu_online
    global imu_seq
    global imu_gyro
    global imu_yaw
    global last_recv_time
    global rx_buffer

    # 一次最多处理一帧
    if not uart.any():
        return

    data = uart.read()

    if data is None:
        return

    rx_buffer += data

    # 找完整一帧
    start = rx_buffer.find(b"<")
    end = rx_buffer.find(b">")

    if start == -1 or end == -1 or end < start:
        return

    frame = rx_buffer[start + 1:end]

    # 删除已经解析的数据
    rx_buffer = rx_buffer[end + 1:]

    try:

        seq, gyro, yaw = frame.decode().split(",")

        imu_seq = int(seq)

        imu_gyro = int(gyro) / 100.0

        imu_yaw = int(yaw) / 100.0

        imu_online = True

        last_recv_time = time.ticks_ms()

    except:

        pass

def imu_alive():

    global imu_online

    if time.ticks_diff(
        time.ticks_ms(),
        last_recv_time
    ) > IMU_TIMEOUT:

        imu_online = False
        
        
# =====================================================
# Sensor
# =====================================================

def read_sensor():

    return [
        adc1.read(),
        adc2.read(),
        adc3.read(),
        adc4.read(),
        adc5.read()
    ]


# =====================================================
# Calculate Error
# =====================================================

def calculate_error(values):

    total = 0
    count = 0

    for value, weight in zip(values, WEIGHTS):

        if value < THRESHOLD:

            total += weight
            count += 1

    # 五路全白（丢线）
    if count == 5:
        return None

    return total / count


# =====================================================
# Line PID
# =====================================================

def line_pid(error):

    global last_error
    global integral
    global last_turn
    global correction

    if error < 0:
        last_turn = 1

    elif error > 0:
        last_turn = -1

    integral += error
    integral = max(-15, min(15, integral))

    derivative = error - last_error

    correction = (
        KP * error +
        KI * integral +
        KD * derivative
    )

    last_error = error

    return correction, derivative


# =====================================================
# IMU Stabilize
# 第一版：仅直线辅助
# =====================================================

def imu_stabilize(error):
    global yaw_ref


    if not imu_online:
        return 0


    # =========================
    # 弯道关闭YAW
    # =========================

    if abs(turn_strength) > 5:

        return 0



    # =========================
    # YAW保持
    # =========================

    yaw_err = yaw_error()
    yaw_corr = -YAW_K * yaw_err



    # =========================
    # gyro阻尼
    # =========================
    gyro_corr = -IMU_GYRO_K * imu_gyro



    correction = (
        yaw_corr +
        gyro_corr
    )


    correction = max(
        -6,
        min(6, correction)
    )

    return correction


def yaw_error():
    error = imu_yaw - yaw_ref

    if error > 180:
        error -= 360


    elif error < -180:
        error += 360

    return error


# =====================================================
# Dynamic Speed
# =====================================================

def speed_strategy(error, derivative):

    global base_speed

    turn_strength = abs(correction)

    if turn_strength < 5:

        target_base = 60

    elif turn_strength < 10:

        target_base = 58

    elif turn_strength < 15:

        target_base = 55

    elif turn_strength < 25:

        target_base = 52

    else:

        target_base = 48

    # 平滑
    base_speed = (
        0.2 * base_speed +
        0.8 * target_base
    )

    return int(base_speed)

# =====================================================
# Speed PID
# =====================================================

def speed_pid_l(target, actual):

    global last_e_l
    global int_l

    e = target - actual

    int_l += e
    int_l = max(-80, min(80, int_l))

    d = e - last_e_l

    out = (
        KP_S * e +
        KI_S * int_l +
        KD_S * d
    )

    last_e_l = e

    return max(-100, min(100, out))


def speed_pid_r(target, actual):

    global last_e_r
    global int_r

    e = target - actual

    int_r += e
    int_r = max(-80, min(80, int_r))

    d = e - last_e_r

    out = (
        KP_S * e +
        KI_S * int_r +
        KD_S * d
    )

    last_e_r = e

    return max(-100, min(100, out))


# =====================================================
# Output Motor
# =====================================================

def output_motor(base, correction, gyro_corr):
    

    # IMU只负责直线微调
    correction += gyro_corr

    target_l = base + correction
    target_r = base - correction

    # 限幅
    target_l = max(-100, min(100, target_l))
    target_r = max(-100, min(100, target_r))

    # 编码器测速
    actual_l = get_speed_l()
    actual_r = get_speed_r()

    # 速度闭环
    pwm_l = speed_pid_l(target_l, actual_l)
    pwm_r = speed_pid_r(target_r, actual_r)

    # 输出
    set_motor1_speed(-pwm_l)
    set_motor2_speed(pwm_r)
    
    #print(
     #   "BASE:", round(base_speed,1),
      #  "TL:", round(target_l,1),
      #  "TR:", round(target_r,1),
       # "AL:", actual_l,
       # "AR:", actual_r,
     #   "PL:", round(pwm_l,1),
     #   "PR:", round(pwm_r,1)
    #)


# =====================================================
# Main Control
# =====================================================

def control(error):

    global lost_start
    global yaw_locked
    global yaw_ref
    
    # -----------------------------
    # 丢线
    # -----------------------------
    if error is None:

        now = time.ticks_ms()

        if lost_start is None:
            lost_start = now

        lost = time.ticks_diff(now, lost_start)

        # 第一阶段
        if lost < 250:

            correction, derivative = line_pid(last_error)

            base = speed_strategy(last_error, derivative)

            output_motor(base, correction, 0)

        # 第二阶段
        elif lost < 1000:

            if last_turn >= 0:

                set_motor1_speed(40)
                set_motor2_speed(70)

            else:

                set_motor1_speed(-70)
                set_motor2_speed(-40)

        # 第三阶段
        else:

            if last_turn >= 0:

                set_motor1_speed(50)
                set_motor2_speed(50)

            else:

                set_motor1_speed(-50)
                set_motor2_speed(-50)

        return

    # -----------------------------
    # 找到线
    # -----------------------------
    lost_start = None
    
    # =============================
    # 进入直线，锁定一次yaw
    # =============================

    if abs(turn_strength)<5:

        if not yaw_locked:

            yaw_ref = imu_yaw
            yaw_locked = True

    else:

        yaw_locked = False

    correction, derivative = line_pid(error)

    base = speed_strategy(error, derivative)

    gyro_corr = imu_stabilize(error)

    output_motor(
        base,
        correction,
        gyro_corr
    )
    
    
# =====================================================
# Cross Detection
# =====================================================

def detect_cross(values):

    black = 0

    for v in values:

        if v > THRESHOLD:
            black += 1

    return black >= 4


# =====================================================
# Right Angle Detection
# =====================================================

def detect_right_angle(values):

    global yaw_start

    line = []

    for v in values:

        if v > THRESHOLD:

            line.append(1)

        else:

            line.append(0)

    if line[0] and line[2]:

        yaw_start = imu_yaw

        return "left"

    if line[2] and line[4]:

        yaw_start = imu_yaw

        return "right"

    return None


# =====================================================
# Main
# =====================================================

print("========== V2 Competition ==========")
print("IMU Waiting...")
print("====================================")

last_debug = time.ticks_ms()

try:

    while True:

        # -----------------------------
        # IMU
        # -----------------------------
        receive_imu()

        imu_alive()

        # -----------------------------
        # Sensor
        # -----------------------------
        values = read_sensor()

        # -----------------------------
        # Cross
        # -----------------------------
        if detect_cross(values):

            integral = 0
            last_error = 0

            set_motor1_speed(-60)
            set_motor2_speed(60)

            time.sleep_ms(10)

            continue

        # -----------------------------
        # Right Angle
        # -----------------------------
        turn = detect_right_angle(values)

        if turn == "left":
            yaw_locked=False
            integral = 0
            last_error = 0

            timeout = time.ticks_ms()

            while True:

                values = read_sensor()
                # -----------------------------
                # Yaw变化
                # -----------------------------
                delta = imu_yaw - yaw_start

                if delta > 180:
                    delta -= 360

                elif delta < -180:
                    delta += 360
                
                set_motor1_speed(55)
                set_motor2_speed(55)
                
                # -----------------------------
                # Yaw完成
                # -----------------------------
                if delta <= -TURN_ANGLE:
                    break
                
                if values[2] > THRESHOLD:
                    break

                if time.ticks_diff(
                    time.ticks_ms(),
                    timeout
                ) > 500:
                    break

                receive_imu()

                time.sleep_ms(2)

            continue

        elif turn == "right":
            yaw_locked=False
            integral = 0
            last_error = 0

            timeout = time.ticks_ms()

            while True:

                values = read_sensor()
                # -----------------------------
                # Yaw变化
                # -----------------------------
                delta = imu_yaw - yaw_start

                if delta > 180:
                    delta -= 360

                elif delta < -180:
                    delta += 360
                    
                set_motor1_speed(-55)
                set_motor2_speed(-55)
                
                # -----------------------------
                # Yaw完成
                # -----------------------------
                if delta >= TURN_ANGLE:
                    break
                    
                if values[2] > THRESHOLD:
                    break
                
            
                if time.ticks_diff(
                    time.ticks_ms(),
                    timeout
                ) > 500:
                    break

                receive_imu()

                time.sleep_ms(2)

            continue

        # -----------------------------
        # Follow Line
        # -----------------------------
        error = calculate_error(values)

        control(error)

        # -----------------------------
        # Debug（可注释）
        # -----------------------------
        if time.ticks_diff(time.ticks_ms(), last_debug) >= 50:

            last_debug = time.ticks_ms()

        #print(
        #    "SEQ:", imu_seq,
         #   "GYRO:", round(imu_gyro, 1),
          #  "YAW:", round(imu_yaw, 1),
        #)

        time.sleep_ms(1)
except KeyboardInterrupt:

    stop()

    pwm_m1_in1.deinit()
    pwm_m1_in2.deinit()

    pwm_m2_in1.deinit()
    pwm_m2_in2.deinit()

    print("STOP")







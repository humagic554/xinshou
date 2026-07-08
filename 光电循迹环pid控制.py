
# ============================================
# ESP32 五路光电循迹程序（单文件版）
# 硬件：
#   左电机 PWM：GPIO13、GPIO15
#   右电机 PWM：GPIO14、GPIO25
#   五路光电 ADC：
#       adc1 -> GPIO27
#       adc2 -> GPIO33
#       adc3 -> GPIO32
#       adc4 -> GPIO35
#       adc5 -> GPIO34
#
# 已根据你的测试数据：
#   白线 ≈ 2000
#   黑线 ≈ 100
# 设置阈值 THRESHOLD = 1000
# （小于1000认为检测到黑线）
# ============================================

from machine import Pin, PWM, ADC
import time

# -----------------------------
# PWM配置
# -----------------------------
PWM_FREQ = 20000

# 左电机
pwm_m1_in1 = PWM(Pin(13), freq=PWM_FREQ, duty=0)
pwm_m1_in2 = PWM(Pin(15), freq=PWM_FREQ, duty=0)

# 右电机
pwm_m2_in1 = PWM(Pin(14), freq=PWM_FREQ, duty=0)
pwm_m2_in2 = PWM(Pin(25), freq=PWM_FREQ, duty=0)

# -----------------------------
# 电机控制
# -----------------------------
def set_motor1_speed(speed):
    # speed:-100~100
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


# -----------------------------
# ADC初始化
# -----------------------------
adc1 = ADC(Pin(27))
adc2 = ADC(Pin(33))
adc3 = ADC(Pin(32))
adc4 = ADC(Pin(35))
adc5 = ADC(Pin(34))

for adc in [adc1, adc2, adc3, adc4, adc5]:
    adc.atten(ADC.ATTN_11DB)
    adc.width(ADC.WIDTH_12BIT)

# -----------------------------
# 循迹参数
# -----------------------------
THRESHOLD = 1000      # <1000 判定为黑线   
BASE_SPEED = 64       # 基础速度
KP = 35               # 比例系数，可调15~30

# ===== 新增：PID 参数 =====
KI = 0.0              # 积分项（建议先保持0）
KD = 12.0             # 微分项（建议8~20之间调）

# 五路权重
WEIGHTS = [2, 1, 0, -1, -2]

# ===== 新增：PID状态变量 =====
last_error = 0.0
integral = 0.0


def read_sensor():
    return [
        adc1.read(),
        adc2.read(),
        adc3.read(),
        adc4.read(),
        adc5.read()
    ]


def calculate_error(values):
    """
    根据五路传感器计算误差
    """

    total = 0
    count = 0

    for value, weight in zip(values, WEIGHTS):
        if value < THRESHOLD:
            total += weight
            count += 1

    if count == 0:
        return last_error

    return total / count


def follow_line(error):
    """
    PID差速控制
    （由原来的 P 控制升级为 PID 控制）
    """
    global last_error, integral

    # ===== 新增：PID计算 =====
    integral += error                    # 积分项
    derivative = error - last_error      # 微分项

    correction = (
        KP * error +
        KI * integral +
        KD * derivative
    )

    # 保存本次误差
    last_error = error

    # ===== 保持原来的差速逻辑 =====
    left_speed = BASE_SPEED + correction
    right_speed = BASE_SPEED - correction

    left_speed = max(0, min(100, int(left_speed)))
    right_speed = max(0, min(100, int(right_speed)))

    # 根据你的电机方向：
    # 左轮前进 = 负速度
    # 右轮前进 = 正速度
    set_motor1_speed(-left_speed)
    set_motor2_speed(right_speed)
    
def detect_right_angle(values):
    """
    当连续三个传感器检测到黑线时，
    判断进入直角弯。

    返回：
        "left"  -> 左直角
        "right" -> 右直角
        None    -> 非直角
    """

    line = [1 if v > THRESHOLD else 0 for v in values]

    # 左侧2个传感器检测到黑线
    if line[0] == 1 and line[1] == 1:
        return "left"

    # 右侧2个传感器检测到黑线
    if line[3] == 1 and line[4] == 1:
        return "right"

    return None

def detect_cross(values):
    """
    五个传感器都检测到黑线，认为进入十字路口
    """
    # 黑线数量（如果你的黑线ADC比白线高，就用 > THRESHOLD）
    black_count = sum(1 for v in values if v > THRESHOLD)

    return black_count >= 4
  

# -----------------------------
# 主程序
# -----------------------------
print("开始五路循迹...")

try:

    while True:

        values = read_sensor()

             # 先判断是否进入十字路口
        cross = detect_cross(values)

        if cross:
                # ===== 新增：防止积分累积 =====
            integral = 0
            last_error = 0
            print("检测到十字路口，直行")
            set_motor1_speed(-70)
            set_motor2_speed(70)
            time.sleep_ms(300)
            continue

    
        error = calculate_error(values)
        turn = detect_right_angle(values)

            
        if turn == "left":
                # ===== 新增：防止积分累积 =====
            integral = 0
            last_error = 0
            print("检测到左直角")
            set_motor1_speed(65)
            set_motor2_speed(65)
            time.sleep_ms(50)

        elif turn == "right":
                # ===== 新增：防止积分累积 =====
            integral = 0
            last_error = 0
            print("检测到右直角")
            set_motor1_speed(-65)
            set_motor2_speed(-65)
            time.sleep_ms(50)

        else:
            # 正常循迹
            follow_line(error)
        
        
                # 调试输出
        print(
            "ADC =", values,
            " error =", error
        )

        time.sleep_ms(15)


        

except KeyboardInterrupt:

    stop()

    pwm_m1_in1.deinit()
    pwm_m1_in2.deinit()
    pwm_m2_in1.deinit()
    pwm_m2_in2.deinit()

    print("程序结束")


    
  

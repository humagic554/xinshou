
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

# =============================
# 编码器（A相测速）
# =============================


enc_l = Pin(16, Pin.IN)
enc_r = Pin(18, Pin.IN)

encoder_l = 0
encoder_r = 0

last_time_l = 0
last_time_r = 0


def enc_l_irq(pin):
    global encoder_l, last_time_l

    now = time.ticks_us()

    # 防抖（非常重要）
    if time.ticks_diff(now, last_time_l) > 300:
        encoder_l += 1
        last_time_l = now


def enc_r_irq(pin):
    global encoder_r, last_time_r

    now = time.ticks_us()

    if time.ticks_diff(now, last_time_r) > 300:
        encoder_r += 1
        last_time_r = now


# 开启中断
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

# =============================
# 速度环 PID
# =============================
KP_S = 1.6
KI_S = 0.13
KD_S = 0.05

last_e_l = 0
int_l = 0

last_e_r = 0
int_r = 0


def speed_pid_l(target, actual):
    global last_e_l, int_l

    e = target - actual
    int_l += e
    int_l = max(-80, min(80, int_l))

    d = e - last_e_l

    out = KP_S * e + KI_S * int_l + KD_S * d

    last_e_l = e

    return max(-100, min(100, out))


def speed_pid_r(target, actual):
    global last_e_r, int_r

    e = target - actual
    int_r += e
    int_r = max(-80, min(80, int_r))

    d = e - last_e_r

    out = KP_S * e + KI_S * int_r + KD_S * d

    last_e_r = e

    return max(-100, min(100, out))

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
base_speed = 68        # 基础速度
KP = 35                  # 比例系数，可调15~30

# ===== 新增：PID 参数 =====
KI = 0.1              # 积分项（建议先保持0）
KD = 100             # 微分项（建议8~20之间调）

# 五路权重
WEIGHTS = [3, 2, 0, -2, -3]

# ===== 新增：PID状态变量 =====
last_error = 0.0
integral = 0.0
turn_strength = 0.0

# ===== 丢线恢复状态 =====
last_turn = 0          # 1=左，-1=右
lost_start = None      # 开始丢线的时间


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

    if count == 5:
        return None

    return total / count


def follow_line(error):
    """
    PID差速控制
    （由原来的 P 控制升级为 PID 控制）
    """
    global last_error, integral, last_turn, base_speed 
    
    # 记录最近一次转向方向
    if error < 0:
        last_turn = 1      # 最近向左修正
    elif error > 0:
        last_turn = -1     # 最近向右修正

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
    
    # ===========================
    # 根据 correction 动态降速（推荐）
    # correction 越大说明需要转得越急
    # ===========================
    turn_strength = abs(correction)

    if turn_strength < 5:
        target_base = 79          # 全速直线

    elif turn_strength < 10:
        target_base = 78          # 轻微修正

    elif turn_strength < 15:
        target_base = 75          # 普通弯

    elif turn_strength < 20:
        target_base = 69          # 急弯

    else:
        target_base = 65          # 直角、连续弯
        
    # ===== 平滑过渡（推荐）=====
    base_speed = 0.2 * base_speed + 0.8 * target_base
    base = int(base_speed)

    target_l = base + correction
    target_r = base - correction

    #  获取实际速度（每10ms一次）
    actual_l = get_speed_l()
    actual_r = get_speed_r()

    # 速度闭环
    pwm_l = speed_pid_l(target_l, actual_l)
    pwm_r = speed_pid_r(target_r, actual_r)

    # 输出
    set_motor1_speed(-pwm_l)
    set_motor2_speed(pwm_r)
    
    ##print("turn_strength =", turn_strength)
    
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
    if line[0] == 1 and line[2] == 1:
        return "left"

    # 右侧2个传感器检测到黑线
    if line[2] == 1 and line[4] == 1:
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
            #print("检测到十字路口，直行")
            set_motor1_speed(-73)
            set_motor2_speed(73)
            time.sleep_ms(50)
            continue

    
        error = calculate_error(values)
        turn = detect_right_angle(values)

            
        if turn == "left":
            integral = 0
            last_error = 0
            #print("检测到左直角")

            timeout = time.ticks_ms()

            while True:
                values = read_sensor()

                # 中间传感器重新检测到白底（你的 error 是按白底计算）
                if values[2] > THRESHOLD:
                    break

                set_motor1_speed(70)
                set_motor2_speed(70)

                # 最多转 500ms，防止死循环
                if time.ticks_diff(time.ticks_ms(), timeout) > 500:
                    break

                time.sleep_ms(5)

        elif turn == "right":
            integral = 0
            last_error = 0
            #print("检测到右直角")

            timeout = time.ticks_ms()

            while True:
                values = read_sensor()

                if values[2] < THRESHOLD:
                    break

                set_motor1_speed(-70)
                set_motor2_speed(-70)

                if time.ticks_diff(time.ticks_ms(), timeout) > 500:
                    break

                time.sleep_ms(5)
        
        # ==========================
        # 三阶段丢线恢复策略
        # ==========================
        else:
            if error is not None:

                # 找到线，恢复正常状态
                lost_start = None

                follow_line(error)

            else:

                now = time.ticks_ms()

                # 第一次丢线
                if lost_start is None:
                    lost_start = now

                lost_time = time.ticks_diff(now, lost_start)

                # ---------- 第一阶段 ----------
                # 丢线 50ms 内：保持上一次控制
                if lost_time < 250:

                    follow_line(last_error)

                # ---------- 第二阶段 ----------
                # 50~200ms：按最后方向寻找
                elif lost_time < 1000:

                    if last_turn >= 0:
                        # 左找（缓慢左偏）
                        set_motor1_speed(55)
                        set_motor2_speed(80)
                    
                    else:
                        # 右找（缓慢右偏）
                        set_motor1_speed(-80)
                        set_motor2_speed(-55)
                    

                # ---------- 第三阶段 ----------
                # 超过200ms：原地旋转搜索
                else:

                    if last_turn >= 0:
                        # 原地逆时针
                        set_motor1_speed(65)
                        set_motor2_speed(65)
                    else:
                        # 原地顺时针
                        set_motor1_speed(-65)
                        set_motor2_speed(-65)
        
        
        # 调试输出
       # print(
        #    "ADC =", values,
         #   " error =", error,
        #)

        time.sleep_ms(10)


        

except KeyboardInterrupt:

    stop()

    pwm_m1_in1.deinit()
    pwm_m1_in2.deinit()
    pwm_m2_in1.deinit()
    pwm_m2_in2.deinit()

    print("程序结束")


    
  



  




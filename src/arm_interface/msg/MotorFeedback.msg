# MotorFeedback.msg
# 多舵机反馈消息定义（数组形式）

# 时间戳
builtin_interfaces/Time stamp

# 舵机ID数组
int32[] servo_ids

# 位置数组 (0-4095)
int32[] positions

# 速度数组 (步/秒)
int32[] speeds

# 负载数组
int32[] loads

# 电压数组 (mV)
int32[] voltages

# 温度数组 (℃)
int32[] temperatures

# 运动状态数组 (0=停止, 1=运动中)
int32[] moves

# 电流数组 (mA)
int32[] currents

# 反馈是否有效（整体标志）
bool feedback_valid
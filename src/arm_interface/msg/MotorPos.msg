# MotorPos.msg
# 可以控制单个或多个舵机

int32 servo_id          # 单个舵机ID（用于单舵机控制）
int32[] servo_ids       # 多个舵机ID数组（用于批量控制）
int32[] position        # 位置数组 (0-4095)
int32[] speed           # 速度数组
int32[] acceleration    # 加速度数组
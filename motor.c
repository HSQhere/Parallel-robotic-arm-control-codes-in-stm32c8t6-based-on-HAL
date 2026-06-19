#include "motor.h"

extern TIM_HandleTypeDef htim4;

/* ========== 全局状态变量 ========== */
static float current_alpha = 60.0f;   // 当前大臂几何角
static float current_beta  = 30.0f;  // 当前小臂内角
static float current_yaw   = 45.0f;    // 当前底座角度

/* ========== 运动规划变量 ========== */
static float start_alpha, start_beta, start_yaw;
static float delta_alpha, delta_beta, delta_yaw;
static uint32_t motion_total_steps = 0;
static uint32_t motion_step = 0;

volatile uint8_t MOTOR_MOVE_COMPLETE = 1;  // 初始已完成

/* ===========================================================
 * 初始化 PWM
 * =========================================================== */
void Motor_Init(void) {
    HAL_TIM_PWM_Start(&PWM_TIM, CH_UPPER);
    HAL_TIM_PWM_Start(&PWM_TIM, CH_LOWER);
    HAL_TIM_PWM_Start(&PWM_TIM, CH_YAW);
    HAL_TIM_PWM_Start(&PWM_TIM, CH_CLAW);
}

/* ===========================================================
 * 硬件映射（无延时）
 * @brief 将逆运动学计算出的几何角度转换为实际舵机脉宽
 * @param alpha_deg 几何算法算出的大臂与水平面夹角 (度)
 * @param beta_deg  几何算法算出的小臂与大臂之间的夹角 (度)
 * =========================================================== */
void Motor_Apply_Hardware_Mapping(float alpha_deg, float beta_deg) {
    // 1. 大臂映射：30度时135，向上抬(角度增)数值减
    float servo_upper = 130.0f - (alpha_deg - 30.0f);
    
	// 2. 小臂映射：平行舵机(180度)时180，向下摆(角度减)数值减
    // 这里的 beta_deg 是指大小臂之间的内角
    float servo_lower = 180.0f - (180.0f - beta_deg) + (alpha_deg - 30.0f); // 相对大臂角度

    // 边界保护
    if(servo_upper > 175) servo_upper = 175; if(servo_upper < 0) servo_upper = 0;
    if(servo_lower > 180) servo_lower = 180; if(servo_lower < 0) servo_lower = 0;

    // 转化为脉宽
    uint32_t pulse_upper = 500 + (uint32_t)(servo_upper * 2000 / 180);
    uint32_t pulse_lower = 500 + (uint32_t)(servo_lower * 2000 / 180);

    __HAL_TIM_SET_COMPARE(&PWM_TIM, CH_UPPER, pulse_upper);
    __HAL_TIM_SET_COMPARE(&PWM_TIM, CH_LOWER, pulse_lower);
}

/* ===========================================================
 * @brief 空间坐标解算（直接）
 * @param x 水平X
 * @param y 水平Y 
 * @param z 垂直于XY平面的高度 (30mm)
 * =========================================================== */
void Motor_Update_Space_IK(float x, float y, float z) {
    // 1. 计算水平面内的伸展距离 S
    // 此时 S 变成了由 x 和 y 决定的复合长度
    float S = sqrtf(x * x + y * y);
    float height = z;

    // 2. 计算底座 Yaw 角度
    float yaw_deg = atan2f(y, x) * 180.0f / PI;
    // 将 [-180°, +180°] 转换为 [0°, 360°]
    if (yaw_deg < 0.0f) {
        yaw_deg += 360.0f;
    }

    // 3. 逆运动学核心计算 (S, height)
    float dist_sq = S * S + height * height;
    float dist = sqrtf(dist_sq);

    if (dist > (L1 + L2) || dist < fabsf(L1 - L2)) return;

    // 余弦定理计算
    float cos_beta = (L1 * L1 + L2 * L2 - dist_sq) / (2 * L1 * L2);
    // --- 增加限位防止非法输入导致崩溃 ---
    if (cos_beta > 1.0f) cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;

    float beta_deg = acosf(cos_beta) * 180.0f / PI;

    float alpha_base = atan2f(height, S);
    float cos_alpha_tri = (L1 * L1 + dist_sq - L2 * L2) / (2 * L1 * dist);
    float alpha_deg = (alpha_base + acosf(cos_alpha_tri)) * 180.0f / PI;

    // ======= 物理约束修正 =======
    if (alpha_deg > 120.0f) alpha_deg = 120.0f;
    if (alpha_deg < 0.0f)   alpha_deg = 0.0f;

    // 保护小臂不撞大臂，内角不小于 10 度
    if (beta_deg < 10.0f)   beta_deg = 10.0f;

    // 立即更新全局角度
    current_alpha = alpha_deg;
    current_beta  = beta_deg;
    current_yaw   = yaw_deg;

    // 4. 应用硬件偏置映射
    Motor_Apply_Hardware_Mapping(alpha_deg, beta_deg);
    Motor_SetYaw(yaw_deg);
}

/* ===========================================================
 * 底座直接控制
 * =========================================================== */
void Motor_SetYaw(float angle_from_ik) { 
    // 1. 45度矫正
    // 正前方 (0°) 对应舵机的 45°
    float actual_servo_angle = 45.0f + angle_from_ik;
    
    // 2. 270度舵机的边界保护 超出360度时，取余
    if (actual_servo_angle > 270.0f && actual_servo_angle < 360.0f) actual_servo_angle = 270.0f;
    if (actual_servo_angle < 0.0f)   actual_servo_angle = 0.0f;
	if (actual_servo_angle >= 360.0f)   actual_servo_angle -= 360.0f;
    
    // 3. 计算脉宽
    uint32_t pulse = 500 + (uint32_t)(actual_servo_angle * 2000.0f / 270.0f);
    __HAL_TIM_SET_COMPARE(&PWM_TIM, CH_YAW, pulse);
}

/* ===========================================================
 * 爪子控制
 * =========================================================== */
void Motor_SetClaw(uint8_t open) {
    // 180度是张开，90度是闭合 0开 1闭
    uint32_t pulse = open ? 1500 : 2500;
    __HAL_TIM_SET_COMPARE(&PWM_TIM, CH_CLAW, pulse);
}
/* ===========================================================
 * 启动平滑运动（时间随距离自动变化）
 * =========================================================== */
void Motor_MoveTo(float x, float y, float z) {
    float S = sqrtf(x * x + y * y);
    float height = z;

    // 1. 计算目标几何角度
    float target_yaw = atan2f(y, x) * 180.0f / PI;
    if (target_yaw < 0.0f) target_yaw += 360.0f;   // 归一化到 [0, 360)
    float dist_sq = S * S + height * height;
    float dist = sqrtf(dist_sq);
    if (dist > (L1 + L2) || dist < fabsf(L1 - L2)) {
        MOTOR_MOVE_COMPLETE = 1;   // 不可达，直接报完成
        return;
    }

    float cos_beta = (L1*L1 + L2*L2 - dist_sq) / (2*L1*L2);
    if (cos_beta > 1.0f) cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;
    float target_beta = acosf(cos_beta) * 180.0f / PI;

    float alpha_base = atan2f(height, S);
    float cos_alpha_tri = (L1*L1 + dist_sq - L2*L2) / (2*L1*dist);
    float target_alpha = (alpha_base + acosf(cos_alpha_tri)) * 180.0f / PI;

    if (target_alpha > 120.0f) target_alpha = 120.0f;
    if (target_alpha < 0.0f)   target_alpha = 0.0f;
    if (target_beta < 10.0f)   target_beta = 10.0f;

    // 2. 记录起点
    start_alpha = current_alpha;
    start_beta  = current_beta;
    start_yaw   = current_yaw;

    // 3. 计算差值
    delta_alpha = target_alpha - start_alpha;
    delta_beta  = target_beta  - start_beta;
    float diff_yaw    = target_yaw   - start_yaw;
    // 判断正逆时钟方向 0-45度为逆时针对应315-360度 45-270度为顺时针
    if (diff_yaw >= 315.0f) {
        delta_yaw   = -(360.0f-diff_yaw);
    } else {
        delta_yaw   = diff_yaw ;
    }

    // 4. 根据最大角度差自动计算运动时间 (至少 300ms，每度 8ms)
    float max_deg = fabsf(delta_alpha);
    if (fabsf(delta_beta) > max_deg) max_deg = fabsf(delta_beta);
    if (fabsf(delta_yaw) > max_deg) max_deg = fabsf(delta_yaw);
    uint32_t move_time_ms = 300 + (uint32_t)(max_deg * 8);  // 可调系数

    // 5. 计算总步数（必须整除）
    motion_total_steps = move_time_ms / MOTION_PERIOD;
    if (motion_total_steps < 1) motion_total_steps = 1;
    motion_step = 0;

    MOTOR_MOVE_COMPLETE = 0;
}

/* ===========================================================
 * 周期更新函数（放在主循环中每 MOTION_PERIOD ms 调用）
 * S 形插值 + 输出
 * =========================================================== */
void Motor_Process(void) {
    if (motion_step >= motion_total_steps) {
        return;  // 运动已结束
    }

    float t = (float)motion_step / (float)motion_total_steps;
    // 正弦 S 曲线 (0→1 光滑)
    float ratio = (sinf(t * PI - PI/2.0f) + 1.0f) / 2.0f;

    current_alpha = start_alpha + delta_alpha * ratio;
    current_beta  = start_beta  + delta_beta  * ratio;
    current_yaw   = start_yaw   + delta_yaw   * ratio;
    // // 计算当前 yaw（可能暂时超出 [0,360)）
    // float yaw_temp = start_yaw + delta_yaw * ratio;
    // // 归一化到 [0, 360) 再传给 SetYaw
    // if (yaw_temp < 0.0f) yaw_temp += 360.0f;
    // if (yaw_temp >= 360.0f) yaw_temp -= 360.0f;
    // current_yaw = yaw_temp;

    motion_step++;

    Motor_Apply_Hardware_Mapping(current_alpha, current_beta);
    Motor_SetYaw(current_yaw);

    if (motion_step >= motion_total_steps) {
        MOTOR_MOVE_COMPLETE = 1;
    }
}
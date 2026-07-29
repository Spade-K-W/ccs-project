#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

/* =========================================================
 * 系统基础参数
 * ========================================================= */
#define CPU_CLOCK_HZ                (80000000U)

/* =========================================================
 * 外设电平逻辑配置
 * ========================================================= */
#define LINE_SENSOR_ACTIVE_LOW      (0U)
#define BUZZER_ACTIVE_HIGH          (1U)  /* 1=高电平发声，低电平静音 */
#define BOARD_LED_ACTIVE_HIGH       (1U)

/* =========================================================
 * 电机方向修正
 * M1/M4 与 M2/M3 机械转向相反，软件统一取反
 * ========================================================= */
#define M1_REVERSED                 (true)
#define M2_REVERSED                 (true)
#define M3_REVERSED                 (true)
#define M4_REVERSED                 (true)

/* =========================================================
 * 陀螺仪方向修正
 * ========================================================= */
#define GYRO_Z_SIGN                 (1.0f)

/* =========================================================
 * USER_KEY / 圈数设定配置
 * ========================================================= */
#define USER_KEY_ACTIVE_LOW         (1U)
#define USER_KEY_DEBOUNCE_MS        (50U)

/* 蜂鸣器短促提示时长(ms) */
#define BUZZER_SHORT_BEEP_MS        (50U)

/* 第一次按键后统计按下次数的最长窗口(ms) */
#define LAP_SET_WINDOW_MS           (2000U)

/*
 * 松手后静置该时长即确认当前次数（不必等满窗口）。
 * 避免：选完第 1 关后立刻再按一次设圈数，被当成「窗口内第 2 下 → 第 2 关」而卡死。
 */
#define KEY_CONFIRM_IDLE_MS         (800U)

/* 末弯转完进入直线循迹后，再循迹该时长再拉低 STBY(ms) */
#define LAP_FINISH_DELAY_MS         (1000U)

/* =========================================================
 * 软件 PWM 参数
 * ========================================================= */
#define SOFT_PWM_TICK_HZ            (20000U)
#define SOFT_PWM_PERIOD             (100U)

/* =========================================================
 * 主控制循环周期
 * ========================================================= */
#define LOOP_PERIOD_MS              (10U)

/* 串口调试数据发送间隔 */
#define UART_DEBUG_PRINT_INTERVAL_MS (50U)   /* 串口 0.05s 输出一次（仅转弯） */

/* 蓝牙透传（Bluetooth_UART_1：RX=PA9，TX=PB4）：主循环调用，按间隔发 OLED 六行 */
#define BLUETOOTH_ENABLE             (1U)
#define BLUETOOTH_SEND_INTERVAL_MS   (20U)
/* 上电握手：周期广播 BT_READY，收到对端 BT_READY/BT_ACK 即视为链路 OK */
#define BLUETOOTH_HANDSHAKE_RETRY_MS (500U)  /* 每轮听 500ms，避免半帧被冲掉 */

/* =========================================================
 * 视觉模块 SPI（SPI_1 = 硬件 SPI0）— TI 从机 / 泰山派主机
 *   三线无 CS：SCLK=PB18，PICO=PA14，POCI=PA13(MISO 出数)
 *   Mode0，8bit，MSB，MOTO3
 * 帧：sState:0|1,Angle:±x.xt  （0=直行，1=转弯 + 陀螺仪角）
 * 上电握手：发 Mode:N，泰山派 MOSI 回小写 ok 后再巡线（VISION_HANDSHAKE_MODE）
 * TIMG0 刷新缓冲；主机 xfer 时 SPI 中断送出
 * ========================================================= */
#define UART_VISION_ENABLE           (1U)
#define UART_VISION_SEND_INTERVAL_MS (50U)
#define SPI_VISION_MIRROR_UART0      (1U)
/* 1=把泰山派 MOSI 收到的字节打印到 UART0（跳过 0x00 填充） */
#define SPI_VISION_MIRROR_RX_UART0   (1U)
/* 上电与泰山派握手：TI 发 Mode:N，等主机 MOSI 回 ok；取值 2~5，默认 2 */
#define VISION_HANDSHAKE_MODE        (2U)

/* 编码器计数：0=GPIO 中断（推荐，正反转对称）；1=主循环轮询（仅慢速手转调试） */
#define ENC_CALIB_POLL_MODE          (0U)

/* =========================================================
 * 编码器：脉冲 ↔ 距离 换算
 *
 * 一圈滚动距离 = π × 直径
 * 每个脉冲对应距离 = π × WHEEL_DIAMETER_MM / ENC_PULSES_PER_REV
 *
 *   距离(mm) = 脉冲数 × 每脉冲距离
 *   速度(cm/s) = 距离变化 / 时间（由测速函数调用）
 *   轮直径和360°脉冲数需要实测
 * ENC_PULSES_PER_REV：手转 1 圈，OLED 第 4 行该轮读数（取绝对值）后填入
 * ========================================================= */
#define WHEEL_DIAMETER_MM           (65.0f)    /* 车轮直径 mm */
#define ENC_PULSES_PER_REV          (530.0f)   /* 实测：转 360° ≈ 530 脉冲 */
#define ENC_SPEED_FIT_K             (1.0f)
#define ENC_SPEED_FIT_B             (0.0f)

/* =========================================================
 * 速度参数（直线 / 路口原地转，不含弧线）
 * ========================================================= */
#define BASE_SPEED_STRAIGHT         (30) /* 直线锁航向 / 循迹共模速度 */

/*
 * 路口原地转（CH123/678 状态机用，与弧线四电机无关）：
 *   左转：左内侧倒退较小，右外侧前进较大
 *   右转：右内侧倒退较小，左外侧前进较大
 */
#define TURN_INNER_SPEED            (18)
#define TURN_OUTER_SPEED            (22)
#define SEARCH_SPEED_LOW            (3)  /* 丢线搜线：慢侧 PWM */
#define SEARCH_SPEED_HIGH           (8)  /* 丢线搜线：快侧 PWM */

/* 检测到 CH123/CH678 后先直行再转弯的预延时(ms) */
#define TURN_DETECT_DELAY_MS        (25U)
/* 转弯完成判定：相对进入转弯时的偏航角（度） */
#define TURN_TARGET_DEG             (83.00f)
/* 一圈所需路口次数：每次 CH123 或 CH678 全亮计 1 次，满 4 次记 1 圈 */
#define CORNERS_PER_LAP             (4U)

/* =========================================================
 * 直线段参数
 *
 * 控制方式：红外循迹为主 + 陀螺仪保持出发时方向辅助
 * 到达判断：连续 CURVE_ENTRY_HOLD_CNT 次检测到进弯特征
 * ========================================================= */

/* 直线段循迹比例系数（过大易蛇形抖动，建议 5~9） */
#define KP_LINE_STRAIGHT            (6.5f)

/* 直线误差死区：|error| 小于该值时不转向（可抑制感器微抖） */
#define LINE_ERROR_DEADZONE         (0.50f)

/* 直线误差低通滤波系数（0.2~0.5，越小越平滑、响应越慢） */
#define LINE_ERROR_FILTER_ALPHA     (0.4f)

/* 线误差 D 项系数：D = KD × d(error)/dt（error 单位约 /s，建议 0.05~0.20） */
#define KD_LINE_STRAIGHT            (0.04f)

/* 陀螺仪航向辅助系数（无线直行锁航向，建议 0.3~1.2） */
#define K_HEADING_STRAIGHT          (0.8f)

/* 航向误差死区（度）：小于该值不纠偏 */
#define HEADING_ERROR_DEADZONE_DEG  (0.8f)

/* 直线最短保护时间(ms)，防止出发时误触发到达判断 */
#define STRAIGHT_MIN_TIME_MS        (800U)

/* 进弯误差阈值：误差绝对值达到该值认为"可能进入弯道" */
#define CURVE_ENTRY_ERROR_TH        (2)

/* 连续满足多少次"可能进入弯道"才真正切换阶段 */
#define CURVE_ENTRY_HOLD_CNT        (6)
/* 直行差速限幅（过大时左右轮速跳变大，易左右甩动） */
#define STRAIGHT_DIFF_MAX           (6)

/* =========================================================
 * 编码器 + 速度内环（PWM 单位）
 * 物理左编码器=M4，物理右编码器=M1
 * ========================================================= */
/*
 * 前进为正。正转时若 Mea 为负，把对应侧 ENC_*_REVERSE 取反。
 * （左右符号相反时平均≈0，速度环会误以为停转、把 Out 顶满）
 */
#define ENC_LEFT_REVERSE            (false)
#define ENC_RIGHT_REVERSE           (true)

/* 转速低通（0.2~0.6） */
#define ENC_SPEED_FILTER_ALPHA      (0.45f)

/*
 * 实测等效 PWM（无零点偏置）：
 *   Mea = Spd / PWM_TO_SPEED_GAIN
 * 停车 Spd=0 → Mea=0，不需要另做零点标定。
 * 标定：关内环 ACCEL_LOOP_ENABLE=0，固定 Cmd 空转，
 *   令 GAIN ≈ (旧Mea × 旧GAIN) / Cmd，使新 Mea≈Cmd。
 * 本次：Cmd=25 时旧 GAIN=0.91 下 Mea≈21
 *   → GAIN = 21×0.91/25 ≈ 0.76
 */
#define PWM_TO_SPEED_GAIN           (0.76f)

/* 使能速度内环（0=只测速显示，不修正 PWM） */
#define ACCEL_LOOP_ENABLE           (1)

/*
 * 八路全亮时的动作：
 * 0 = 开环直行 BASE（过十字）
 * 1 = 停车（抱起/大黑区保护，避免空转飞车）
 */
#define LINE_ALL_ON_STOP            (1)

/*

 * 直线速度环（共模）：左右加同一 corr，保留外环差速
 * Cmd=16/16 → Out 必相等
 *直线速度环参数，需要实测微调
 */
#define KP_ACCEL_STRAIGHT           (0.30f)
#define KI_ACCEL_STRAIGHT           (0.03f)
#define KD_ACCEL_STRAIGHT           (0.00f)
#define ACCEL_INTEGRAL_MAX_STRAIGHT (4.0f)
#define ACCEL_PWM_CORR_MAX_STRAIGHT (3)

/*
 * 转弯速度环（左右独立跟踪各自 Cmd）
 * 非对称转弯后整体已降速，FF/增益放小，避免 Out 再把弯拧飞
 */
#define ACCEL_TURN_FF_BOOST         (5)
#define KP_ACCEL_TURN               (0.40f)
#define KI_ACCEL_TURN               (0.10f)
#define KD_ACCEL_TURN               (0.00f)
#define ACCEL_INTEGRAL_MAX_TURN     (10.0f)
#define ACCEL_PWM_CORR_MAX_TURN     (10)

/* 各直线段目标航向偏移 */
#define HEADING_OFFSET_AB           (0.0f)
#define HEADING_OFFSET_CD           (180.0f)
#define HEADING_OFFSET_AC           (-50.0f)   /* 需实测微调 */
#define HEADING_OFFSET_BD           (130.0f)   /* 需实测微调 */

/* =========================================================
 * 弧线循迹（全部调参集中在此）
 *
 * 思路：
 *   1) ARC_CURVE_BIAS：弯道前馈，线在正中时也持续转弯（贴弧关键）
 *   2) 红外 PD：把车压回 CH4/CH5
 *   3) L = BASE + steer，R = BASE - steer（同向差速）
 * prepare(+1)=左圆弧（右轮更快）；prepare(-1)=右圆弧
 * ========================================================= */

/* --- 1) 速度 --- */
#define BASE_SPEED_ARC              (25)  /* 共模略降，把余量留给差速 */
/*
 * 线在正中时的弯道前馈差速（转不过就加大）：
 *   左弧 prepare(+1)：L=BASE-BIAS, R=BASE+BIAS
 *   右弧 prepare(-1)：相反
 */
#define ARC_CURVE_BIAS              (20)
/*
 * 弧向最小差速：左弧 steer 上限为 -MIN（绝不因 CH678 变成右转）；
 * 右弧 steer 下限为 +MIN。建议 4~8
 */
#define ARC_STEER_KEEP_MIN          (8)
/*
 * 弧线线灭分两种（用累计偏航区分）：
 *   |A| ≥ ARC_END_COAST_DEG → 弧线自然结束：沿用上一拍 Cmd（Arc coast）
 *   |A| <  该值           → 异常走出线：原地差速找回（Arc recover）
 * 出弯还必须：已 coast 满 ARC_COAST_MIN_MS，且 |A|≥ARC_TARGET_DEG
 * （防止积分提前到 180° 时线一灭就立刻切 Straight）
 */
#define ARC_END_COAST_DEG           (140.0f)
#define ARC_COAST_MIN_MS            (500U)
#define ARC_LOST_SPEED_L            (-20) /* 异常丢线：左弧 L 倒 R 进 */
#define ARC_LOST_SPEED_R            (20)
/* 丢线搜线用全局 SEARCH_SPEED_LOW / SEARCH_SPEED_HIGH */

/* --- 2) 红外外环 --- */
/* CH4/CH5=0：压在中心时 error≈0 */
#define LINE_WEIGHT_ARC_CH1         (-4)
#define LINE_WEIGHT_ARC_CH2         (-3)
#define LINE_WEIGHT_ARC_CH3         (-2)
#define LINE_WEIGHT_ARC_CH4         (-1)
#define LINE_WEIGHT_ARC_CH5         (0)
#define LINE_WEIGHT_ARC_CH6         (1)
#define LINE_WEIGHT_ARC_CH7         (2)
#define LINE_WEIGHT_ARC_CH8         (3)

#define KP_LINE_ARC                 (18.0f) /* 偏线时再加大拧弯 */
#define KD_LINE_ARC                 (0.2f)
#define LINE_ERROR_DEADZONE_ARC     (0.25f)
#define LINE_ERROR_FILTER_ALPHA_ARC (0.40f)
#define KD_GYRO_ARC                 (0.00f)
#define KD_GYRO_ARC_SIGN            (1)
#define ARC_DIFF_MAX                (100)  /* |steer| 总限幅；须 ≥ CURVE_BIAS+ENTER */

#define ARC_ENTER_ASSIST_MS         (400U)
#define ARC_ENTER_TURN_BIAS         (10)  /* 入弧再加拧，帮助咬住弯 */
/*
 * 进入弧线阶段后，先用普通红外巡线直行该时长；
 * 到时后才加入 ARC_CURVE_BIAS 和 ARC_ENTER_TURN_BIAS。
 */
#define ARC_LINE_ONLY_MS             (1000U)
/* --- 3) Straight ↔ Arc --- */
#define LINE_DETECT_HOLD_CNT        (3U)
#define LINE_LOST_HOLD_CNT          (8U)
#define ARC_TARGET_DEG              (180.0f) /* Arc→Straight / 丢线结束：累计偏航 */
#define ARC_MIN_TIME_MS             (1200U)
#define ARC_EXIT_ERR_BAND           (1)

/* =========================================================
 * 声光提示参数
 * ========================================================= */
#define POINT_SIGNAL_ON_MS          (20U)
#define POINT_SIGNAL_OFF_MS         (20U)
#define START_SIGNAL_TIMES          (2U)
#define POINT_SIGNAL_TIMES          (1U)
#define FINISH_SIGNAL_TIMES         (3U)

/* =========================================================
 * 任务目标
 * ========================================================= */
#define TARGET_LAPS                 (4U)

/* =========================================================
 * MPU6050 寄存器地址（勿随意修改）
 * ========================================================= */
#define MPU6050_ADDR                (0x68U)
#define MPU6050_WHO_AM_I_REG        (0x75U)
#define MPU6050_PWR_MGMT_1_REG      (0x6BU)
#define MPU6050_SMPLRT_DIV_REG      (0x19U)
#define MPU6050_CONFIG_REG          (0x1AU)
#define MPU6050_GYRO_CONFIG_REG     (0x1BU)
#define MPU6050_ACCEL_CONFIG_REG    (0x1CU)
#define MPU6050_GYRO_ZOUT_H_REG     (0x47U)
#define MPU6050_INT_PIN_CFG_REG     (0x37U)
#define MPU6050_INT_ENABLE_REG      (0x38U)
#define MPU6050_INT_STATUS_REG      (0x3AU)

#define MPU6050_DATA_RATE_HZ        (125.0f)
#define MPU_INT_SAMPLE_HZ           (125.0f)
/*
 * 采样率核对（与 mpu6050_init 寄存器一致）：
 *   CONFIG=0x03 → DLPF 开，陀螺输出 1kHz
 *   SMPLRT_DIV=0x07 → SampleRate = 1000/(1+7) = 125Hz
 *   → INT_DT = 1/125 s，勿改成主循环周期
 */

/* =========================================================
 * MPU6050 量程与校准 / 积分净化
 * ========================================================= */
#define MPU_GYRO_SENS_500DPS        (65.5f)
#define MPU_CALIB_SAMPLES           (400U)   /* 静止标定点数（约 400×8ms≈3.2s） */
#define MPU_CALIB_TRIM_EACH         (20U)    /* 去掉两端极值各 N 点 */

/* 积分前低通：filt = α·new + (1-α)·old；α 越大越跟手、越小越稳 */
#define GYRO_Z_LPF_ALPHA            (0.30f)
/* 死区（dps）：过小易噪声积分，过大转弯欠积分；建议 0.8~1.5 */
#define GYRO_Z_DEADZONE_DPS         (1.2f)
/* 单周期 |Δψ| 上限（deg）：125Hz 下 4°/拍 ≈ 500°/s，压掉抖动尖峰 */
#define GYRO_Z_DYAW_MAX_DEG         (4.0f)

#endif /* APP_CONFIG_H */

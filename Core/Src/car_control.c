/**
 * @file car_control.c
 * @brief 智能车控制系统实现
 * @details 包含车辆运动控制、避障控制、任务规划等功能
 */

#include "car_control.h"
#include "motor_drive.h"
#include "speed_encoder.h"
#include "UltrasonicWave.h"
#include "oled_i2c.h"
#include "ctrl_menu.h"

/* ==================== 宏定义 ==================== */
#define CHA         +11

/* ==================== 全局变量定义 ==================== */
int mission = 0;
int a = 0;
int b = 0;
__IO CarCtrl_State_TypeDef g_car_ctrl_state = CarCtrl_STOP;

avoid_ctrl_t g_avoid_ctrl = {0, 0, 0, -1, 0, {0, 0}};

car_config_t g_CarConfig = {
    .speed_KP = 8,
    .speed_KI = 0.1,
    .speed_KD = 0
};

car_ctrl_t g_CarCtrl;
car_plan_t* g_CarPlan_Ptr;

/* ==================== 运行计划数组定义 ==================== */

/**
 * @brief 基础运行计划
 * @note 用于基本的方形路径行驶
 */
car_plan_t g_CarPlan_Base[] = {
    { CHA - 8,   { 2000, 2000 }, 0, 120 },     // 直行 1.2 秒
    { CHA + 75,  { 480,  1350 }, 0, 148 },     // 右转
    { CHA,       { 2000, 2000 }, 0, 90  },     // 直行 0.9 秒
    { CHA + 75,  { 480,  1350 }, 0, 80  },     // 右转
    { CHA,       { 0,    0    }, 0, 0   }      // 停止
};

/**
 * @brief 高级运行计划
 * @note 包含避障和复杂路径的运行计划
 */
car_plan_t g_CarPlan_Supper[] = {
    // 第一阶段：检测障碍物
    { CHA,       { 400,  400  }, 40000, 1000 },
    { CHA,       { 0,    0    }, 0,     50   },
    
    // 第二阶段：第一次转弯
    { CHA,       { 600,  -500 }, 0,     180  },
    { CHA,       { 0,    0    }, 0,     50   },
    
    // 第三阶段：慢速前进
    { CHA,       { 200,  200  }, 30000, 200  },
    { CHA,       { 0,    0    }, 0,     50   },
    
    // 第四阶段：左转
    { CHA,       { -500, 600  }, 0,     140  },
    { CHA,       { 0,    0    }, 0,     50   },
    
    // 第五阶段：直行
    { CHA,       { 500,  500  }, 30000, 150  },
    { CHA,       { 0,    0    }, 0,     100  },
    
    // 第六阶段：再次左转
    { CHA,       { -500, 600  }, 0,     140  },
    { CHA,       { 0,    0    }, 0,     50   },
    
    // 第七阶段：慢速前进
    { CHA,       { 200,  200  }, 30000, 140  },
    { CHA,       { 0,    0    }, 0,     50   },
    
    // 第八阶段：转弯复位
    { CHA,       { 600,  -500 }, 0,     180  },
    { CHA,       { 0,    0    }, 0,     50   },
    
    // 最后阶段：前进并停止
    { CHA,       { 200,  200  }, 30000, 200  },
    { CHA,       { 0,    0    }, 0,     0    }
};

/**
 * @brief 避障演示计划
 * @note 用于测试避障功能
 */
car_plan_t g_CarPlan_AvoidDemo[] = {
    { 0,    {300, 300}, 25000, 500},
    { 0,    {400, 400}, 25000, 500},
    { 20,   {300, 300}, 25000, 300},
    { -20,  {300, 300}, 25000, 300},
    { 0,    {0, 0},     0,     0},
};

/* ==================== 避障控制功能实现 ==================== */

/**
 * @brief 车辆避障控制函数
 * @details 实现基于超声波传感器的避障功能
 */
void CarCtrl_AvoidObstacle(void)
{
    // 避障状态定义
    #define AVOID_IDLE          0
    #define AVOID_DETECTED      1
    #define AVOID_BACKUP        2
    #define AVOID_TURN_OUT      3
    #define AVOID_FORWARD_OUT   4
    #define AVOID_TURN_BACK     5
    #define AVOID_FORWARD_BACK  6
    #define AVOID_RECOVER       7
    
    // 避障参数定义
    const uint32_t OBSTACLE_DIST = 25000;       // 250mm 障碍物检测距离
    const int16_t AVOID_ANGLE = 45;             // 避障转向角度
    const uint16_t TURN_OUT_TIME = 80;          // 转出时间
    const uint16_t FORWARD_OUT_TIME = 60;       // 前进绕行时间
    const uint16_t TURN_BACK_TIME = 80;         // 转回时间
    const uint16_t FORWARD_BACK_TIME = 60;      // 前进回归时间
    
    switch(g_avoid_ctrl.state) {
        case AVOID_IDLE:
            // 空闲状态：检测障碍物
            if (g_ultrawave_data[0].distance < OBSTACLE_DIST && 
                g_ultrawave_data[0].distance > 100) {
                // 检测到障碍物，保存当前状态
                g_avoid_ctrl.original_angle = g_CarCtrl.car_angle;
                g_avoid_ctrl.original_speed[0] = g_CarCtrl.car_speed_set[0];
                g_avoid_ctrl.original_speed[1] = g_CarCtrl.car_speed_set[1];
                
                // 停车
                g_CarCtrl.car_speed_set[0] = 0;
                g_CarCtrl.car_speed_set[1] = 0;
                g_avoid_ctrl.state = AVOID_DETECTED;
                g_avoid_ctrl.timer = 0;
                
                // 切换转向方向
                g_avoid_ctrl.turn_direction *= -1;
            }
            break;
            
        case AVOID_DETECTED:
            // 检测到障碍物：短暂停止
            if (++g_avoid_ctrl.timer > 10) {  // 100ms
                g_avoid_ctrl.state = AVOID_BACKUP;
                g_avoid_ctrl.timer = 0;
            }
            break;
            
        case AVOID_BACKUP:
            // 后退阶段
            g_CarCtrl.car_angle = g_avoid_ctrl.original_angle;
            Steer_Moto_Ctrl(STEER_MOTO_POS, g_CarCtrl.car_angle);
            g_CarCtrl.car_speed_set[0] = -200;
            g_CarCtrl.car_speed_set[1] = -200;
            
            if (++g_avoid_ctrl.timer > 20) {  // 200ms
                g_avoid_ctrl.state = AVOID_TURN_OUT;
                g_avoid_ctrl.timer = 0;
            }
            break;
            
        case AVOID_TURN_OUT:
            // 转向避开阶段
            g_CarCtrl.car_angle = g_avoid_ctrl.original_angle + 
                                  (g_avoid_ctrl.turn_direction * AVOID_ANGLE);
            // 限制转向角度
            if (g_CarCtrl.car_angle > 45) g_CarCtrl.car_angle = 45;
            if (g_CarCtrl.car_angle < -45) g_CarCtrl.car_angle = -45;
            
            Steer_Moto_Ctrl(STEER_MOTO_POS, g_CarCtrl.car_angle);
            g_CarCtrl.car_speed_set[0] = 300;
            g_CarCtrl.car_speed_set[1] = 300;
            
            if (++g_avoid_ctrl.timer > TURN_OUT_TIME) {
                g_avoid_ctrl.state = AVOID_FORWARD_OUT;
                g_avoid_ctrl.timer = 0;
            }
            break;
            
        case AVOID_FORWARD_OUT:
            // 前进绕行阶段
            g_CarCtrl.car_speed_set[0] = 350;
            g_CarCtrl.car_speed_set[1] = 350;
            
            if (++g_avoid_ctrl.timer > FORWARD_OUT_TIME) {
                g_avoid_ctrl.state = AVOID_TURN_BACK;
                g_avoid_ctrl.timer = 0;
            }
            break;
            
        case AVOID_TURN_BACK:
            // 转回原方向阶段
            g_CarCtrl.car_angle = g_avoid_ctrl.original_angle - 
                                  (g_avoid_ctrl.turn_direction * AVOID_ANGLE);
            // 限制转向角度
            if (g_CarCtrl.car_angle > 45) g_CarCtrl.car_angle = 45;
            if (g_CarCtrl.car_angle < -45) g_CarCtrl.car_angle = -45;
            
            Steer_Moto_Ctrl(STEER_MOTO_POS, g_CarCtrl.car_angle);
            g_CarCtrl.car_speed_set[0] = 300;
            g_CarCtrl.car_speed_set[1] = 300;
            
            if (++g_avoid_ctrl.timer > TURN_BACK_TIME) {
                g_avoid_ctrl.state = AVOID_FORWARD_BACK;
                g_avoid_ctrl.timer = 0;
            }
            break;
            
        case AVOID_FORWARD_BACK:
            // 前进回归阶段
            g_CarCtrl.car_speed_set[0] = 350;
            g_CarCtrl.car_speed_set[1] = 350;
            
            if (++g_avoid_ctrl.timer > FORWARD_BACK_TIME) {
                g_avoid_ctrl.state = AVOID_RECOVER;
                g_avoid_ctrl.timer = 0;
            }
            break;
            
        case AVOID_RECOVER:
            // 恢复原始状态阶段
            g_CarCtrl.car_angle = g_avoid_ctrl.original_angle;
            Steer_Moto_Ctrl(STEER_MOTO_POS, g_CarCtrl.car_angle);
            g_CarCtrl.car_speed_set[0] = g_avoid_ctrl.original_speed[0];
            g_CarCtrl.car_speed_set[1] = g_avoid_ctrl.original_speed[1];
            
            if (++g_avoid_ctrl.timer > 20) {  // 200ms
                g_avoid_ctrl.state = AVOID_IDLE;
            }
            break;
    }
}

/* ==================== 菜单控制功能实现 ==================== */

/**
 * @brief 启动车辆控制
 */
void CarCtrl_Start(void)
{
    g_car_ctrl_state = CarCtrl_IDLE;
}

/**
 * @brief 停止车辆控制
 */
void CarCtrl_Stop(void)
{
    g_car_ctrl_state = CarCtrl_STOP;
    // Ctrl_Menu_Show();
}

/**
 * @brief 增加速度
 */
void CarCtrl_SpeedUp(void)
{
    for (int32_t i = 0; i < DRIVE_MOTO_NUM; i++) {
        g_CarCtrl.car_speed_set[i] += 5;
    }
}

/**
 * @brief 减少速度
 */
void CarCtrl_SpeedDown(void)
{
    for (int32_t i = 0; i < DRIVE_MOTO_NUM; i++) {
        g_CarCtrl.car_speed_set[i] -= 5;
    }
}

/**
 * @brief 速度归零
 */
void CarCtrl_SpeedStop(void)
{
    for (int32_t i = 0; i < DRIVE_MOTO_NUM; i++) {
        g_CarCtrl.car_speed_set[i] = 0;
    }
}

/**
 * @brief 前进
 */
void CarCtrl_Forward(void)
{
    for (int32_t i = 0; i < DRIVE_MOTO_NUM; i++) {
        g_CarCtrl.car_speed_set[i] = abs(g_CarCtrl.car_speed_set[i]);
    }
}

/**
 * @brief 后退
 */
void CarCtrl_Backward(void)
{
    for (int32_t i = 0; i < DRIVE_MOTO_NUM; i++) {
        g_CarCtrl.car_speed_set[i] = -1 * abs(g_CarCtrl.car_speed_set[i]);
    }
}

/**
 * @brief 直行
 */
void CarCtrl_Straight(void)
{
    g_CarCtrl.car_angle = CHA;
    Steer_Moto_Ctrl(STEER_MOTO_POS, g_CarCtrl.car_angle);
}

/**
 * @brief 右转
 */
void CarCtrl_Right(void)
{
    g_CarCtrl.car_angle -= 1;
    // if (g_CarCtrl.car_angle < -45) g_CarCtrl.car_angle = -45;
    if (g_CarCtrl.car_angle < -90) g_CarCtrl.car_angle = -90;
    Steer_Moto_Ctrl(STEER_MOTO_POS, g_CarCtrl.car_angle);
}

/**
 * @brief 左转
 */
void CarCtrl_Left(void)
{
    g_CarCtrl.car_angle += 1;
    // if (g_CarCtrl.car_angle > 45) g_CarCtrl.car_angle = 45;
    if (g_CarCtrl.car_angle > 90) g_CarCtrl.car_angle = 90;
    Steer_Moto_Ctrl(STEER_MOTO_POS, g_CarCtrl.car_angle);
}

/* ==================== 核心控制功能实现 ==================== */

/**
 * @brief 初始化车辆控制系统
 */
void CarCtrl_Init(void)
{
    car_config_t *p_car_cfg = &g_CarConfig;
    memset(&g_CarCtrl, 0, sizeof(g_CarCtrl));
    g_CarPlan_Ptr = g_CarPlan_Base;
    mission = 0;
}

/**
 * @brief 速度PID控制
 * @details 实现电机速度的闭环控制
 */
void CarCtrl_Speed_PID(void)
{
    static int32_t last_speed[DRIVE_MOTO_NUM] = {0, 0};
    static int32_t speed_intergrade[DRIVE_MOTO_NUM] = {0, 0};
    int32_t speed_error[DRIVE_MOTO_NUM];
    int32_t speed_diff[DRIVE_MOTO_NUM];

    for (int32_t i = 0; i < DRIVE_MOTO_NUM; i++) {
        // 计算速度误差
        speed_error[i] = g_CarCtrl.car_speed_set[i] - g_speed_encoder[i].speed;
        
        // 积分项累加
        speed_intergrade[i] = speed_intergrade[i] + speed_error[i];
        
        // 计算微分项
        speed_diff[i] = last_speed[i] - g_speed_encoder[i].speed;
        last_speed[i] = g_speed_encoder[i].speed;
        
        // PID计算
        g_CarCtrl.moto_drive[i] = speed_error[i] * g_CarConfig.speed_KP +
                                  speed_intergrade[i] * g_CarConfig.speed_KI +
                                  speed_diff[i] * g_CarConfig.speed_KD;
        
        // Drive_Moto_Ctrl(i, g_CarCtrl.moto_drive[i]);
    }
    
    // 驱动电机（第二个电机反向）
    Drive_Moto_Ctrl(0, g_CarCtrl.moto_drive[0]);
    Drive_Moto_Ctrl(1, -g_CarCtrl.moto_drive[1]);
}

/**
 * @brief 执行运行计划
 * @details 根据预设的运行计划控制车辆
 */
void CarCtrl_PlanSet(void)
{
    car_plan_t* car_plan_ptr;
    
    car_plan_ptr = g_CarPlan_Ptr + a;
    
    // 检查是否到达计划终点
    if (car_plan_ptr->run_time_set == 0) {
        g_car_ctrl_state = CarCtrl_STOP;
        Steer_Moto_Ctrl(STEER_MOTO_POS, car_plan_ptr->car_angle_set);
        for (int32_t i = 0; i < DRIVE_MOTO_NUM; i++) {
            Drive_Moto_Ctrl(i, 0);
        }
        memset(&g_CarCtrl, 0, sizeof(g_CarCtrl));
        return;
    }
    
    // 加载新计划
    if (g_CarCtrl.run_time == 0) {
        g_CarCtrl.run_time++;
        for (int32_t i = 0; i < DRIVE_MOTO_NUM; i++) {
            g_CarCtrl.car_speed_set[i] = car_plan_ptr->car_speed_set[i];
        }
        Steer_Moto_Ctrl(STEER_MOTO_POS, car_plan_ptr->car_angle_set);
    }
    // 执行当前计划
    else {
        g_CarCtrl.run_time++;
        
        // 检查前方障碍物距离
        if (g_ultrawave_data[0].distance < car_plan_ptr->front_distance_set && 
            g_ultrawave_data[0].distance > 0) {
            // if(g_ultrawave_data[0].distance != -1){
                g_CarCtrl.run_time = 0;
                a = 1;
            // }
        }
        
        // 检查运行时间是否到达
        if (g_CarCtrl.run_time == car_plan_ptr->run_time_set) {
            // if(g_ultrawave_data[0].distance != -1){
                g_CarCtrl.run_time = 0;
                if (a == 1) {
                    b++;
                }
                if ((a == 12) && (b != 0)) {
                    b--;
                } else {
                    a++;
                }
            // }
        }
    }
}

/**
 * @brief 显示车辆状态信息
 * @details 在OLED显示屏上显示运行状态
 */
void CarCtrl_Show(void)
{
    static uint8_t index = 0;
    static int32_t speed[DRIVE_MOTO_NUM] = {0, 0};
    static int32_t pwm[DRIVE_MOTO_NUM] = {0, 0};
    uint8_t buf[17];
    
    // 累加速度和PWM值用于计算平均值
    for (int i = 0; i < DRIVE_MOTO_NUM; i++) {
        speed[i] += g_speed_encoder[i].speed;
        pwm[i] += g_CarCtrl.moto_drive[i];
    }
    
    // 每10次更新一次显示
    if (index < 9) {
        index++;
    } else {
        index = 0;
        speed[0] = speed[0] / 10;
        speed[1] = speed[1] / 10;
        pwm[0] = pwm[0] / 10;
        pwm[1] = pwm[1] / 10;
        
        // 显示任务编号
        sprintf(buf, "%d", mission);
        OLED_ShowAscii(0, 0, buf, 16, 0);
        
        // 显示超声波距离
        sprintf(buf, "%d", g_ultrawave_data[0].distance);
        OLED_ShowAscii(1, 0, buf, 16, 0);
        
        // 重置累加值
        speed[0] = 0;
        speed[1] = 0;
        pwm[0] = 0;
        pwm[1] = 0;
    }
}

/**
 * @brief 车辆控制主处理函数
 * @details 在定时器中断中周期调用
 */
void CarCtrl_Process(void)
{
    if (g_car_ctrl_state == CarCtrl_STOP) {
        return;
    }
    
    if (g_car_ctrl_state == CarCtrl_START) {
        g_car_ctrl_state = CarCtrl_IDLE;
        Speed_Calculate();
        CarCtrl_Speed_PID();
        CarCtrl_Show();
        CarCtrl_PlanSet();
    }
}

/* ==================== 任务定义 ==================== */

/**
 * @brief 任务1运行计划
 * @note 基础方形路径
 */
car_plan_t g_CarPlan_Mission1[] = {
    { CHA,       { 2000, 2000 }, 0, 120 },
    { CHA + 75,  { 520,  1350 }, 0, 160 },
    { CHA,       { 2000, 2000 }, 0, 70  },
    { CHA + 75,  { 550,  1350 }, 0, 90  },
    { CHA - 25,  { 2000, 2000 }, 0, 0   },
    { CHA,       { 0,    0    }, 0, 0   }
};

/**
 * @brief 任务2运行计划
 * @note 带避障的路径
 */
car_plan_t g_CarPlan_Mission2[] = {
    { CHA,       { 1000, 1000 }, 40000, 100 },
    { CHA - 75,  { 500,  500  }, 0,     100 },
    { CHA + 75,  { 500,  500  }, 0,     200 },
    { CHA - 50,  { 250,  250  }, 0,     100 },
    { CHA - 50,  { 500,  500  }, 0,     70  },
    { CHA,       { 1000, 1000 }, 0,     100 },
    { CHA,       { 0,    0    }, 0,     0   }
};

/**
 * @brief 任务3运行计划
 * @note 复杂路径与避障
 */
car_plan_t g_CarPlan_Mission3[] = {
    // 第一阶段：检测障碍物
    { CHA,       { 400,  400  }, 40000, 1000 },
    { CHA,       { 0,    0    }, 0,     50   },
    
    // 第二阶段：第一次转弯
    { CHA,       { 600,  -500 }, 0,     180  },
    { CHA,       { 0,    0    }, 0,     50   },
    
    // 第三阶段：慢速前进检测
    { CHA,       { 200,  200  }, 30000, 200  },
    { CHA,       { 0,    0    }, 0,     50   },
    
    // 第四阶段：左转避障
    { CHA,       { -500, 600  }, 0,     140  },
    { CHA,       { 0,    0    }, 0,     50   },
    
    // 第五阶段：快速直行
    { CHA,       { 500,  500  }, 30000, 150  },
    { CHA,       { 0,    0    }, 0,     100  },
    
    // 第六阶段：再次左转
    { CHA,       { -500, 600  }, 0,     140  },
    { CHA,       { 0,    0    }, 0,     50   },
    
    // 第七阶段：慢速前进
    { CHA,       { 200,  200  }, 30000, 140  },
    { CHA,       { 0,    0    }, 0,     50   },
    
    // 第八阶段：右转复位
    { CHA,       { 600,  -500 }, 0,     180  },
    { CHA,       { 0,    0    }, 0,     50   },
    
    // 最后阶段：前进并停止
    { CHA,       { 200,  200  }, 30000, 200  },
    { CHA,       { 0,    0    }, 0,     0    }
};

/* ==================== 任务启动函数 ==================== */

/**
 * @brief 启动任务1
 * @details 基础运行任务
 */
void CarCtrl_Mission1(void)
{
    CarCtrl_Init();
    g_CarPlan_Ptr = g_CarPlan_Mission1;
    mission = 1;
    g_car_ctrl_state = CarCtrl_IDLE;
    
    // OLED_clear();
    // OLED_ShowAscii(0, 0, "Mission 1", 16, 0);
    // HAL_Delay(1000);
}

/**
 * @brief 启动任务2
 * @details 避障运行任务
 */
void CarCtrl_Mission2(void)
{
    CarCtrl_Init();
    g_CarPlan_Ptr = g_CarPlan_Mission2;
    mission = 2;
    // g_CarCtrl.run_step = 0;
    // g_CarCtrl.run_time = 0;
    g_car_ctrl_state = CarCtrl_IDLE;
    
    // OLED_clear();
    // OLED_ShowAscii(0, 0, "Mission 2", 16, 0);
    // HAL_Delay(1000);
}

/**
 * @brief 启动任务3
 * @details 复杂路径避障任务
 */
void CarCtrl_Mission3(void)
{
    CarCtrl_Init();
    g_CarPlan_Ptr = g_CarPlan_Mission3;
    mission = 3;
    // g_CarCtrl.run_step = 0;
    // g_CarCtrl.run_time = 0;
    g_car_ctrl_state = CarCtrl_IDLE;
    
    // OLED_clear();
    // OLED_ShowAscii(0, 0, "Mission 3", 16, 0);
    // OLED_ShowAscii(1, 0, "Avoid ON", 16, 0);
    // HAL_Delay(1000);
}
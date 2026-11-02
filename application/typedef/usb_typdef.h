#ifndef USB_TYPEDEF_H
#define USB_TYPEDEF_H

#include "attribute_typedef.h"
#include "remote_control.h"
#include "struct_typedef.h"

#define DEBUG_PACKAGE_NUM 10

#define DATA_DOMAIN_OFFSET 0x08

// clang-format off
#define SEND_SOF    ((uint8_t)0x5A)
#define RECEIVE_SOF ((uint8_t)0x5A)

#define DEBUG_DATA_SEND_ID        ((uint8_t)0x01)
#define IMU_DATA_SEND_ID          ((uint8_t)0x02)
#define ROBOT_STATE_INFO_DATA_SEND_ID   ((uint8_t)0x03)

#define GAME_STATUS_SEND_ID       ((uint8_t)0x04) 
#define GAME_RESULT_SEND_ID       ((uint8_t)0x05)
#define ALL_ROBOT_HP_SEND_ID      ((uint8_t)0x06)
#define EVENT_DATA_SEND_ID        ((uint8_t)0x07)
#define REFEREE_WARNING_SEND_ID   ((uint8_t)0x08)
#define DART_INFO_SEND_ID         ((uint8_t)0x09)
#define ROBOT_STATUS_SEND_ID      ((uint8_t)0x0A)
#define POWER_HEAT_DATA_SEND_ID   ((uint8_t)0x0B)
#define ROBOT_POS_SEND_ID         ((uint8_t)0x0C)
#define BUFF_SEND_ID              ((uint8_t)0x0D)
#define HURT_DATA_SEND_ID         ((uint8_t)0x0E)
#define SHOOT_DATA_SEND_ID        ((uint8_t)0x0F)
#define PROJECTILE_ALLOWANCE_SEND_ID   ((uint8_t)0x10)
#define RFID_STATUS_SEND_ID       ((uint8_t)0x11)
#define DART_CLIENT_CMD_SEND_ID      ((uint8_t)0x12)
#define GROUND_ROBOT_POSITION_SEND_ID ((uint8_t)0x13)
#define RADAR_MARK_DATA_SEND_ID         ((uint8_t)0x14)
#define SENTRY_INFO_SEND_ID         ((uint8_t)0x15)
#define RADAR_INFO_SEND_ID         ((uint8_t)0x16)
#define ROBOT_INTERACTION_DATA_SEND_ID ((uint8_t)0x17)
#define CUSTOM_CONTROLLER_SEND_ID  ((uint8_t)0x18)
#define MAP_COMMAND_SEND_ID        ((uint8_t)0x19)
#define ROBOT_CUSTOM_DATA_SEND_ID     ((uint8_t)0x1A)
#define ROBOT_CUSTOM_DATA_3_SEND_ID   ((uint8_t)0x1B)
#define PID_DEBUG_DATA_SEND_ID    ((uint8_t)0x1C)
#define JOINT_STATE_SEND_ID       ((uint8_t)0x1D)
#define ROBOT_MOTION_DATA_SEND_ID ((uint8_t)0x1E)
#define USB_TEST_SEND_ID          ((uint8_t)0x1F)

#define ROBOT_CMD_DATA_RECEIVE_ID  ((uint8_t)0x01)
#define PID_DEBUG_DATA_RECEIVE_ID  ((uint8_t)0x02)
#define VIRTUAL_RC_DATA_RECEIVE_ID ((uint8_t)0x03)
#define USB_ECHO_RECEIVE_ID        ((uint8_t)0x04)
#define REFEREE_INTERACTION_DATA_RECEIVE_ID ((uint8_t)0x05)
// clang-format on

typedef struct
{
    uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
    uint8_t len;  // 数据段长度
    uint8_t id;   // 数据段id
    uint8_t crc;  // 数据帧头的 CRC8 校验
} __packed__ FrameHeader_t;
/*-------------------- Send --------------------*/

// 串口调试数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x01
    uint32_t time_stamp;
    struct
    {
        uint8_t name[10];
        uint8_t type;
        float data;
    } __packed__ packages[DEBUG_PACKAGE_NUM];
    uint16_t checksum;
} __packed__ SendDataDebug_s;

// IMU 数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x02
    uint32_t time_stamp;
    struct
    {
        float yaw;    // rad
        float pitch;  // rad
        float roll;   // rad

        float yaw_vel;    // rad/s
        float pitch_vel;  // rad/s
        float roll_vel;   // rad/s

        // float x_accel;  // m/s^2
        // float y_accel;  // m/s^2
        // float z_accel;  // m/s^2
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataImu_s;

// 机器人信息数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x03
    uint32_t time_stamp;
    struct
    {
        /// @brief 机器人部位类型 2 bytes
        struct
        {
            uint16_t chassis : 3;
            uint16_t gimbal : 3;
            uint16_t shoot : 3;
            uint16_t arm : 3;
            uint16_t custom_controller : 3;
            uint16_t reserve : 1;
        } __packed__ type;
        /// @brief 机器人部位状态 1 byte
        /// @note 0: 正常，1: 错误
        struct
        {
            uint8_t chassis : 1;
            uint8_t gimbal : 1;
            uint8_t shoot : 1;
            uint8_t arm : 1;
            uint8_t custom_controller : 1;
            uint8_t reserve : 3;
        } __packed__ state;
        // /// @brief 机器人裁判系统信息 7 bytes
        // struct
        // {
        //     uint8_t id;
        //     uint8_t color;  // 0-red 1-blue 2-unknown
        //     bool attacked;
        //     uint16_t hp;
        //     uint16_t heat;
        // } __packed__ referee;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRobotStateInfo_s;


// 比赛信息数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x04
    uint32_t time_stamp;
    struct
    {
        uint8_t game_type : 4;
        uint8_t game_progress : 4;
        uint16_t stage_remain_time;
        uint64_t SyncTimeStamp;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataGameStatus_s;

typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x05
    uint32_t time_stamp;
    struct
    {
        uint8_t winner;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataGameResult_s;

// 全场机器人hp信息数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x06
    uint32_t time_stamp;
    struct
    {
        // uint16_t red_1_robot_hp;
        // uint16_t red_2_robot_hp;
        // uint16_t red_3_robot_hp;
        // uint16_t red_4_robot_hp;
        // uint16_t red_7_robot_hp;
        // uint16_t red_outpost_hp;
        // uint16_t red_base_hp;
        // uint16_t blue_1_robot_hp;
        // uint16_t blue_2_robot_hp;
        // uint16_t blue_3_robot_hp;
        // uint16_t blue_4_robot_hp;
        // uint16_t blue_7_robot_hp;
        // uint16_t blue_outpost_hp;
        // uint16_t blue_base_hp;
        uint16_t ally_1_robot_HP;  
        uint16_t ally_2_robot_HP;  
        uint16_t ally_3_robot_HP; 
        uint16_t ally_4_robot_HP;  
        uint16_t reserved;  
        uint16_t ally_7_robot_HP;  
        uint16_t ally_outpost_HP;  
        uint16_t ally_base_HP;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataAllRobotHp_s;


// 事件数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x07
    uint32_t time_stamp;

    struct
    {
        uint32_t event_data;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataEvent_s;

// 裁判警告数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x08
    uint32_t time_stamp;

    struct
    {
        uint8_t level;
        uint8_t offending_robot_id;
        uint8_t count;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRefereeWarning_s;

typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x09
    uint32_t time_stamp;

    struct
    {
        uint8_t dart_remaining_time;
        uint16_t dart_info;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataDartInfo_s;

// 机器人状态数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0A
    uint32_t time_stamp;

    struct
    {
        uint8_t robot_id; 
        uint8_t robot_level; 
        uint16_t current_HP;  
        uint16_t maximum_HP; 
        uint16_t shooter_barrel_cooling_value; 
        uint16_t shooter_barrel_heat_limit; 
        uint16_t chassis_power_limit;  
        float x;      //本机器人位置 x 坐标，单位：m
        float y;      //本机器人位置 y 坐标，单位：m
        float angle;  //本机器人测速模块的朝向，单位：度。正北为 0 度
        uint8_t armor_id : 4;
        uint8_t HP_deduction_reason : 4;
        uint16_t projectile_allowance_17mm;     // 17mm弹丸允许发弹量
        uint16_t projectile_allowance_42mm;     // 42mm弹丸允许发弹量
        uint16_t remaining_gold_coin;           // 剩余金币
        uint16_t projectile_allowance_fortress; // 堡垒增益点提供的储备17mm弹丸
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRobotStatus_s;


// 电源热量数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0B
    uint32_t time_stamp;

    struct
    {
        uint16_t reserved1; 
        uint16_t reserved2; 
        float reserved; 
        uint16_t buffer_energy; 
        uint16_t shooter_17mm_barrel_heat; 
        uint16_t shooter_42mm_barrel_heat;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataPowerHeat_s;

// 地面机器人位置数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x0C
    uint32_t time_stamp;

    struct
    {
        float x;      //本机器人位置 x 坐标，单位：m
        float y;      //本机器人位置 y 坐标，单位：m
        float angle;  //本机器人测速模块的朝向，单位：度。正北为 0 度
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRobotPos_s;

// 机器人增益和底盘能量数据包
typedef struct
{
    FrameHeader_t frame_header; // 数据段id = 0x0D
    uint32_t time_stamp;

    struct
    {
        uint8_t recovery_buff;
        uint16_t cooling_buff;
        uint8_t defence_buff;
        uint8_t vulnerability_buff;
        uint16_t attack_buff;
        uint8_t remaining_energy;
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataBuff_s;

// 机器人受伤数据包
typedef struct
{
    FrameHeader_t frame_header; // 数据段id = 0x0E
    uint32_t time_stamp;

    struct
    {
        uint8_t armor_id : 4;
        uint8_t HP_deduction_reason : 4;
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataHurt_s;

// 机器人射击数据包
typedef struct
{
    FrameHeader_t frame_header; // 数据段id = 0x0F
    uint32_t time_stamp;

    struct
    {
        uint8_t bullet_type;          //弹丸类型
        uint8_t shooter_number;       //发射机构 ID：
        uint8_t launching_frequency;  //弹丸射速（单位：Hz）
        float initial_speed;          //弹丸初速度（单位：m/s）
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataShootData_s;

// 允许发弹量数据包
typedef struct
{
    FrameHeader_t frame_header; // 数据段id = 0x10
    uint32_t time_stamp;

    struct
    {
        uint16_t projectile_allowance_17mm;     // 17mm弹丸允许发弹量
        uint16_t projectile_allowance_42mm;     // 42mm弹丸允许发弹量
        uint16_t remaining_gold_coin;           // 剩余金币
        uint16_t projectile_allowance_fortress; // 堡垒增益点提供的储备17mm弹丸
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataProjectileAllowance_s;

// RFID状态数据包
typedef struct
{
    FrameHeader_t frame_header; // 数据段id = 0x11
    uint32_t time_stamp;

    struct
    {
        uint32_t rfid_status;                    // 32位RFID状态
        uint8_t rfid_status_2;                    // 扩展RFID状态
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataRfidStatus_s;

// 飞镖客户端命令数据包
typedef struct
{
    FrameHeader_t frame_header; // 数据段id = 0x12
    uint32_t time_stamp;

    struct
    {
        uint8_t dart_launch_opening_status;
        uint8_t reserved;
        uint16_t target_change_time;
        uint16_t latest_launch_cmd_time;
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataDartClientCmd_s;

// 地面机器人位置数据包
typedef struct
{
    FrameHeader_t frame_header; // 数据段id = 0x13
    uint32_t time_stamp;

    struct
    {
        float hero_x;
        float hero_y;
        float engineer_x;
        float engineer_y;
        float standard_3_x;
        float standard_3_y;
        float standard_4_x;
        float standard_4_y;
        float reserved1;
        float reserved2;
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataGroundRobotPosition_s;

// 雷达标志数据包
typedef struct
{
    FrameHeader_t frame_header; // 数据段id = 0x14
    uint32_t time_stamp;

    struct
    {
        uint16_t mark_progress;                    // 位域定义参见协议
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataRadarMarkData_s;

//哨兵信息数据包
typedef struct
{
    FrameHeader_t frame_header; // 数据段id = 0x15
    uint32_t time_stamp;

    struct
    {
        uint32_t sentry_info;                       // 32位哨兵信息
        uint16_t sentry_info_2;                      // 扩展哨兵信息
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataSentryInfo_s;

// 雷达信息数据包
typedef struct
{
    FrameHeader_t frame_header; // 数据段id = 0x16
    uint32_t time_stamp;

    struct
    {
        uint8_t radar_info;
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataRadarInfo_s;

// 机器人交互数据包
typedef struct
{
    FrameHeader_t frame_header; // 数据段id = 0x17
    uint32_t time_stamp;

    struct
    {
        uint16_t data_cmd_id;
        uint16_t sender_id;
        uint16_t receiver_id;
        uint8_t user_data[112];                      // 最大112字节
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataRobotInteraction_s;

// 自定义控制器数据包
typedef struct
{
    FrameHeader_t frame_header; // 数据段id = 0x18
    uint32_t time_stamp;

    struct
    {
        uint8_t data[30];
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataCustomController_s;

// 地图命令数据包
typedef struct
{
    FrameHeader_t frame_header; // 数据段id = 0x19
    uint32_t time_stamp;

    struct
    {
        float target_position_x; 
        float target_position_y; 
        uint8_t cmd_keyboard; 
        uint8_t target_robot_id; 
        uint16_t cmd_source; 
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataMapCommand_s;

// 机器人自定义数据包
typedef struct
{
    FrameHeader_t frame_header; // 数据段id = 0x1A
    uint32_t time_stamp;

    struct
    {
        uint8_t data[30]; 
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataRobotCustomData_s;

// 机器人自定义数据包3
typedef struct
{
    FrameHeader_t frame_header; // 数据段id = 0x1B
    uint32_t time_stamp;

    struct
    {
        uint8_t data[30]; 
    } __packed__ data;

    uint16_t crc;
} __packed__ SendDataRobotCustomData3_s;

// PID调参数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x1C
    uint32_t time_stamp;
    struct
    {
        float fdb;
        float ref;
        float pid_out;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataPidDebug_s;

// 云台状态数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x1D
    uint32_t time_stamp;
    struct
    {
        float pitch;
        float yaw;

    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataJointState_s;

// 机器人运动数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x1E
    uint32_t time_stamp;
    struct
    {
        struct
        {
            float vx;
            float vy;
            float wz;
        } __packed__ speed_vector;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRobotMotion_s;

// 链路测试数据包（压测/延迟测量专用，不影响既有包的字节布局）
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x1F
    uint32_t time_stamp;
    struct
    {
        uint32_t seq;         // 单调递增序号，丢包率的分母；t_pc_tx == 0 表示周期包
        uint32_t t_tx_us;     // 发送瞬间的 DWT µs 时间戳
        uint32_t t_rx_us;     // 收到回环请求瞬间（仅回环应答包有效，任务级捕获）
        uint32_t t_pc_tx;     // PC 填写的发送时刻，原样回传；周期包恒为 0
        uint8_t payload[16];  // 压力载荷
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataTest_s;

/*-------------------- Receive --------------------*/
typedef struct RobotCmdData
{
    FrameHeader_t frame_header;  // 数据段id = 0x01
    uint32_t time_stamp;
    struct
    {
        struct
        {
            float vx;
            float vy;
            float wz;
        } __packed__ speed_vector;
        struct
        {
            float roll;
            float pitch;
            float yaw;
            float leg_lenth;
        } __packed__ chassis;
        struct
        {
            float pitch;
            float yaw;
        } __packed__ gimbal;

        struct
        {
            uint8_t fire;
            uint8_t fric_on;
        } __packed__ shoot;

        struct
        {
            bool tracking;
        } __packed__ tracking;
    } __packed__ data;
    uint16_t checksum;
} __packed__ ReceiveDataRobotCmd_s;

// PID调参数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x02
    uint32_t time_stamp;
    struct
    {
        float kp;
        float ki;
        float kd;
        float max_out;
        float max_iout;
    } __packed__ data;
    uint16_t crc;
} __packed__ ReceiveDataPidDebug_s;

// 虚拟遥控器数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x03
    uint32_t time_stamp;
    RC_ctrl_t data;
    uint16_t crc;
} __packed__ ReceiveDataVirtualRc_s;

// 回环请求数据包（延迟测量：PC → MCU → PC，MCU 收到后立即以 0x1F 应答）
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x04
    uint32_t time_stamp;
    struct
    {
        uint32_t seq;         // PC 侧序号
        uint32_t t_pc_tx;     // PC 发送时刻，原样回传
        uint8_t payload[16];  // 校验载荷，应答时原样回传
    } __packed__ data;
    uint16_t crc;
} __packed__ ReceiveDataEcho_s;

// USB 下发裁判系统交互数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x05
    uint32_t time_stamp;
    struct
    {
        uint16_t data_cmd_id;
        uint16_t sender_id;
        uint16_t receiver_id;
        uint8_t user_data[112];
    } __packed__ data;
    uint16_t crc;
} __packed__ ReceiveDataRefereeInteraction_s;
#endif  // USB_TYPEDEF_H

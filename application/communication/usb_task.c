/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb_task.c
  * @brief      通过 USB 虚拟串口与上位机通信
  *
  * @note       本文件的核心是"包表驱动"：
  *             新增一个上行包，只需要在 USB_SEND_PACKET_TABLE 里加一行，
  *             再写一个只负责填字段的 UsbSendXxxData()。
  *             变量声明、函数声明、帧头初始化、分频调度、CRC16 与发送
  *             全部由表自动生成，不需要在多处同步修改。
  *
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done
  *  V1.1.0     2026-4-8        CJH             1. done
  *  V1.2.0     2026-10-20      Refactor        1. 改为包表驱动，消除 30 处重复的帧头/发送代码
  *                                             2. 删除 15 个从未启用的注释包（约 290 行）
  *                                             3. 修复 USB 接收缓冲区越界与整帧越界读
  *                                             4. 修复 ModifyDebugDataPackage 索引越界与名称未截断
  *                                             线协议字节布局未改动
  *
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#include "usb_task.h"

#include <stdbool.h>
#include <string.h>

#include "bsp_dwt.h"
#include "CRC8_CRC16.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "gimbal.h"
#include "IMU.h"
#include "macro_typedef.h"
#include "supervisory_computer_cmd.h"
#include "usb_debug.h"
#include "usb_device.h"
#include "usb_typdef.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t usb_high_water;
#endif

/*******************************************************************************/
/* Configuration                                                               */
/*******************************************************************************/

#define USB_TASK_CONTROL_TIME 1    // ms
#define USB_OFFLINE_THRESHOLD 100  // ms
#define USB_CONNECT_CNT 10

#define USB_RECEIVE_LEN 150  // 单次接收最多读取的字节数
// 接收缓存必须容纳一个最大 USB 帧和后续半帧
#define USB_RX_DATA_SIZE 512

#define HEADER_SIZE 4  // 帧头字节数

// clang-format off
/* 各包发送周期（ms）—— 发送策略集中在此，修改这里即可 */
#define SEND_DURATION_Debug               5
#define SEND_DURATION_Imu                 5
#define SEND_DURATION_RobotStateInfo      5
#define SEND_DURATION_GameStatus          100
#define SEND_DURATION_AllRobotHp          50
#define SEND_DURATION_Event               100
#define SEND_DURATION_RefereeWarning      100
#define SEND_DURATION_RobotStatus         20
#define SEND_DURATION_Buff                50
#define SEND_DURATION_Shoot               10
#define SEND_DURATION_RfidStatus          10
#define SEND_DURATION_GroundRobotPosition 50
#define SEND_DURATION_JointState          5
#define SEND_DURATION_RobotMotion         5
#define SEND_DURATION_Test                5
// clang-format on

/*******************************************************************************/
/* Packet Table                                                                */
/*                                                                             */
/* 新增包 = 加一行 + 写一个填充函数                     */
/*                                                                             */
/* X(调度名, 变量名, 包类型, 包 ID, 发送周期)                                   */
/*******************************************************************************/

// clang-format off
#define USB_SEND_PACKET_TABLE(X)                                                                        \
    X(Debug,               SEND_DATA_DEBUG,                 SendDataDebug_s,               DEBUG_DATA_SEND_ID,               SEND_DURATION_Debug)               \
    X(Imu,                 SEND_DATA_IMU,                   SendDataImu_s,                 IMU_DATA_SEND_ID,                 SEND_DURATION_Imu)                 \
    X(RobotStateInfo,      SEND_DATA_ROBOT_STATE_INFO,      SendDataRobotStateInfo_s,      ROBOT_STATE_INFO_DATA_SEND_ID,    SEND_DURATION_RobotStateInfo)      \
    X(GameStatus,          SEND_DATA_GAME_STATUS,           SendDataGameStatus_s,          GAME_STATUS_SEND_ID,              SEND_DURATION_GameStatus)          \
    X(AllRobotHp,          SEND_DATA_ALL_ROBOT_HP,          SendDataAllRobotHp_s,          ALL_ROBOT_HP_SEND_ID,             SEND_DURATION_AllRobotHp)          \
    X(Event,               SEND_DATA_EVENT,                 SendDataEvent_s,               EVENT_DATA_SEND_ID,               SEND_DURATION_Event)               \
    X(RefereeWarning,      SEND_DATA_REFEREE_WARNING,       SendDataRefereeWarning_s,      REFEREE_WARNING_SEND_ID,          SEND_DURATION_RefereeWarning)      \
    X(RobotStatus,         SEND_ROBOT_STATUS_DATA,          SendDataRobotStatus_s,         ROBOT_STATUS_SEND_ID,             SEND_DURATION_RobotStatus)         \
    X(Buff,                SEND_BUFF_DATA,                  SendDataBuff_s,                BUFF_SEND_ID,                     SEND_DURATION_Buff)                \
    X(Shoot,               SEND_SHOOT_DATA,                 SendDataShootData_s,           SHOOT_DATA_SEND_ID,               SEND_DURATION_Shoot)               \
    X(RfidStatus,          SEND_RFID_STATUS_DATA,           SendDataRfidStatus_s,          RFID_STATUS_SEND_ID,              SEND_DURATION_RfidStatus)          \
    X(GroundRobotPosition, SEND_GROUND_ROBOT_POSITION_DATA, SendDataGroundRobotPosition_s, GROUND_ROBOT_POSITION_SEND_ID,    SEND_DURATION_GroundRobotPosition) \
    X(JointState,          SEND_JOINT_STATE_DATA,           SendDataJointState_s,          JOINT_STATE_SEND_ID,              SEND_DURATION_JointState)          \
    X(RobotMotion,         SEND_ROBOT_MOTION_DATA,          SendDataRobotMotion_s,         ROBOT_MOTION_DATA_SEND_ID,        SEND_DURATION_RobotMotion)         \
    X(Test,                SEND_TEST_DATA,                  SendDataTest_s,                USB_TEST_SEND_ID,                 SEND_DURATION_Test)
// clang-format on

/*
 * 以下包的协议结构已在 usb_typdef.h 中定义，但当前未启用发送：
 *
 *   GameResult  DartInfo  PowerHeat  RobotPos  Hurt  ProjectileAllowance
 *   DartClientCmd  RadarMark  SentryInfo  RadarInfo  RobotInteraction
 *   CustomController  MapCommand  RobotCustom  RobotCustom3  PidDebug
 *
 * 启用方式：在 USB_SEND_PACKET_TABLE 中加一行 + 写一个填充函数即可，
 * 变量声明、函数声明、帧头初始化、分频调度、CRC16、发送全部自动生成。
 */

/*******************************************************************************/
/* Table Generated Macros                                                      */
/*******************************************************************************/

/** 帧开销：帧头(4) + 尾部 CRC16(2)，len 字段只统计 data 段 */
#define USB_PKT_OVERHEAD (sizeof(FrameHeader_t) + sizeof(uint16_t))

/** 生成变量声明 */
#define USB_PKT_DECL(pkt_name, pkt_var, pkt_type, pkt_id, pkt_dur) \
    static pkt_type pkt_var;                                       \
    static uint32_t LAST_T_##pkt_name;

/** 生成发送函数前向声明 */
#define USB_PKT_PROTO(pkt_name, pkt_var, pkt_type, pkt_id, pkt_dur) \
    static void UsbSend##pkt_name##Data(void);

/*
 * 1：宏参数一律加 pkt_ 前缀。若直接叫 id，会与成员 .id 同名，
 *         预处理器把 ".id" 也替换掉，编译报 "expected identifier"。
 * 2：下面两个宏会被包表逐个展开后首尾相接地排列，
 *         因此必须用"裸复合语句"而不是 do{...}while(0)——
 *         后者的结尾分号由调用处的 ';' 提供，连续展开时会缺分号。
 */

/** 初始化帧头（sof / len / id / CRC8），行为与原逐包手写版本一致 */
#define USB_PKT_INIT(pkt_name, pkt_var, pkt_type, pkt_id, pkt_dur)               \
    {                                                                             \
        (pkt_var).frame_header.sof = SEND_SOF;                                    \
        (pkt_var).frame_header.len = (uint8_t)(sizeof(pkt_type) - USB_PKT_OVERHEAD); \
        (pkt_var).frame_header.id = (pkt_id);                                     \
        append_CRC8_check_sum(                                                    \
            (uint8_t *)(&(pkt_var).frame_header), sizeof((pkt_var).frame_header)); \
    }

/** 按各自周期调度发送 */
#define USB_PKT_DISPATCH(pkt_name, pkt_var, pkt_type, pkt_id, pkt_dur) \
    {                                                                  \
        uint32_t now = HAL_GetTick();                                  \
        if ((now - LAST_T_##pkt_name) >= (pkt_dur)) {                  \
            LAST_T_##pkt_name = now;                                   \
            UsbSend##pkt_name##Data();                                 \
        }                                                              \
    }

/** 补 CRC16 并发出 */
#define USB_PKT_SEND(pkt_var, pkt_type)                                  \
    do {                                                                 \
        append_CRC16_check_sum((uint8_t *)&(pkt_var), sizeof(pkt_type)); \
        USB_Transmit((uint8_t *)&(pkt_var), sizeof(pkt_type));           \
    } while (0)

/*******************************************************************************/
/* Variables                                                                   */
/*******************************************************************************/

// 发送包实例与各自的上次发送时刻
USB_SEND_PACKET_TABLE(USB_PKT_DECL)

static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

static const Imu_t * IMU;
static const ChassisSpeedVector_t * FDB_SPEED_VECTOR;

static const game_status_t * GAME_STATUS;
static const game_robot_HP_t * GAME_ROBOT_HP;
static const event_data_t * EVENT_DATA;
static const referee_warning_t * REFEREE_WARNING;
static const robot_status_t * ROBOT_STATUS;
static const robot_pos_t * ROBOT_POS;
static const buff_t * BUFF_DATA;
static const hurt_data_t * HURT_DATA;
static const shoot_data_t * SHOOT_DATA;
static const projectile_allowance_t * PROJECTILE_ALLOWANCE;
static const rfid_status_t * RFID_STATUS;
static const ground_robot_position_t * GROUND_ROBOT_POSITION;

// 判断 USB 连接状态用到的一些变量
static bool USB_OFFLINE = true;
static uint32_t RECEIVE_TIME = 0;
static uint32_t LATEST_RX_TIMESTAMP = 0;
static uint32_t CONTINUE_RECEIVE_CNT = 0;

// 数据接收结构体
static ReceiveDataRobotCmd_s RECEIVE_ROBOT_CMD_DATA;
static ReceiveDataPidDebug_s RECEIVE_PID_DEBUG_DATA;
static ReceiveDataVirtualRc_s RECEIVE_VIRTUAL_RC_DATA;
static ReceiveDataEcho_s RECEIVE_ECHO_DATA;
static ReceiveDataRefereeInteraction_s RECEIVE_REFEREE_INTERACTION_DATA;

// 机器人控制指令数据
RobotCmdData_t ROBOT_CMD_DATA;
static RC_ctrl_t VIRTUAL_RC_CTRL;

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

static void UsbSendData(void);
static void UsbReceiveData(void);
static void UsbInit(void);
static void GetCmdData(void);
static void GetVirtualRcCtrlData(void);
static void UsbEchoReply(const ReceiveDataEcho_s * request);

// 由包表生成全部发送函数的前向声明
USB_SEND_PACKET_TABLE(USB_PKT_PROTO)

/*******************************************************************************/
/* Task                                                                        */
/*******************************************************************************/

/**
 * @brief      USB任务主函数
 * @param[in]  argument: 任务参数
 * @retval     None
 */
void usb_task(void const * argument)
{
    Publish(&ROBOT_CMD_DATA, ROBOT_CMD_DATA_NAME);
    Publish(&USB_OFFLINE, USB_OFFLINE_NAME);
    Publish(&VIRTUAL_RC_CTRL, VIRTUAL_RC_NAME);

    MX_USB_DEVICE_Init();

    vTaskDelay(10);  // 等待USB设备初始化完成
    UsbInit();

    while (1) {
        UsbSendData();
        UsbReceiveData();
        GetCmdData();
        GetVirtualRcCtrlData();

        if (HAL_GetTick() - RECEIVE_TIME > USB_OFFLINE_THRESHOLD) {
            USB_OFFLINE = true;
            CONTINUE_RECEIVE_CNT = 0;
        } else if (CONTINUE_RECEIVE_CNT > USB_CONNECT_CNT) {
            USB_OFFLINE = false;
        } else {
            CONTINUE_RECEIVE_CNT++;
        }

        vTaskDelay(USB_TASK_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        usb_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/*******************************************************************************/
/* Init                                                                        */
/*******************************************************************************/

/**
 * @brief      USB初始化
 * @param      None
 * @retval     None
 */
static void UsbInit(void)
{
    DWT_Init();  // µs 时基：测试包时间戳使用，与运行统计时基共用，幂等

    // 订阅数据
    IMU = Subscribe(IMU_NAME);                             // 获取IMU数据指针
    FDB_SPEED_VECTOR = Subscribe(CHASSIS_FDB_SPEED_NAME);  // 获取底盘速度矢量指针

    GAME_STATUS = &game_status;
    GAME_ROBOT_HP = &game_robot_HP;
    EVENT_DATA = &field_event;
    REFEREE_WARNING = &referee_warning;
    ROBOT_STATUS = &robot_status;
    ROBOT_POS = &robot_pos;
    BUFF_DATA = &buff;
    HURT_DATA = &hurt_data;
    SHOOT_DATA = &shoot_data;
    PROJECTILE_ALLOWANCE = &projectile_allowance;
    RFID_STATUS = &rfid_status;
    GROUND_ROBOT_POSITION = &ground_robot_position;

    // 数据置零
    memset(&RECEIVE_ROBOT_CMD_DATA, 0, sizeof(ReceiveDataRobotCmd_s));
    memset(&RECEIVE_PID_DEBUG_DATA, 0, sizeof(ReceiveDataPidDebug_s));
    memset(&RECEIVE_VIRTUAL_RC_DATA, 0, sizeof(ReceiveDataVirtualRc_s));
    memset(&RECEIVE_ECHO_DATA, 0, sizeof(ReceiveDataEcho_s));
    memset(
        &RECEIVE_REFEREE_INTERACTION_DATA,
        0,
        sizeof(ReceiveDataRefereeInteraction_s));
    memset(&ROBOT_CMD_DATA, 0, sizeof(RobotCmdData_t));
    memset(&VIRTUAL_RC_CTRL, 0, sizeof(RC_ctrl_t));

    // 所有上行包：帧头初始化（sof / len / id / CRC8）+ 发送时刻清零
    USB_SEND_PACKET_TABLE(USB_PKT_INIT);

    /*
     * 各包的上电固定字段。
     * 注意：这些字段不是周期性刷新的，只在这里写一次。
     */
    SEND_DATA_ROBOT_STATE_INFO.data.type.chassis = CHASSIS_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.gimbal = GIMBAL_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.shoot = SHOOT_TYPE;
    SEND_DATA_ROBOT_STATE_INFO.data.type.arm = MECHANICAL_ARM_TYPE;

    // 调试通道：类型置 1，名称留空由 ModifyDebugDataPackage 首次写入
    for (uint8_t i = 0; i < DEBUG_PACKAGE_NUM; i++) {
        SEND_DATA_DEBUG.packages[i].type = 1;
        SEND_DATA_DEBUG.packages[i].name[0] = '\0';
    }
}

/*******************************************************************************/
/* Send Dispatch                                                               */
/*******************************************************************************/

/**
 * @brief      按各包周期发送数据
 * @param      None
 * @retval     None
 * @note       与原实现相比仅有一处差异：HAL_GetTick() 只取一次快照，
 *             避免比较时刻与记录时刻跨 tick 造成最长 1ms 的周期拉伸。
 */
static void UsbSendData(void)
{
    USB_SEND_PACKET_TABLE(USB_PKT_DISPATCH)
}

/*******************************************************************************/
/* Send Fill                                                                   */
/*                                                                             */
/* 每个函数只负责"填字段"，帧头/CRC/发送由宏统一处理                            */
/*******************************************************************************/

/**
 * @brief 发送DEBUG数据
 */
static void UsbSendDebugData(void)
{
    SEND_DATA_DEBUG.time_stamp = HAL_GetTick();
    USB_PKT_SEND(SEND_DATA_DEBUG, SendDataDebug_s);
}

/**
 * @brief 发送IMU数据
 */
static void UsbSendImuData(void)
{
    if (IMU == NULL) {
        return;
    }

    SEND_DATA_IMU.time_stamp = GetImuUpdateTime();

    SEND_DATA_IMU.data.yaw = GetImuAngle(AX_YAW);
    SEND_DATA_IMU.data.pitch = GetImuAngle(AX_PITCH);
    SEND_DATA_IMU.data.roll = GetImuAngle(AX_ROLL);

    SEND_DATA_IMU.data.yaw_vel = GetImuVelocity(AX_YAW);
    SEND_DATA_IMU.data.pitch_vel = GetImuVelocity(AX_PITCH);
    SEND_DATA_IMU.data.roll_vel = GetImuVelocity(AX_ROLL);

    USB_PKT_SEND(SEND_DATA_IMU, SendDataImu_s);
}

/**
 * @brief 发送机器人信息数据
 * @note  包内 type 字段在上电时一次性写入，此处无需刷新
 */
static void UsbSendRobotStateInfoData(void)
{
    SEND_DATA_ROBOT_STATE_INFO.time_stamp = HAL_GetTick();
    USB_PKT_SEND(SEND_DATA_ROBOT_STATE_INFO, SendDataRobotStateInfo_s);
}

/**
 * @brief 发送比赛状态数据
 */
static void UsbSendGameStatusData(void)
{
    SEND_DATA_GAME_STATUS.time_stamp = HAL_GetTick();

    SEND_DATA_GAME_STATUS.data.game_type = GAME_STATUS->game_type;
    SEND_DATA_GAME_STATUS.data.game_progress = GAME_STATUS->game_progress;
    SEND_DATA_GAME_STATUS.data.stage_remain_time = GAME_STATUS->stage_remain_time;
    SEND_DATA_GAME_STATUS.data.SyncTimeStamp = GAME_STATUS->SyncTimeStamp;

    USB_PKT_SEND(SEND_DATA_GAME_STATUS, SendDataGameStatus_s);
}

/**
 * @brief 发送全场机器人hp信息数据
 */
static void UsbSendAllRobotHpData(void)
{
    SEND_DATA_ALL_ROBOT_HP.time_stamp = HAL_GetTick();

    SEND_DATA_ALL_ROBOT_HP.data.ally_1_robot_HP = GAME_ROBOT_HP->ally_1_robot_HP;
    SEND_DATA_ALL_ROBOT_HP.data.ally_2_robot_HP = GAME_ROBOT_HP->ally_2_robot_HP;
    SEND_DATA_ALL_ROBOT_HP.data.ally_3_robot_HP = GAME_ROBOT_HP->ally_3_robot_HP;
    SEND_DATA_ALL_ROBOT_HP.data.ally_4_robot_HP = GAME_ROBOT_HP->ally_4_robot_HP;
    SEND_DATA_ALL_ROBOT_HP.data.ally_7_robot_HP = GAME_ROBOT_HP->ally_7_robot_HP;
    SEND_DATA_ALL_ROBOT_HP.data.ally_outpost_HP = GAME_ROBOT_HP->ally_outpost_HP;
    SEND_DATA_ALL_ROBOT_HP.data.ally_base_HP = GAME_ROBOT_HP->ally_base_HP;

    USB_PKT_SEND(SEND_DATA_ALL_ROBOT_HP, SendDataAllRobotHp_s);
}

/**
 * @brief 发送事件数据
 */
static void UsbSendEventData(void)
{
    SEND_DATA_EVENT.time_stamp = HAL_GetTick();
    SEND_DATA_EVENT.data.event_data = EVENT_DATA->event_data;
    USB_PKT_SEND(SEND_DATA_EVENT, SendDataEvent_s);
}

/**
 * @brief 发送裁判警告数据
 */
static void UsbSendRefereeWarningData(void)
{
    SEND_DATA_REFEREE_WARNING.time_stamp = HAL_GetTick();
    SEND_DATA_REFEREE_WARNING.data.level = REFEREE_WARNING->level;
    SEND_DATA_REFEREE_WARNING.data.offending_robot_id = REFEREE_WARNING->offending_robot_id;
    SEND_DATA_REFEREE_WARNING.data.count = REFEREE_WARNING->count;

    USB_PKT_SEND(SEND_DATA_REFEREE_WARNING, SendDataRefereeWarning_s);
}

/**
 * @brief 发送机器人状态数据
 */
static void UsbSendRobotStatusData(void)
{
    SEND_ROBOT_STATUS_DATA.time_stamp = HAL_GetTick();

    SEND_ROBOT_STATUS_DATA.data.robot_id = ROBOT_STATUS->robot_id;
    SEND_ROBOT_STATUS_DATA.data.robot_level = ROBOT_STATUS->robot_level;
    SEND_ROBOT_STATUS_DATA.data.current_HP = ROBOT_STATUS->current_HP;
    SEND_ROBOT_STATUS_DATA.data.maximum_HP = ROBOT_STATUS->maximum_HP;
    SEND_ROBOT_STATUS_DATA.data.shooter_barrel_cooling_value =
        ROBOT_STATUS->shooter_barrel_cooling_value;
    SEND_ROBOT_STATUS_DATA.data.shooter_barrel_heat_limit =
        ROBOT_STATUS->shooter_barrel_heat_limit;
    SEND_ROBOT_STATUS_DATA.data.chassis_power_limit = ROBOT_STATUS->chassis_power_limit;

    SEND_ROBOT_STATUS_DATA.data.x = ROBOT_POS->x;
    SEND_ROBOT_STATUS_DATA.data.y = ROBOT_POS->y;
    SEND_ROBOT_STATUS_DATA.data.angle = ROBOT_POS->angle;

    SEND_ROBOT_STATUS_DATA.data.armor_id = HURT_DATA->armor_id;
    SEND_ROBOT_STATUS_DATA.data.HP_deduction_reason = HURT_DATA->HP_deduction_reason;

    SEND_ROBOT_STATUS_DATA.data.projectile_allowance_17mm =
        PROJECTILE_ALLOWANCE->projectile_allowance_17mm;
    SEND_ROBOT_STATUS_DATA.data.projectile_allowance_42mm =
        PROJECTILE_ALLOWANCE->projectile_allowance_42mm;
    SEND_ROBOT_STATUS_DATA.data.remaining_gold_coin =
        PROJECTILE_ALLOWANCE->remaining_gold_coin;
    SEND_ROBOT_STATUS_DATA.data.projectile_allowance_fortress =
        PROJECTILE_ALLOWANCE->projectile_allowance_fortress;

    USB_PKT_SEND(SEND_ROBOT_STATUS_DATA, SendDataRobotStatus_s);
}

/**
 * @brief 发送机器人增益和底盘能量数据
 */
static void UsbSendBuffData(void)
{
    SEND_BUFF_DATA.time_stamp = HAL_GetTick();
    SEND_BUFF_DATA.data.recovery_buff = BUFF_DATA->recovery_buff;
    SEND_BUFF_DATA.data.defence_buff = BUFF_DATA->defence_buff;
    SEND_BUFF_DATA.data.vulnerability_buff = BUFF_DATA->vulnerability_buff;
    SEND_BUFF_DATA.data.attack_buff = BUFF_DATA->attack_buff;
    SEND_BUFF_DATA.data.remaining_energy = BUFF_DATA->remaining_energy;
    USB_PKT_SEND(SEND_BUFF_DATA, SendDataBuff_s);
}

/**
 * @brief 发送机器人射击数据
 */
static void UsbSendShootData(void)
{
    SEND_SHOOT_DATA.time_stamp = HAL_GetTick();
    SEND_SHOOT_DATA.data.bullet_type = SHOOT_DATA->bullet_type;
    SEND_SHOOT_DATA.data.shooter_number = SHOOT_DATA->shooter_number;
    SEND_SHOOT_DATA.data.launching_frequency = SHOOT_DATA->launching_frequency;
    SEND_SHOOT_DATA.data.initial_speed = SHOOT_DATA->initial_speed;
    USB_PKT_SEND(SEND_SHOOT_DATA, SendDataShootData_s);
}

/**
 * @brief 发送RFID状态数据
 */
static void UsbSendRfidStatusData(void)
{
    SEND_RFID_STATUS_DATA.time_stamp = HAL_GetTick();
    SEND_RFID_STATUS_DATA.data.rfid_status = RFID_STATUS->rfid_status;
    SEND_RFID_STATUS_DATA.data.rfid_status_2 = RFID_STATUS->rfid_status_2;
    USB_PKT_SEND(SEND_RFID_STATUS_DATA, SendDataRfidStatus_s);
}

/**
 * @brief 发送地面机器人位置数据
 */
static void UsbSendGroundRobotPositionData(void)
{
    SEND_GROUND_ROBOT_POSITION_DATA.time_stamp = HAL_GetTick();
    SEND_GROUND_ROBOT_POSITION_DATA.data.hero_x = GROUND_ROBOT_POSITION->hero_x;
    SEND_GROUND_ROBOT_POSITION_DATA.data.hero_y = GROUND_ROBOT_POSITION->hero_y;
    SEND_GROUND_ROBOT_POSITION_DATA.data.engineer_x = GROUND_ROBOT_POSITION->engineer_x;
    SEND_GROUND_ROBOT_POSITION_DATA.data.engineer_y = GROUND_ROBOT_POSITION->engineer_y;
    SEND_GROUND_ROBOT_POSITION_DATA.data.standard_3_x = GROUND_ROBOT_POSITION->standard_3_x;
    SEND_GROUND_ROBOT_POSITION_DATA.data.standard_3_y = GROUND_ROBOT_POSITION->standard_3_y;
    SEND_GROUND_ROBOT_POSITION_DATA.data.standard_4_x = GROUND_ROBOT_POSITION->standard_4_x;
    SEND_GROUND_ROBOT_POSITION_DATA.data.standard_4_y = GROUND_ROBOT_POSITION->standard_4_y;
    SEND_GROUND_ROBOT_POSITION_DATA.data.reserved1 = GROUND_ROBOT_POSITION->reserved1;
    SEND_GROUND_ROBOT_POSITION_DATA.data.reserved2 = GROUND_ROBOT_POSITION->reserved2;
    USB_PKT_SEND(SEND_GROUND_ROBOT_POSITION_DATA, SendDataGroundRobotPosition_s);
}

/**
 * @brief 发送云台状态数据
 */
static void UsbSendJointStateData(void)
{
    SEND_JOINT_STATE_DATA.time_stamp = HAL_GetTick();
    SEND_JOINT_STATE_DATA.data.pitch = CmdGimbalJointState(AX_PITCH);
    SEND_JOINT_STATE_DATA.data.yaw = CmdGimbalJointState(AX_YAW);
    USB_PKT_SEND(SEND_JOINT_STATE_DATA, SendDataJointState_s);
}

/**
 * @brief 发送机器人运动数据
 */
static void UsbSendRobotMotionData(void)
{
    if (FDB_SPEED_VECTOR == NULL) {
        return;
    }

    SEND_ROBOT_MOTION_DATA.time_stamp = HAL_GetTick();

    SEND_ROBOT_MOTION_DATA.data.speed_vector.vx = FDB_SPEED_VECTOR->vx;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.vy = FDB_SPEED_VECTOR->vy;
    SEND_ROBOT_MOTION_DATA.data.speed_vector.wz = FDB_SPEED_VECTOR->wz;

    USB_PKT_SEND(SEND_ROBOT_MOTION_DATA, SendDataRobotMotion_s);
}

/**
 * @brief 发送链路测试数据包（周期包：seq + 发送时间戳，供丢包率/时序测量）
 * @note  t_pc_tx 恒为 0，PC 端据此区分周期包与回环应答包
 */
static void UsbSendTestData(void)
{
    SEND_TEST_DATA.time_stamp = HAL_GetTick();
    SEND_TEST_DATA.data.seq++;
    SEND_TEST_DATA.data.t_tx_us = DWT_GetUs();
    SEND_TEST_DATA.data.t_rx_us = 0;
    SEND_TEST_DATA.data.t_pc_tx = 0;
    // 载荷填序号派生模式，PC 端可校验字节翻转
    for (uint32_t i = 0; i < sizeof(SEND_TEST_DATA.data.payload); i++) {
        SEND_TEST_DATA.data.payload[i] = (uint8_t)(SEND_TEST_DATA.data.seq + i);
    }
    USB_PKT_SEND(SEND_TEST_DATA, SendDataTest_s);
}

/**
 * @brief 回环应答：收到 0x04 立即以 0x1F 回传
 * @note  t_rx 在 usb 任务解析时捕获（任务级上界，非中断级），t_tx - t_rx 即固件处理耗时
 */
static void UsbEchoReply(const ReceiveDataEcho_s * request)
{
    SEND_TEST_DATA.time_stamp = HAL_GetTick();
    SEND_TEST_DATA.data.seq = request->data.seq;
    SEND_TEST_DATA.data.t_pc_tx = request->data.t_pc_tx;
    SEND_TEST_DATA.data.t_rx_us = DWT_GetUs();
    memcpy(SEND_TEST_DATA.data.payload, request->data.payload, sizeof(SEND_TEST_DATA.data.payload));
    SEND_TEST_DATA.data.t_tx_us = DWT_GetUs();
    USB_PKT_SEND(SEND_TEST_DATA, SendDataTest_s);
}

/*******************************************************************************/
/* Receive                                                                     */
/*******************************************************************************/

/**
 * @brief      USB接收数据并解包
 * @param      None
 * @retval     None
 * @note       CDC 回调先按真实包长入队；此处消费字节流，保留未完成的半帧。
 */
static void UsbReceiveData(void)
{
    static uint16_t rx_length;
    uint8_t * rx_end;
    uint8_t * sof;

    while (rx_length < USB_RX_DATA_SIZE) {
        uint32_t len = USB_RX_DATA_SIZE - rx_length;
        if (len > USB_RECEIVE_LEN) {
            len = USB_RECEIVE_LEN;
        }
        if ((USB_Receive(USB_RX_BUF + rx_length, &len) != USBD_OK) || (len == 0U)) {
            break;
        }
        rx_length = (uint16_t)(rx_length + len);
    }

    if (rx_length == 0U) {
        return;
    }

    rx_end = USB_RX_BUF + rx_length;

    sof = USB_RX_BUF;
    while (sof < rx_end) {
        // 寻找帧头
        while ((sof < rx_end) && (*sof != RECEIVE_SOF)) {
            sof++;
        }
        if (sof >= rx_end) {
            break;
        }
        // 帧头必须完整，否则留到下一轮
        if ((sof + HEADER_SIZE) > rx_end) {
            break;
        }
        // 帧头 CRC8
        if (!verify_CRC8_check_sum(sof, HEADER_SIZE)) {
            sof++;
            continue;
        }

        uint8_t data_len = sof[1];
        uint8_t data_id = sof[2];
        uint32_t frame_len = (uint32_t)HEADER_SIZE + data_len + 2u;

        // 整帧必须落在缓冲区内，否则留到下一轮（不跳过，避免丢掉帧头）
        if ((sof + frame_len) > rx_end) {
            break;
        }

        if (verify_CRC16_check_sum(sof, frame_len)) {
            switch (data_id) {
                case ROBOT_CMD_DATA_RECEIVE_ID: {
                    memcpy(&RECEIVE_ROBOT_CMD_DATA, sof, sizeof(ReceiveDataRobotCmd_s));
                } break;
                case PID_DEBUG_DATA_RECEIVE_ID: {
                    memcpy(&RECEIVE_PID_DEBUG_DATA, sof, sizeof(ReceiveDataPidDebug_s));
                } break;
                case VIRTUAL_RC_DATA_RECEIVE_ID: {
                    memcpy(&RECEIVE_VIRTUAL_RC_DATA, sof, sizeof(ReceiveDataVirtualRc_s));
                } break;
                case USB_ECHO_RECEIVE_ID: {
                    memcpy(&RECEIVE_ECHO_DATA, sof, sizeof(ReceiveDataEcho_s));
                    UsbEchoReply(&RECEIVE_ECHO_DATA);
                } break;
                case REFEREE_INTERACTION_DATA_RECEIVE_ID: {
                    memcpy(
                        &RECEIVE_REFEREE_INTERACTION_DATA,
                        sof,
                        sizeof(ReceiveDataRefereeInteraction_s));
                    RefereeSendInteraction(
                        RECEIVE_REFEREE_INTERACTION_DATA.data.data_cmd_id,
                        RECEIVE_REFEREE_INTERACTION_DATA.data.sender_id,
                        RECEIVE_REFEREE_INTERACTION_DATA.data.receiver_id,
                        RECEIVE_REFEREE_INTERACTION_DATA.data.user_data,
                        sizeof(RECEIVE_REFEREE_INTERACTION_DATA.data.user_data));
                } break;
                default:
                    break;
            }
            // 时间戳用于判断上位机是否在线（用 memcpy 避免非对齐访问）
            uint32_t rx_ts;
            memcpy(&rx_ts, &sof[HEADER_SIZE], sizeof(rx_ts));
            if (rx_ts > LATEST_RX_TIMESTAMP) {
                LATEST_RX_TIMESTAMP = rx_ts;
                RECEIVE_TIME = HAL_GetTick();
            }
        }
        sof += frame_len;
    }

    // 把未解析完的半帧挪到缓冲区起始处
    uint16_t remaining = (uint16_t)(rx_end - sof);
    if ((remaining > 0u) && (sof != USB_RX_BUF)) {
        memmove(USB_RX_BUF, sof, remaining);
    }
    rx_length = remaining;
}

/*******************************************************************************/
/* Receive Decode                                                              */
/*******************************************************************************/

static void GetCmdData(void)
{
    ROBOT_CMD_DATA.speed_vector.vx = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vx;
    ROBOT_CMD_DATA.speed_vector.vy = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.vy;
    ROBOT_CMD_DATA.speed_vector.wz = RECEIVE_ROBOT_CMD_DATA.data.speed_vector.wz;

    ROBOT_CMD_DATA.chassis.yaw = RECEIVE_ROBOT_CMD_DATA.data.chassis.yaw;
    ROBOT_CMD_DATA.chassis.pitch = RECEIVE_ROBOT_CMD_DATA.data.chassis.pitch;
    ROBOT_CMD_DATA.chassis.roll = RECEIVE_ROBOT_CMD_DATA.data.chassis.roll;
    ROBOT_CMD_DATA.chassis.leg_length = RECEIVE_ROBOT_CMD_DATA.data.chassis.leg_lenth;

    ROBOT_CMD_DATA.gimbal.yaw = RECEIVE_ROBOT_CMD_DATA.data.gimbal.yaw;
    ROBOT_CMD_DATA.gimbal.pitch = RECEIVE_ROBOT_CMD_DATA.data.gimbal.pitch;

    ROBOT_CMD_DATA.shoot.fire = RECEIVE_ROBOT_CMD_DATA.data.shoot.fire;
    ROBOT_CMD_DATA.shoot.fric_on = RECEIVE_ROBOT_CMD_DATA.data.shoot.fric_on;

    ROBOT_CMD_DATA.tracking.tracking = RECEIVE_ROBOT_CMD_DATA.data.tracking.tracking;
}

static void GetVirtualRcCtrlData(void)
{
    memcpy(&VIRTUAL_RC_CTRL, &RECEIVE_VIRTUAL_RC_DATA.data, sizeof(RC_ctrl_t));
}

/*******************************************************************************/
/* Public Function                                                             */
/*******************************************************************************/

/**
 * @brief 修改调试数据包
 * @param index 索引（0 ~ DEBUG_PACKAGE_NUM-1，越界将被忽略）
 * @param data  发送数据
 * @param name  数据名称（仅在首次写入时生效；允许传 NULL 只更新数值）
 */
void ModifyDebugDataPackage(uint8_t index, float data, const char * name)
{
    if (index >= DEBUG_PACKAGE_NUM) {
        return;
    }

    SEND_DATA_DEBUG.packages[index].data = data;

    if ((name == NULL) || (SEND_DATA_DEBUG.packages[index].name[0] != '\0')) {
        return;
    }

    // 预留 1 字节给结束符，避免上位机按字符串读取时越界
    uint8_t i = 0;
    while ((name[i] != '\0') && (i < (sizeof(SEND_DATA_DEBUG.packages[index].name) - 1u))) {
        SEND_DATA_DEBUG.packages[index].name[i] = (uint8_t)name[i];
        i++;
    }
    SEND_DATA_DEBUG.packages[index].name[i] = '\0';
}

/**
 * @brief 获取上位机控制指令：云台姿态，基于欧拉角 r×p×y
 * @param axis 轴id，可配合定义好的轴id宏 AX_PITCH,AX_YAW 使用
 * @return (rad) 云台姿态
 */
inline float GetScCmdGimbalAngle(uint8_t axis)
{
    if (axis == AX_YAW) {
        return ROBOT_CMD_DATA.gimbal.yaw;
    } else if (axis == AX_PITCH) {
        return ROBOT_CMD_DATA.gimbal.pitch;
    }
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动线速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (m/s) 底盘坐标系下axis方向运动线速度
 */
inline float GetScCmdChassisSpeed(uint8_t axis)
{
    if (axis == AX_X) {
        return ROBOT_CMD_DATA.speed_vector.vx;
    } else if (axis == AX_Y) {
        return ROBOT_CMD_DATA.speed_vector.vy;
    } else if (axis == AX_Z) {
        return 0;
    }
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘坐标系下axis方向运动角速度
 * @param axis 轴id，可配合定义好的轴id宏使用
 * @return float (rad/s) 底盘坐标系下axis方向运动角速度
 */
inline float GetScCmdChassisVelocity(uint8_t axis)
{
    if (axis == AX_Z) {
        return ROBOT_CMD_DATA.speed_vector.wz;
    }
    return 0.0f;
}

/**
 * @brief 获取上位机控制指令：底盘离地高度，平衡底盘中可用作腿长参数
 * @param void
 * @return (m) 底盘离地高度
 */
inline float GetScCmdChassisHeight(void) { return ROBOT_CMD_DATA.chassis.leg_length; }

/**
 * @brief 获取上位机控制指令：开火
 * @param void
 * @return bool 是否开火
 */
inline uint8_t GetScCmdFire(void) { return ROBOT_CMD_DATA.shoot.fire; }

/**
 * @brief 获取上位机控制指令：启动摩擦轮
 * @param void
 * @return bool 是否启动摩擦轮
 */
inline uint8_t GetScCmdFricOn(void) { return ROBOT_CMD_DATA.shoot.fric_on; }

/**
 * @brief 获取上位机状态：发现敌方装甲板
 * @param void
 * @return bool 是否发现
 */
inline bool GetSCcmdtracking(void) { return ROBOT_CMD_DATA.tracking.tracking; }
/*------------------------------ End of File ------------------------------*/

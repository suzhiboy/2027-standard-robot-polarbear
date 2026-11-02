/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       referee.c/h
  * @brief      Updated the referee system Communication Protocol 1.6.1
  * @note       裁判系统通信部分
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2024/3/6         YZX             1.更新裁判系统通信协议至1.6.1（不包含半自动步兵和UI）
  *  V1.1.0     2026-4-8         CJH             1. 2026RMUL
  *
  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/
#include "referee.h"

#include "CRC8_CRC16.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "protocol.h"
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "usart.h"

#define REFEREE_TIMEOUT 300  //裁判系统超时时间
#define REFEREE_TX_QUEUE_SIZE 4U
#define REFEREE_TX_MAX_PAYLOAD (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN)

typedef struct
{
    uint16_t length;
    uint8_t data[REF_PROTOCOL_FRAME_MAX_SIZE];
} referee_tx_packet_t;

static referee_tx_packet_t referee_tx_queue[REFEREE_TX_QUEUE_SIZE];
static volatile uint8_t referee_tx_head;
static volatile uint8_t referee_tx_tail;
static volatile bool referee_tx_busy;
static volatile uint32_t referee_tx_drop_count;
static uint8_t referee_tx_active[REF_PROTOCOL_FRAME_MAX_SIZE];
static uint16_t referee_tx_active_length;
static uint8_t referee_send_sequence;
uint32_t referee_online_time = 0;
uint32_t referee_receive_count = 0;


frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;

game_status_t game_status;
game_result_t game_result;
game_robot_HP_t game_robot_HP;

event_data_t field_event;
// ext_supply_projectile_action_t supply_projectile_action_t;
// ext_supply_projectile_booking_t supply_projectile_booking_t;
dart_info_t dart_info;
referee_warning_t referee_warning;

robot_status_t robot_status;
power_heat_data_t power_heat_data;
robot_pos_t robot_pos;
buff_t buff;
// air_support_data_t robot_energy_t;
hurt_data_t hurt_data;
shoot_data_t shoot_data;
projectile_allowance_t projectile_allowance;
rfid_status_t rfid_status;
dart_client_cmd_t dart_client_cmd;
ground_robot_position_t ground_robot_position;
radar_mark_data_t radar_mark_data;
sentry_info_t sentry_info;
radar_info_t radar_info;
// ext_bullet_remaining_t bullet_remaining_t;
robot_interaction_data_t robot_interaction_data;
CustomControllerData_t custom_controller_data;  //自定义控制器数据
map_command_t map_command;
robot_custom_data_t robot_custom_data;
robot_custom_data_3_t robot_custom_data_3;                                   //0x0304



void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));
    referee_tx_head = 0;
    referee_tx_tail = 0;
    referee_tx_busy = false;
    referee_tx_drop_count = 0;
    referee_send_sequence = 0;

    memset(&game_status, 0, sizeof(game_status_t));
    memset(&game_result, 0, sizeof(game_result_t));
    memset(&game_robot_HP, 0, sizeof(game_robot_HP_t));

    memset(&field_event, 0, sizeof(event_data_t));
    // memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    // memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning, 0, sizeof(referee_warning_t));
    memset(&dart_info, 0, sizeof(dart_info_t));


    memset(&robot_status, 0, sizeof(robot_status_t));
    memset(&power_heat_data, 0, sizeof(power_heat_data_t));
    memset(&robot_pos, 0, sizeof(robot_pos_t));
    memset(&buff, 0, sizeof(buff_t));
    // memset(&robot_energy_t, 0, sizeof(air_support_data_t));
    memset(&hurt_data, 0, sizeof(hurt_data_t));
    memset(&shoot_data, 0, sizeof(shoot_data_t));
    memset(&projectile_allowance, 0, sizeof(projectile_allowance_t));
    // memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));

    memset(&robot_interaction_data, 0, sizeof(robot_interaction_data_t));
    memset(&custom_controller_data, 0, sizeof(CustomControllerData_t));

    // memset(&robot_command_t, 0, sizeof(ext_robot_command_t));

    memset(&rfid_status, 0, sizeof(rfid_status_t));
    memset(&dart_client_cmd, 0, sizeof(dart_client_cmd_t));
    memset(&ground_robot_position, 0, sizeof(ground_robot_position_t));
    memset(&radar_mark_data, 0, sizeof(radar_mark_data_t));
    memset(&sentry_info, 0, sizeof(sentry_info_t));
    memset(&radar_info, 0, sizeof(radar_info_t));
    memset(&map_command, 0, sizeof(map_command_t));
    memset(&robot_custom_data, 0, sizeof(robot_custom_data_t));
    memset(&robot_custom_data_3, 0, sizeof(robot_custom_data_3_t));
}

bool RefereeSendFrame(uint16_t cmd_id, const uint8_t * payload, uint16_t payload_len)
{
    uint8_t next_head;
    uint16_t data_length;
    uint16_t frame_length;
    referee_tx_packet_t * packet;

    if ((payload_len > REFEREE_TX_MAX_PAYLOAD) || ((payload_len > 0U) && (payload == NULL))) {
        return false;
    }

    next_head = (uint8_t)((referee_tx_head + 1U) % REFEREE_TX_QUEUE_SIZE);
    if (next_head == referee_tx_tail) {
        referee_tx_drop_count++;
        return false;
    }

    packet = &referee_tx_queue[referee_tx_head];
    packet->data[0] = HEADER_SOF;
    // 本工程协议约定 data_length 只统计 cmd_id 后的 payload 长度。
    data_length = payload_len;
    memcpy(&packet->data[1], &data_length, sizeof(data_length));
    packet->data[3] = referee_send_sequence++;
    packet->data[4] = 0;
    append_CRC8_check_sum(packet->data, REF_PROTOCOL_HEADER_SIZE);

    memcpy(&packet->data[REF_PROTOCOL_HEADER_SIZE], &cmd_id, sizeof(cmd_id));
    if (payload_len > 0U) {
        memcpy(&packet->data[REF_PROTOCOL_HEADER_SIZE + sizeof(cmd_id)], payload, payload_len);
    }

    frame_length = (uint16_t)(REF_HEADER_CRC_CMDID_LEN + payload_len);
    append_CRC16_check_sum(packet->data, frame_length);
    packet->length = frame_length;
    referee_tx_head = next_head;
    return true;
}

bool RefereeSendInteraction(
    uint16_t data_cmd_id,
    uint16_t sender_id,
    uint16_t receiver_id,
    const uint8_t * user_data,
    uint16_t user_data_len)
{
    robot_interaction_data_t interaction = {0};

    if ((user_data_len > sizeof(interaction.user_data)) ||
        ((user_data_len > 0U) && (user_data == NULL))) {
        return false;
    }

    interaction.data_cmd_id = data_cmd_id;
    interaction.sender_id = sender_id;
    interaction.receiver_id = receiver_id;
    if (user_data_len > 0U) {
        memcpy(interaction.user_data, user_data, user_data_len);
    }
    return RefereeSendFrame(
        STUDENT_INTERACTIVE_DATA_CMD_ID,
        (const uint8_t *)&interaction,
        sizeof(interaction));
}

void RefereeTxProcess(void)
{
    referee_tx_packet_t * packet;

    if (referee_tx_busy || (referee_tx_tail == referee_tx_head)) {
        return;
    }

    packet = &referee_tx_queue[referee_tx_tail];
    memcpy(referee_tx_active, packet->data, packet->length);
    referee_tx_active_length = packet->length;
    referee_tx_tail = (uint8_t)((referee_tx_tail + 1U) % REFEREE_TX_QUEUE_SIZE);
    referee_tx_busy = true;

    if (HAL_UART_Transmit_DMA(&huart6, referee_tx_active, referee_tx_active_length) != HAL_OK) {
        referee_tx_busy = false;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart)
{
    if (huart == &huart6) {
        referee_tx_busy = false;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if (huart == &huart6) {
        referee_tx_busy = false;
    }
}

uint32_t RefereeGetTxDropCount(void)
{
    return referee_tx_drop_count;
}

void referee_data_solve(uint8_t * frame)
{
    if (HAL_GetTick() - referee_online_time > REFEREE_TIMEOUT) {
        referee_receive_count = 0;
    }
    referee_receive_count++;

    uint16_t cmd_id = 0;

    uint8_t index = 0;

    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t));

    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);

    switch (cmd_id) {
        case GAME_STATE_CMD_ID: {
            memcpy(&game_status, frame + index, sizeof(game_status_t));
            referee_online_time = HAL_GetTick();
        } break;
        case GAME_RESULT_CMD_ID: {
            memcpy(&game_result, frame + index, sizeof(game_result_t));
            referee_online_time = HAL_GetTick();
        } break;
        case GAME_ROBOT_HP_CMD_ID: {
            memcpy(&game_robot_HP, frame + index, sizeof(game_robot_HP_t));
            referee_online_time = HAL_GetTick();
        } break;

        case FIELD_EVENTS_CMD_ID: {
            memcpy(&field_event, frame + index, sizeof(event_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        // case SUPPLY_PROJECTILE_ACTION_CMD_ID: {
        //     memcpy(
        //         &supply_projectile_action_t, frame + index, sizeof(ext_supply_projectile_action_t));
        //     referee_online_time = HAL_GetTick();
        // } break;
        // case SUPPLY_PROJECTILE_BOOKING_CMD_ID: {
        //     memcpy(
        //         &supply_projectile_booking_t, frame + index,
        //         sizeof(ext_supply_projectile_booking_t));
        //     referee_online_time = HAL_GetTick();
        // } break;
        case REFEREE_WARNING_CMD_ID: {
            memcpy(&referee_warning, frame + index, sizeof(referee_warning_t));
            referee_online_time = HAL_GetTick();
        } break;
        case DART_INFO_CMD_ID: {
            memcpy(&dart_info, frame + index, sizeof(dart_info_t));
            referee_online_time = HAL_GetTick();
        } break;
        case ROBOT_STATE_CMD_ID: {
            memcpy(&robot_status, frame + index, sizeof(robot_status_t));
            referee_online_time = HAL_GetTick();
        } break;
        case POWER_HEAT_DATA_CMD_ID: {
            memcpy(&power_heat_data, frame + index, sizeof(power_heat_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case ROBOT_POS_CMD_ID: {
            memcpy(&robot_pos, frame + index, sizeof(robot_pos_t));
            referee_online_time = HAL_GetTick();
        } break;
        case BUFF_MUSK_CMD_ID: {
            memcpy(&buff, frame + index, sizeof(buff_t));
            referee_online_time = HAL_GetTick();
        } break;
        // case AERIAL_ROBOT_ENERGY_CMD_ID: {
        //     memcpy(&robot_energy_t, frame + index, sizeof(air_support_data_t));
        //     referee_online_time = HAL_GetTick();
        // } break;
        case ROBOT_HURT_CMD_ID: {
            memcpy(&hurt_data, frame + index, sizeof(hurt_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case SHOOT_DATA_CMD_ID: {
            memcpy(&shoot_data, frame + index, sizeof(shoot_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        // case BULLET_REMAINING_CMD_ID: {
        //     memcpy(&bullet_remaining_t, frame + index, sizeof(ext_bullet_remaining_t));
        //     referee_online_time = HAL_GetTick();
        // } break;
        case PROJECTILE_ALLOWANCE_CMD_ID: {
            memcpy(&projectile_allowance, frame + index, sizeof(projectile_allowance_t));
            referee_online_time = HAL_GetTick();
        } break;
        case RFID_STATUS_CMD_ID: {
            memcpy(&rfid_status, frame + index, sizeof(rfid_status_t));
            referee_online_time = HAL_GetTick();
        } break;
        case DART_CLIENT_CMD_ID: {
            memcpy(&dart_client_cmd, frame + index, sizeof(dart_client_cmd_t));
            referee_online_time = HAL_GetTick();
        } break;
        case GROUND_ROBOT_POSITION_CMD_ID: {
            memcpy(&ground_robot_position, frame + index, sizeof(ground_robot_position_t));
            referee_online_time = HAL_GetTick();
        } break;
        case RADAR_MARK_DATA_CMD_ID: {
            memcpy(&radar_mark_data, frame + index, sizeof(radar_mark_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case SENTRY_INFO_CMD_ID: {
            memcpy(&sentry_info, frame + index, sizeof(sentry_info_t));
            referee_online_time = HAL_GetTick();
        } break;
        case RADAR_INFO_CMD_ID: {
            memcpy(&radar_info, frame + index, sizeof(radar_info_t));
            referee_online_time = HAL_GetTick();
        } break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID: {
            memcpy(&robot_interaction_data, frame + index, sizeof(robot_interaction_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case CUSTOM_CONTROLLER_CMD_ID: {
            memcpy(&custom_controller_data, frame + index, sizeof(CustomControllerData_t));
            referee_online_time = HAL_GetTick();
        } break;
        case MAP_COMMAND_CMD_ID: {
            memcpy(&map_command, frame + index, sizeof(map_command_t));
            referee_online_time = HAL_GetTick();
        } break;
        // case ROBOT_COMMAND_CMD_ID: {
        //     memcpy(&robot_command_t, frame + index, sizeof(ext_robot_command_t));
        //     referee_online_time = HAL_GetTick();
        // } break;
        case ROBOT_CUSTOM_DATA_CMD_ID: {
            memcpy(&robot_custom_data, frame + index, sizeof(robot_custom_data_t));
            referee_online_time = HAL_GetTick();
        } break;
        case ROBOT_CUSTOM_DATA_3_CMD_ID: {
            memcpy(&robot_custom_data_3, frame + index, sizeof(robot_custom_data_3_t));
            referee_online_time = HAL_GetTick();
        } break;
        default: {
            referee_receive_count--;
            break;
        }
    }
}

void get_chassis_power_and_buffer(fp32 * power, fp32 * buffer)
{
    *power = power_heat_data.chassis_power;
    *buffer = power_heat_data.buffer_energy;
}

uint8_t get_robot_id(void) { return robot_status.robot_id; }

void get_shoot_heat0_limit_and_heat0(uint16_t * heat0_limit, uint16_t * heat0)
{
    *heat0_limit = robot_status.shooter_barrel_heat_limit;
    *heat0 = power_heat_data.shooter_17mm_barrel_heat;
}




void get_shoot_heat42_limit_and_heat42(uint16_t * heat_limit, uint16_t * heat)
{
    *heat_limit = robot_status.shooter_barrel_heat_limit;
    *heat = power_heat_data.shooter_42mm_barrel_heat;
}

/**
 * @brief 反馈机器人颜色
 * @param  none
 * @return 0为红色,1为蓝色,2为未知
 */
uint8_t get_team_color(void)  // 谨防“哨兵在打我”
{
    switch (robot_status.robot_id) {
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
        case 8:
        case 9:
        case 10:
        case 11:
            return 0;
        case 101:
        case 102:
        case 103:
        case 104:
        case 105:
        case 106:
        case 107:
        case 108:
        case 109:
        case 110:
        case 111:
            return 1;
        default:
            return 2;
    }
}

uint16_t get_shoot_heat(void)  // 双枪管哨兵
{
    return power_heat_data.shooter_17mm_barrel_heat;

}

robot_custom_data_t * GetCustomControllerDataPoint(void) { return &robot_custom_data; }

/*========== API ==========*/

inline bool GetRefereeOffline(void)
{
    uint32_t current_time = HAL_GetTick();
    return !((referee_receive_count > 5) && (current_time - referee_online_time < REFEREE_TIMEOUT));
}

/**
 * @brief 获取自定义控制器数据
 * @param  index 数据索引
 * @return float 数据
 */
inline float GetCustomControllerPos(uint8_t index)
{
    float data = 0;
    memcpy(&data, &robot_custom_data.data[index * 4], 4);
    return data;
    // return *((float *)(&custom_robot_data.data[index * 4]));
}
// /**
//  * @brief 获取事件数据
//  * @param  无
//  * @return uint32_t 
//  */
// inline event_data_t GetEventData(void){return event_data;}

// /**
//  * @brief 获取机器人血量数据
//  * @param  无
//  * @return game_robot_HP_t
//  */
// inline game_robot_HP_t GetGameRobotHP(void){return game_robot_HP;}

// /**
//  * @brief 获取比赛信息
//  * @param  无
//  * @return game_status_t
//  */
// inline game_status_t GetGameStatus(void){return game_status;}

// /**
//  * @brief 获取地面机器人位置信息
//  * @param  无
//  * @return ground_robot_position_t
//  */
// inline ground_robot_position_t GetGroundRobotPosition(void){return ground_robot_position;}

// /**
//  * @brief 获取RFID信息
//  * @param  无
//  * @return rfid_status_t
//  */
// inline rfid_status_t GetRFIDStatus(void){return rfid_status;}


// /**
//  * @brief 获取机器人信息
//  * @param  无
//  * @return robot_status_t
//  */
// inline robot_status_t GetRobotStatus(void){return robot_status;}

// /**
//  * @brief 获取能量热量信息
//  * @param  无
//  * @return power_heat_data_t
//  */
// inline power_heat_data_t GetPoweHeatData(void){return power_heat_data;}

// /**
//  * @brief 获取机器人位姿信息
//  * @param  无
//  * @return robot_pos_t
//  */
// inline robot_pos_t GetRobotPos(void){return robot_pos;}

// /**
//  * @brief 获取受伤信息
//  * @param  无
//  * @return hurt_data_t
//  */
// inline hurt_data_t GetHurtData(void){return hurt_data;}

// /**
//  * @brief 获取允许发弹量信息
//  * @param  无
//  * @return hurt_data_t
//  */
// inline projectile_allowance_t GetProjectiliAllowance(void){return projectile_allowance;}

// /**
//  * @brief 获取buff信息
//  * @param  无
//  * @return buff_t
//  */
// inline buff_t GetBuff(void){return buff;}
/*------------------------------ End of File ------------------------------*/

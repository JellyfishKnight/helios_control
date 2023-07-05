#include "Referee.h"

#define BLUE 0
#define RED 1

/*****************系统数据定义**********************/
ext_judge_system_data judge_system_data;
/****************************************************/

/**
  * @brief  判断自己红蓝方
  * @param  void
  * @retval RED   BLUE
  * @attention  数据打包,打包完成后通过串口发送到裁判系统
  */
bool Color;
bool is_red_or_blue(void)
{
	if (judge_system_data.GameRobotStat.robot_id > 10)
	{
		return BLUE;
	}
	else
	{
		return RED;
	}
}

/*****************机器人血量数据;分为1，2，3,4部分**********************/
void get_robo_hp_one(ext_game_robot_HP_t *ptr, uint8_t *Data)
{
	ptr->red_1_robot_HP =(uint16_t)(Data[0]<<8 | Data[1]);
	ptr->red_2_robot_HP =(uint16_t)(Data[2]<<8 | Data[3]);
	ptr->red_3_robot_HP =(uint16_t)(Data[4]<<8 | Data[5]);
	ptr->red_4_robot_HP =(uint16_t)(Data[6]<<8 | Data[7]);
}

void get_robo_hp_two(ext_game_robot_HP_t *ptr, uint8_t *Data)
{
	ptr->red_5_robot_HP =(uint16_t)(Data[0]<<8 | Data[1]);
	ptr->red_7_robot_HP =(uint16_t)(Data[2]<<8 | Data[3]);
	ptr->red_outpost_HP =(uint16_t)(Data[4]<<8 | Data[5]);
	ptr->red_base_HP    =(uint16_t)(Data[6]<<8 | Data[7]);
}

void get_robo_hp_three(ext_game_robot_HP_t *ptr, uint8_t *Data)
{
	ptr->blue_1_robot_HP =(uint16_t)(Data[0]<<8 | Data[1]);
	ptr->blue_2_robot_HP =(uint16_t)(Data[2]<<8 | Data[3]);
	ptr->blue_3_robot_HP =(uint16_t)(Data[4]<<8 | Data[5]);
	ptr->blue_4_robot_HP =(uint16_t)(Data[6]<<8 | Data[7]);
}

void get_robo_hp_four(ext_game_robot_HP_t *ptr, uint8_t *Data)
{
	ptr->blue_5_robot_HP =(uint16_t)(Data[0]<<8 | Data[1]);
	ptr->blue_7_robot_HP =(uint16_t)(Data[2]<<8 | Data[3]);
	ptr->blue_outpost_HP =(uint16_t)(Data[4]<<8 | Data[5]);
}

/*****************补给站动作标识**********************/
void get_supply_projectile_action(ext_supply_projectile_action_t *ptr, uint8_t *Data)
{
		ptr->supply_projectile_id   = Data[0];
	 	ptr->supply_robot_id        = Data[1];
		ptr->supply_projectile_step = Data[2];
  	ptr->supply_projectile_num  = Data[3];
}

/*****************比赛机器人状态数据：分为1，2，3，4部分**********************/
void get_game_robot_state_one(ext_game_robot_state_t *ptr, uint8_t *Data)
{
		ptr->robot_id    = Data[0];
	 	ptr->robot_level = Data[1];
		ptr->remain_HP   = (uint16_t)(Data[2]<<8 | Data[3]);
  	ptr->max_HP      = (uint16_t)(Data[4]<<8 | Data[5]);
	  ptr->shooter_id1_17mm_cooling_rate   = (uint16_t)(Data[6]<<8 | Data[7]);
}

void get_game_robot_state_two(ext_game_robot_state_t *ptr, uint8_t *Data)
{
		ptr->shooter_id1_17mm_cooling_limit = (uint16_t)(Data[0]<<8 | Data[1]);
		ptr->shooter_id1_17mm_speed_limit   = (uint16_t)(Data[2]<<8 | Data[3]);
  	ptr->shooter_id2_17mm_cooling_rate  = (uint16_t)(Data[4]<<8 | Data[5]);
		ptr->shooter_id2_17mm_cooling_rate  = (uint16_t)(Data[6]<<8 | Data[7]);
}

void get_game_robot_state_three(ext_game_robot_state_t *ptr, uint8_t *Data)
{
		ptr->shooter_id2_17mm_speed_limit    = (uint16_t)(Data[0]<<8 | Data[1]);
		ptr->shooter_id1_42mm_cooling_rate   = (uint16_t)(Data[2]<<8 | Data[3]);
  	ptr->shooter_id1_42mm_cooling_limit  = (uint16_t)(Data[4]<<8 | Data[5]);
		ptr->shooter_id1_42mm_speed_limit    = (uint16_t)(Data[6]<<8 | Data[7]);
}

void get_game_robot_state_four(ext_game_robot_state_t *ptr, uint8_t *Data)
{
		ptr->chassis_power_limit         = (uint16_t)(Data[0]<<8 | Data[1]);
	 	ptr->mains_power_gimbal_output   = Data[2];
		ptr->mains_power_chassis_output  = Data[3];
  	ptr->mains_power_shooter_output  = Data[4];
}

/*****************实时功率热量数据：分为1，2部分**********************/
void get_power_heat_data_one(ext_power_heat_data_t *ptr, uint8_t *Data)
{
		ptr->chassis_volt           = (uint16_t)(Data[0]<<8 | Data[1]);
	 	ptr->chassis_current        = (uint16_t)(Data[2]<<8 | Data[3]);
		((uint8_t*)(&ptr->chassis_power))[0]=Data[4];
  	((uint8_t*)(&ptr->chassis_power))[1]=Data[5];
	  ((uint8_t*)(&ptr->chassis_power))[2]=Data[6];
  	((uint8_t*)(&ptr->chassis_power))[3]=Data[7];
}

void get_power_heat_data_two(ext_power_heat_data_t *ptr, uint8_t *Data)
{
		ptr->chassis_power_buffer           = (uint16_t)(Data[0]<<8 | Data[1]);
	 	ptr->shooter_id1_17mm_cooling_heat  = (uint16_t)(Data[2]<<8 | Data[3]);
		ptr->shooter_id2_17mm_cooling_heat  = (uint16_t)(Data[4]<<8 | Data[5]);
	 	ptr->shooter_id1_42mm_cooling_heat  = (uint16_t)(Data[6]<<8 | Data[7]);
	  ptr->shooter_id1_17mm_residual_cooling_heat = judge_system_data.GameRobotStat.shooter_id1_17mm_cooling_limit - ptr->shooter_id1_17mm_cooling_heat;
	  ptr->shooter_id2_17mm_residual_cooling_heat = judge_system_data.GameRobotStat.shooter_id2_17mm_cooling_limit - ptr->shooter_id2_17mm_cooling_heat;
		ptr->shooter_id1_42mm_residual_cooling_heat = judge_system_data.GameRobotStat.shooter_id1_42mm_cooling_limit - ptr->shooter_id1_42mm_cooling_heat;
}

/*****************伤害状态数据**********************/
void get_robot_hurt(ext_robot_hurt_t *ptr, uint8_t *Data)
{
		ptr->armor_id  = Data[0];
	 	ptr->hurt_type = Data[1];
}

/*****************机器人增益数据**********************/
void get_buff_musk(ext_buff_musk_t *ptr, uint8_t *Data)
{
		ptr->power_rune_buff  = Data[0];
}

/*****************实时射击数据 **********************/
void get_shoot_data(ext_shoot_data_t *ptr, uint8_t *Data)
{
		ptr->bullet_type = Data[0];
	  ptr->shooter_id  = Data[1];
	 	ptr->bullet_freq = Data[2];
	  ((uint8_t*)(&ptr->bullet_speed))[0]=Data[3];
  	((uint8_t*)(&ptr->bullet_speed))[1]=Data[4];
	  ((uint8_t*)(&ptr->bullet_speed))[2]=Data[5];
  	((uint8_t*)(&ptr->bullet_speed))[3]=Data[6];
}

/*****************剩余子弹数量**********************/
void get_bullet_remaining(ext_bullet_remaining_t*ptr, uint8_t *Data)
{
		ptr->bullet_remaining_num_17mm = (uint16_t)(Data[0]<<8 | Data[1]);
		ptr->bullet_remaining_num_42mm = (uint16_t)(Data[2]<<8 | Data[3]);
  	ptr->coin_remaining_num        = (uint16_t)(Data[4]<<8 | Data[5]);
}

/*****************机器人 RFID 状态**********************/
void get_rfid_status(ext_rfid_status_t *ptr, uint8_t *Data)
{
		((uint8_t*)(&ptr->rfid_status))[0]=Data[0];
  	((uint8_t*)(&ptr->rfid_status))[1]=Data[1];
	  ((uint8_t*)(&ptr->rfid_status))[2]=Data[2];
  	((uint8_t*)(&ptr->rfid_status))[3]=Data[3];
}

/*****************超级电容剩余量**********************/
void get_supercap_remain(ext_judge_system_data *ptr, uint8_t *Data)
{
	((uint8_t *)(&ptr->SuperCapRemain))[0] = Data[0];
	((uint8_t *)(&ptr->SuperCapRemain))[1] = Data[1];
	((uint8_t *)(&ptr->SuperCapRemain))[2] = Data[2];
	((uint8_t *)(&ptr->SuperCapRemain))[3] = Data[3];
//	((uint32_t)&ptr->SuperCapRemain[4]
}

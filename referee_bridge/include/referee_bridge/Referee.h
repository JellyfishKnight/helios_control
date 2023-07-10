#ifndef __JUDGE_H__
#define __JUDGE_H__


#include <iostream>
#include <stdbool.h>
#include <stdint.h>
#include <CRC.h>
#define __packed __attribute__((__packed__))
/** 移植：胡廷文
  * 注解：裁判系统中，所有结构体使用__packed关键字进行字节对齐
	*      __packed字节对齐，比如说int float double char它的总大小是4 + 4 + 8 + 1 = 17。
	*      如果不用__packed，系统将以默认的方式对齐（32位系统是4字节），那么它占4 + 4 + 8 + 4 = 20（不足4字节以4字节补齐）。
	*      2021.10.2
	*/

typedef enum
{ 
	ID_game_state_t       		               	= 0x0F1,	//比赛状态数据
	ID_game_result_t 	   				              = 0x0F2,	//比赛结果数据        
	
	ID_game_robot_HP_one_t                    = 0x013,	//比赛机器人存活数据 1           *****
	ID_game_robot_HP_two_t                    = 0x023,	//比赛机器人存活数据 2           *****
	ID_game_robot_HP_three_t                  = 0x033,	//比赛机器人存活数据 3           *****
	ID_game_robot_HP_four_t                   = 0x043,	//比赛机器人存活数据 4           *****
	
	ID_dart_status_t                          = 0x0F4,	//人工智能挑战赛加成与惩罚区状态
	ID_ICRA_buff_debuff_zone_status_t         = 0x0F5,	//飞镖发射状态
	ID_event_data_t  				           	      = 0x1F1,	//场地事件数据 
	ID_supply_projectile_action_t   	        = 0x1F2,	//补给站动作标识                 *****
	ID_referee_warning_t 	                    = 0x1F4,	//裁判警告信息
	ID_dart_remaining_time_t                  = 0x1F5,	//飞镖发射口倒计时
	
	ID_game_robot_state_one_t    			        = 0x211,	//比赛机器人状态数据 1           *****
	ID_game_robot_state_two_t   			        = 0x221,	//比赛机器人状态数据 2           *****
  ID_game_robot_state_three_t    			  	  = 0x231,	//比赛机器人状态数据 3           *****
	ID_game_robot_state_four_t                = 0x241,	//比赛机器人状态数据 4           *****
		
	ID_power_heat_data_one_t    			        = 0x101,	//实时功率热量数据   1           *****
	ID_power_heat_data_two_t    			        = 0x102,	//实时功率热量数据   2           *****
	
	ID_game_robot_pos_t        	              = 0x2F3,	//机器人位置数据
	ID_buff_musk_t					                  = 0x2F4,	//机器人增益数据                 *****
	ID_aerial_robot_energy_t			            = 0x2F5,	//空中机器人能量状态数据
	ID_robot_hurt_t					                  = 0x2F6,	//伤害状态数据                   *****
	ID_shoot_data_t					                  = 0x2F7,	//实时射击数据                   *****
	ID_bullet_remaining_t                     = 0x2F8,	//子弹剩余发射数                 *****            
  ID_rfid_status_t                          = 0x2F9,	//机器人 RFID 状态               *****
	ID_dart_client_cmd_t                      = 0x2FA,	//飞镖机器人客户端指令数据
	ID_super_cap                              = 0x03F5 	//超级电容 
} CmdID;

typedef enum
{
	
	ID_all_state_tx                         = 0x0A1

} TX_CmdID;


/* ID: 0x0001  Byte:  3    比赛状态数据 */
typedef struct __packed 
{ 
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t  SyncTimeStamp;
} ext_game_state_t; 


/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef struct __packed
{ 
	uint8_t winner;
} ext_game_result_t; 


/* ID: 0x0003  Byte:  2    比赛机器人存活数据 */
typedef struct __packed
{

 uint16_t red_1_robot_HP;
 uint16_t red_2_robot_HP;
 uint16_t red_3_robot_HP;
 uint16_t red_4_robot_HP;
 uint16_t red_5_robot_HP;
 uint16_t red_7_robot_HP;
 uint16_t red_outpost_HP;
 uint16_t red_base_HP;
 uint16_t blue_1_robot_HP;
 uint16_t blue_2_robot_HP;
 uint16_t blue_3_robot_HP;
 uint16_t blue_4_robot_HP;
 uint16_t blue_5_robot_HP;
 uint16_t blue_7_robot_HP;
uint16_t blue_outpost_HP;
} ext_game_robot_HP_t;


/* ID: 0x0004  Byte:  3    飞镖发射状态 */
typedef  struct __packed
{
 uint8_t dart_belong;
 uint16_t stage_remaining_time;
} ext_dart_status_t;


/* ID: 0x0005  Byte:  3    . 人工智能挑战赛加成与惩罚区状态(用不到) */
typedef  struct __packed
{
 uint8_t F1_zone_status:1;
 uint8_t F1_zone_buff_debuff_status:3;
 uint8_t F2_zone_status:1;
 uint8_t F2_zone_buff_debuff_status:3;
 uint8_t F3_zone_status:1;
 uint8_t F3_zone_buff_debuff_status:3;
 uint8_t F4_zone_status:1;
 uint8_t F4_zone_buff_debuff_status:3;
 uint8_t F5_zone_status:1;
 uint8_t F5_zone_buff_debuff_status:3;
 uint8_t F6_zone_status:1;
 uint8_t F6_zone_buff_debuff_status:3;
	
 uint16_t red1_bullet_left;
 uint16_t red2_bullet_left;
 uint16_t blue1_bullet_left;
 uint16_t blue2_bullet_left;	
}ext_ICRA_buff_debuff_zone_status_t;




/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef  struct __packed
{ 
	uint32_t event_type;
} ext_event_data_t; 


/* ID: 0x0102  Byte:  4    场地补给站动作标识数据 */
typedef  struct __packed
{
 uint8_t supply_projectile_id;
 uint8_t supply_robot_id;
 uint8_t supply_projectile_step;
 uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


/* ID: 0x0104  Byte:  2    裁判警告信息 */
typedef  struct __packed
{
 uint8_t level;
	
 uint8_t foul_robot_id;
} ext_referee_warning_t;


/* ID: 0x0105  Byte:  1    飞镖发射口倒计时 */
typedef  struct __packed
{
 uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;


/* ID: 0X0201  Byte: 18    机器人状态数据 */
typedef  struct __packed
{ 
	uint8_t robot_id;                       //机器人ID，可用来校验发送
	uint8_t robot_level;                    //1一级，2二级，3三级
	uint16_t remain_HP;                     //机器人剩余血量
	uint16_t max_HP;                        //机器人满血量
  uint16_t shooter_id1_17mm_cooling_rate;
	
  uint16_t shooter_id1_17mm_cooling_limit;
  uint16_t shooter_id1_17mm_speed_limit;
  uint16_t shooter_id2_17mm_cooling_rate;
  uint16_t shooter_id2_17mm_cooling_limit;
	
  uint16_t shooter_id2_17mm_speed_limit;
  uint16_t shooter_id1_42mm_cooling_rate;
  uint16_t shooter_id1_42mm_cooling_limit;
  uint16_t shooter_id1_42mm_speed_limit;
	
  uint16_t chassis_power_limit;
  uint8_t mains_power_gimbal_output : 1;
  uint8_t mains_power_chassis_output : 1;
  uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t; 


/* ID: 0X0202  Byte: 16    实时功率热量数据 */
typedef  struct __packed
{ 
 uint16_t chassis_volt;
 uint16_t chassis_current;
 float chassis_power;
 uint16_t chassis_power_buffer;
 uint16_t shooter_id1_17mm_cooling_heat;
 uint16_t shooter_id2_17mm_cooling_heat;
 uint16_t shooter_id1_42mm_cooling_heat;
 int16_t shooter_id1_17mm_residual_cooling_heat;	//剩余热量，非官方类型
 int16_t shooter_id2_17mm_residual_cooling_heat;	//剩余热量，非官方类型
 int16_t shooter_id1_42mm_residual_cooling_heat;	//剩余热量，非官方类型
} ext_power_heat_data_t; 


/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef  struct __packed
{   
	float x;   
	float y;   
	float z;   
	float yaw; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef  struct __packed
{ 
	uint8_t power_rune_buff;  // bit 0：机器人血量补血状态 bit 1：枪口热量冷却加速 bit 2：机器人防御加成 bit 3：机器人攻击加成
} ext_buff_musk_t; 


/* ID: 0x0205  Byte:  3    空中机器人能量状态数据 */
typedef  struct __packed
{ 
	uint8_t attack_time; 
} aerial_robot_energy_t; 


/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef  struct __packed
{ 
	uint8_t armor_id : 4; 
	uint8_t hurt_type : 4; 
} ext_robot_hurt_t; 


/* ID: 0x0207  Byte:  6    实时射击数据 */
typedef  struct __packed
{ 
 uint8_t bullet_type;
 uint8_t shooter_id;
 uint8_t bullet_freq;
 float bullet_speed; 
} ext_shoot_data_t; 

/* ID: 0x0208  Byte:  2    子弹剩余发射数 */
typedef  struct __packed
{
 uint16_t bullet_remaining_num_17mm;
 uint16_t bullet_remaining_num_42mm;
 uint16_t coin_remaining_num;
} ext_bullet_remaining_t;


/* ID: 0x0209  Byte:  4    机器人 RFID 状态 */
typedef  struct __packed
{
 uint32_t rfid_status;
} ext_rfid_status_t;

//ID：0x020A  Byte:  12    飞镖机器人客户端指令数据
typedef  struct __packed
{
 uint8_t dart_launch_opening_status;
 uint8_t dart_attack_target;
 uint16_t target_change_time;
 uint8_t first_dart_speed;
 uint8_t second_dart_speed;
 uint8_t third_dart_speed;
 uint8_t fourth_dart_speed;
 uint16_t last_dart_launch_time;
 uint16_t operate_launch_cmd_time;
}ext_dart_client_cmd_t;

typedef struct __packed
{
	ext_game_robot_HP_t                  RoboHP;                  //0x0003 机器人血量   *****
	ext_supply_projectile_action_t		   SupplyProjectileAction;	//0x0102 补给站       *****
  ext_game_robot_state_t			  	     GameRobotStat;				    //0x0201 机器人状态   *****
  ext_power_heat_data_t		  		       PowerHeatData;				    //0x0202 功率热量     *****
  ext_buff_musk_t					             BuffMusk;				        //0x0204 增益数据     *****
  ext_robot_hurt_t				             RobotHurt;					      //0x0206 伤害类型     *****
  ext_shoot_data_t					           ShootData;					      //0x0207 射击信息     *****
  ext_bullet_remaining_t               ReBullet;                //0x0208 剩余子弹数量 ***** 
  ext_rfid_status_t                    RFIDState;               //0x0209 RFID状态     *****
	float														 SuperCapRemain;					//超级电容剩余量

}ext_judge_system_data;

typedef struct __packed
{
    uint8_t sof;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
    uint16_t cmd_id;
    uint8_t data[256]{};
    bool CheckHeaderCRC8() {
      return Verify_CRC8_Check_Sum((uint8_t *)this, GetHeaderLength());
    }
    bool CheckPackCRC16()
    {
      return Verify_CRC16_Check_Sum((uint8_t *)this, GetLength());
    }
    uint16_t CheckTail()
    {
      uint16_t tail = *(uint16_t *)((uint8_t *)this + GetLength() - 2);
      return tail;
    }
    void AddCRC()
    {
      Append_CRC8_Check_Sum((uint8_t *)this, GetHeaderLength());
      Append_CRC16_Check_Sum((uint8_t *)this, GetLength());
    }
    size_t GetHeaderLength() { return 5; }
    size_t GetLength() { return 7 + data_length + 2; }
} FrameBuffer;


/*****************系统数据定义**********************/
extern ext_judge_system_data judge_system_data;

/****************************************************/

/*****************机器人血量数据**********************/
void get_robo_hp_one(ext_game_robot_HP_t *ptr, uint8_t *Data);
void get_robo_hp_two(ext_game_robot_HP_t *ptr, uint8_t *Data);
void get_robo_hp_three(ext_game_robot_HP_t *ptr, uint8_t *Data);
void get_robo_hp_four(ext_game_robot_HP_t *ptr, uint8_t *Data);

/*****************补给站动作标识**********************/
void get_supply_projectile_action(ext_supply_projectile_action_t *ptr, uint8_t *Data);

/*****************比赛机器人状态数据：分为1，2，3部分**********************/
void get_game_robot_state_one(ext_game_robot_state_t *ptr, uint8_t *Data);
void get_game_robot_state_two(ext_game_robot_state_t *ptr, uint8_t *Data);
void get_game_robot_state_three(ext_game_robot_state_t *ptr, uint8_t *Data);
void get_game_robot_state_four(ext_game_robot_state_t *ptr, uint8_t *Data);

/*****************实时功率热量数据：分为1，2部分**********************/
void get_power_heat_data_one(ext_power_heat_data_t *ptr, uint8_t *Data);
void get_power_heat_data_two(ext_power_heat_data_t *ptr, uint8_t *Data);

/*****************伤害状态数据**********************/
void get_robot_hurt(ext_robot_hurt_t *ptr, uint8_t *Data);

/*****************机器人增益数据**********************/
void get_buff_musk(ext_buff_musk_t *ptr, uint8_t *Data);

/*****************实时射击数据 **********************/
void get_shoot_data(ext_shoot_data_t *ptr, uint8_t *Data);

/*****************剩余子弹数量**********************/
void get_bullet_remaining(ext_bullet_remaining_t*ptr, uint8_t *Data);

/*****************机器人 RFID 状态**********************/
void get_rfid_status(ext_rfid_status_t *ptr, uint8_t *Data);

/*****************超级电容剩余量**********************/
void get_supercap_remain(ext_judge_system_data *ptr, uint8_t *Data);
	
/*****************判断红蓝方**********************/
bool is_red_or_blue(void);

/***************判断裁判系统是否连接**********************/
void Judge_IF_REF_ONL(void);
#endif


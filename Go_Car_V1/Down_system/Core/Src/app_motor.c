#include "app_motor.h"
#include "system_cfg.h"
#include <string.h>
#include "cJSON.h"
#include "usart.h"

/**
 * 现在所用的直流减速电机的型号为:
 * 单向脉冲编码器
 * 减速比：1:48
 * 空载转速：200RPM
 * 编码器线数：11线
 * 编码器精度：528线
 * 
 * 小车直径65mm，一圈的距离C = π * d = 3.14 * 65 mm = 204mm ，转一圈的脉冲为528个，一个脉冲距离为204/528 = 0.387mm
 * 编码器捕获采用双边沿计数，编码器精度528线，读取脉冲计数为528 * 2 = 1056 个脉冲
 * 调速周期为20ms一次，1秒调速50次，读取码盘的周期为20ms，注意全速下，码盘脉冲周期为300us左右，20ms最多计数140次左右
 * 最小速度设置为:200mm/s 表示20ms周期计数34个脉冲左右;最大速度设置为:1400mm/s 表示20ms周期计数140个脉冲
 * 速度值作为参考，实际速度有偏差，低速下，电机扭矩偏小，容易堵转，速度步增值为10
 */

__IO uint32_t left_encoder_temp, right_encoder_temp; // 左电机和右电机的编码器脉冲计数值 - 临时值
//
uint16_t L_Motor_PWM, R_Motor_PWM;
uint16_t L_Motor_ENC, R_Motor_ENC;
//左右电机的PID参数结构体定义
TYPEDEF_PID_STRUCT gL_MotorPID;	
TYPEDEF_PID_STRUCT gR_MotorPID;	
//PID参数初始化
void PID_init(void)
{	
	//左电机
	gL_MotorPID.SetSpeed = 0.0;
	gL_MotorPID.ActualSpeed = 0.0;
	gL_MotorPID.err = 0.0;
	gL_MotorPID.err_next = 0.0;
	gL_MotorPID.err_last = 0.0;
	gL_MotorPID.Kp = 1.0;
	gL_MotorPID.Ki = 1.0;
	gL_MotorPID.Kd = 0.0;
	//右电机
	gR_MotorPID.SetSpeed = 0.0;
	gR_MotorPID.ActualSpeed = 0.0;
	gR_MotorPID.err = 0.0;
	gR_MotorPID.err_next = 0.0;
	gR_MotorPID.err_last = 0.0;
	gR_MotorPID.Kp = 1.0;
	gR_MotorPID.Ki = 1.0;
	gR_MotorPID.Kd = 0.0;
}
//增量式PID控制算法
float PID_Control_Algorithm(float TargetSpeed, float ActualSpeed, TYPEDEF_PID_STRUCT *PID)
{
	float incrementSpeed;
	//设置速度
	PID->SetSpeed = TargetSpeed;
	PID->ActualSpeed = ActualSpeed;
	//计算误差
	PID->err = PID->SetSpeed - PID->ActualSpeed ;
	//算法实现
	incrementSpeed = PID->Kp * (PID->err - PID->err_next) + PID->Ki * PID->err + PID->Kd * (PID->err - 2 * PID->err_next + PID->err_last);
	PID->ActualSpeed += incrementSpeed;
	//保存误差
	PID->err_last = PID->err_next ;
	PID->err_next = PID->err ;
	//PWM输出设置
	if(PID->ActualSpeed < 0) PID->ActualSpeed = 0;
	if(PID->ActualSpeed > 1000) PID->ActualSpeed = 1000;
	//返回数据
	return PID->ActualSpeed;
}
/*
 * 定时器的输入捕获上升沿触发的中断回调函数
 *
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* 左电机编码器 */
  if ((htim->Instance == TIM1) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
  {
    left_encoder_temp++;
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  }

  /* 右电机编码器 */
  if ((htim->Instance == TIM14) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
  {
    right_encoder_temp++;
    HAL_TIM_IC_Start_IT(&htim14, TIM_CHANNEL_1);
  }
}
/**
 * @brief  直流电机编码器的初始化
 * @note
 * @retval None
 */
void motor_encoder_init(void)
{
  /* 清零计数器 */
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_COUNTER(&htim14, 0);

    /* 使能输入捕获中断 */
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim14, TIM_CHANNEL_1);
}
/**
 * @brief  读取左轮编码器
 * @note
 * @retval
 */
uint32_t motor_read_encoder(te_L_R_Motor_t motor)
{
  uint32_t temp;
  switch (motor)
  {
  case L_MOTOR:
    temp = left_encoder_temp;
    left_encoder_temp = 0;
    break;
  case R_MOTOR:
    temp = right_encoder_temp;
    right_encoder_temp = 0;
    break;

  default:
    temp = 0;
    break;
  }

  return temp;
}

/**
 * @brief  解析JSON数据包
 * @note
 * @param  *pstr:
 * @retval None
 */
void parse_json_data(uint8_t *pstr, uint16_t len)
{
  if (pstr == NULL || len < 2)
    return;

  cJSON *json_root = cJSON_Parse((const char *)pstr);
  if (json_root)
  {
    cJSON *json_control = cJSON_GetObjectItem(json_root, "control");
    if (json_control)
    {
      cJSON *json_turn = cJSON_GetObjectItem(json_control, "turn");
      if (json_turn)
      {
        if (!strcmp(json_turn->valuestring, "run"))
        {
          console_log_success("control turn %s", json_turn->valuestring);
          systemValue.car_status = CAR_STATUS_RUN;
        }
        else if (!strcmp(json_turn->valuestring, "back"))
        {
          console_log_success("control turn %s", json_turn->valuestring);
          systemValue.car_status = CAR_STATUS_BACK;
        }
        else if (!strcmp(json_turn->valuestring, "left"))
        {
          console_log_success("control turn %s", json_turn->valuestring);
          systemValue.car_status = CAR_STATUS_LEFT;
        }
        else if (!strcmp(json_turn->valuestring, "right"))
        {
          console_log_success("control turn %s", json_turn->valuestring);
          systemValue.car_status = CAR_STATUS_RIGHT;
        }
				else if (!strcmp(json_turn->valuestring, "stop"))
        {
          console_log_success("control turn %s", json_turn->valuestring);
          systemValue.car_status = CAR_STATUS_STOP;
        }
      }
      json_turn = NULL;

      cJSON *json_power = cJSON_GetObjectItem(json_control, "power");
      if (json_power)
      {
        if (!strcmp(json_power->valuestring, "on"))
        {
          console_log_success("control power %s", json_power->valuestring);
          systemValue.car_status = CAR_STATUS_ON;
        }
        else if (!strcmp(json_power->valuestring, "off"))
        {
          console_log_success("control power %s", json_power->valuestring);
          systemValue.car_status = CAR_STATUS_OFF;
        }
      }
      json_power = NULL;

      cJSON *json_pwm = cJSON_GetObjectItem(json_control, "pwm");
      if (json_pwm)
      {
        cJSON *json_L_pwm = cJSON_GetObjectItem(json_pwm, "L_Motor");
        cJSON *json_R_pwm = cJSON_GetObjectItem(json_pwm, "R_Motor");
				//保存设置的目标速度，低速下发值：40，中速：70  高速100，Hi3861端下发的PWM值增加了死区时间
				if((json_L_pwm->valueint) == 390)	//低速350
				{
					gL_MotorPID.SetSpeed = (json_L_pwm->valueint - 355) * 10;	//占空比范围：300-1000，电机死区为300左右
					gR_MotorPID.SetSpeed = (json_R_pwm->valueint - 355) * 10;	//占空比范围：300-1000，电机死区为300左右
					console_log_success("control pwm_L: %d, pwm_R: %d", L_Motor_PWM, R_Motor_PWM);
				}
				else if((json_L_pwm->valueint) == 420)	//中速500
				{
					gL_MotorPID.SetSpeed = (json_L_pwm->valueint - 370) * 10;	//占空比范围：300-1000，电机死区为300左右
					gR_MotorPID.SetSpeed = (json_R_pwm->valueint - 370) * 10;	//占空比范围：300-1000，电机死区为300左右
					console_log_success("control pwm_L: %d, pwm_R: %d", L_Motor_PWM, R_Motor_PWM);
				}
				else if((json_L_pwm->valueint) == 450)	//高速600
				{
					gL_MotorPID.SetSpeed = (json_L_pwm->valueint - 390) * 10;	//占空比范围：300-1000，电机死区为300左右
					gR_MotorPID.SetSpeed = (json_R_pwm->valueint - 390) * 10;	//占空比范围：300-1000，电机死区为300左右
					console_log_success("control pwm_L: %d, pwm_R: %d", L_Motor_PWM, R_Motor_PWM);
				}
      }
      json_pwm = NULL;
			
    }
    json_control = NULL;
  }
  cJSON_Delete(json_root);
  json_root = NULL;
}




/**
 * @brief  生成JSON数据
 * @note   注意：生成JSON数据完成之后，一定要将packet_buff进行清空处理
 * @param  *sysData:
 * @retval
 */
void packet_json_data(ts_SystemValue_t *sysData)
{
  uint8_t packet_buff[125] = {0};
	//ActualSpeed为实际速度，* 0.968是因微信小程序将 速度值 * 1000 / 1968 * 0.2
  sprintf((char *)packet_buff, "{\"status\":{\"distance\":%d,\"carPower\":%d,\"L_speed\":%d,\"R_speed\":%d}}",
          sysData->distance, sysData->battery, (uint16_t)(gL_MotorPID.ActualSpeed * 0.968), (uint16_t)(gL_MotorPID.ActualSpeed * 0.968));

  HAL_UART_Transmit(&huart1, packet_buff, strlen((char *)packet_buff), 10); // 将数据发送出去
//	console_log_info("buff: %s", packet_buff);
}

// 10ms产生一个中断
volatile uint16_t TIM3_Times_Count = 0;
//
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) 
  {
		TIM3_Times_Count++;
		if(!(TIM3_Times_Count % 2))
		{
			TIM3_Times_Count = 0;
			//读取编码器值
			systemValue.L_enc = motor_read_encoder(L_MOTOR);
			systemValue.R_enc = motor_read_encoder(R_MOTOR);
			//设置输出PWM
			motor_control_running_status(systemValue.car_status);
		}
  }
}
//
volatile uint8_t gMotorStatus = 0;	//PWM输出是否开启
extern volatile uint8_t gLineOut;	//光电管输出, 8位二值化: bit0=最左, bit7=最右, 0=检测到黑线, 1=白
extern volatile uint8_t gLineType;	//巡线的类型(高4位为0xF时表示进入巡线模式)

//巡线速度定义(针对E题: 1m*1m正方形直角赛道, 线宽1.8cm, 逆时针行驶)
//注意: 本车为7路光电管, 8位数据中有一位恒为1(无效), 加权位置算法天然兼容, 无需关心无效位在哪一侧
#define SPEED_BASE        350    //直行基础速度(用户实测750直线稳定, 先按550调试)
#define SPEED_TURN_MID    200    //比例差速下限: 偏差较大时内侧轮的最低速度(防止小角度弯降得太低拖慢响应)
#define SPEED_TURN_HARD   120    //强偏差: 内侧轮速度(外侧提速), 大差速快转, 直接覆写PWM立即生效
#define SPEED_CORNER      280    //入弯降速: |dev|较大(线接近阵列边缘/接近角点)时两轮基础速度, 物理上减小冲出距离(替代手动降速)
#define SPEED_PIVOT       420    //直角弯/丢线搜索: 原地旋转外侧轮速度(内侧轮刹车)
#define PIVOT_TIMEOUT     200    //原地旋转超时: 200*20ms=4秒
#define LINE_DEV_DEAD     1.0f   //居中死区: |dev|小于此值不纠正, 防止小偏差过度修正引发钟摆(兼容7路/8路布局的±0.5中心偏移)
#define LINE_DEV_GAIN     75.0f  //比例差速增益: 偏差越大修正越强(配合微分项后适当调低, 单独过大会钟摆)
#define LINE_DEV_KD       65.0f  //微分阻尼增益: 对dev变化率做PD控制, 回中途中自动收力防过冲, 并补偿模组帧率低带来的滞后(等效预判)

//循迹状态机
typedef enum
{
	TRACK_NORMAL = 0,	//正常循线(位置式差速)
	TRACK_PIVOT,		//直角弯/丢线: 原地旋转找线状态(带丢线记忆)
} te_track_state_t;

//直角弯处理与丢线记忆变量(进入循线模式时统一复位)
static te_track_state_t gTrackState = TRACK_NORMAL;
static int8_t gLastLineDir = 0;      //丢线记忆: 最近一次线的方向 -1=线偏左, 0=居中, 1=线偏右, -99=未建立记忆(默认逆时针左搜)
static int8_t gPivotDir = 0;         //原地旋转方向: 0=不旋转, -1=左转, 1=右转
static uint16_t gPivotTimer = 0;     //原地旋转超时计数器(20ms为单位)
static uint8_t gEdgeTrigCnt = 0;     //边缘跳跃触发去抖计数(连续2个周期确认才原地旋转, 防单帧干扰误触发)
static float gLastDev = 0.0f;        //上一周期加权偏差(用于微分阻尼与回中收力)

//原地旋转辅助函数: 左转(左轮刹车, 右轮前进)
static void Motor_pivot_left(uint16_t pwm)
{
    L_MOTOR_IN0(GPIO_PIN_RESET);
    L_MOTOR_IN1(GPIO_PIN_RESET);
    L_MOTOR_PWM_OUT = 0;
    R_MOTOR_IN0(GPIO_PIN_SET);
    R_MOTOR_IN1(GPIO_PIN_RESET);
    R_MOTOR_PWM_OUT = pwm;
}

//原地旋转辅助函数: 右转(右轮刹车, 左轮前进)
static void Motor_pivot_right(uint16_t pwm)
{
    R_MOTOR_IN0(GPIO_PIN_RESET);
    R_MOTOR_IN1(GPIO_PIN_RESET);
    R_MOTOR_PWM_OUT = 0;
    L_MOTOR_IN0(GPIO_PIN_SET);
    L_MOTOR_IN1(GPIO_PIN_RESET);
    L_MOTOR_PWM_OUT = pwm;
}
//
void motor_control_running_status(te_car_status_t status)
{
	L_Motor_ENC = systemValue.L_enc;
	R_Motor_ENC = systemValue.R_enc;
	//通过码盘输出，计算输出的PWM值，关联关系：编码器值*7后与PWM值进行关联，速度步增值为10
	L_Motor_PWM = (uint16_t) PID_Control_Algorithm(gL_MotorPID.SetSpeed, L_Motor_ENC * 7, &gL_MotorPID);
	R_Motor_PWM = (uint16_t) PID_Control_Algorithm(gR_MotorPID.SetSpeed, R_Motor_ENC * 7, &gR_MotorPID);
	//设置输出
  switch (status)
  {
    case CAR_STATUS_RUN:	//前进
      Motor_turn_run(L_Motor_PWM, R_Motor_PWM);
    break;

    case CAR_STATUS_BACK:	//后退
      Motor_turn_back(L_Motor_PWM, R_Motor_PWM);
    break;

    case CAR_STATUS_LEFT:	//左转
			Motor_turn_left(350, 350);
    break;

    case CAR_STATUS_RIGHT:	//右转
			Motor_turn_right(350, 350);
    break;

    case CAR_STATUS_ON:
			Motor_turn_on();	//开启PWM输出
    break;

    case CAR_STATUS_OFF:
			//速度值归零
			gL_MotorPID.ActualSpeed = 0.0;
			gR_MotorPID.ActualSpeed = 0.0;		
		  //小车在CAR_STATUS_OFF状态与gLineType高四位全为高的情况下进入巡线模式
			if((gLineType >> 4) == 0x0F)	//巡线模式
			{
				Car_search_black_Line();
			}
			else	//非巡线模式
			{
				gMotorStatus = 0x00;	//巡线模式下，gMotorStatus用于初始化PWM输出
				Motor_turn_off();	//关闭PWM输出
			}
    break;
	
	  case CAR_STATUS_STOP:	//PWM输出为零
			//速度值归零
			gL_MotorPID.ActualSpeed = 0.0;
			gR_MotorPID.ActualSpeed = 0.0;
			//
			Motor_turn_stop();
		break;
		
    default:
    break;
  }
}
//循线模式 - 面向E题1m*1m纯直角赛道 | 8传感器二值化编码: bit0=最左, bit7=最右, 0=检测到黑线, 1=白
//策略: 位置式加权差速循线 + 直角弯原地旋转 + 丢线记忆搜索(丢线后朝最后一次看到线的方向旋转找回)
void Car_search_black_Line(void)
{
	uint8_t lo = gLineOut;
	uint8_t leftActive = 0, rightActive = 0, totalActive = 0;
	uint8_t i;
	int16_t weightSum = 0;
	float dev = 0.0f;	//线位置偏差: 负=线偏左(需左转), 正=线偏右(需右转)
	
	//进入循线模式，先开启PWM输出，并复位循迹状态机与PID积分，防止上次残留导致飞车或抖动
	if(gMotorStatus == 0x00)
	{
		gMotorStatus = 0xFF;
		gTrackState = TRACK_NORMAL;
		gPivotDir = 0;
		gPivotTimer = 0;
		gLastLineDir = -99;	//未建立记忆, 丢线默认按逆时针方向(左)搜索
		gEdgeTrigCnt = 0;		//边缘跳跃去抖计数清零
		gLastDev = 0.0f;		//微分历史清零
		gL_MotorPID.ActualSpeed = 0.0f;
		gR_MotorPID.ActualSpeed = 0.0f;
		gL_MotorPID.err = gL_MotorPID.err_next = gL_MotorPID.err_last = 0.0f;
		gR_MotorPID.err = gR_MotorPID.err_next = gR_MotorPID.err_last = 0.0f;
		Motor_turn_on();
	}
	
	//统计左右两侧活跃传感器数量与加权位置 (0=检测到黑线)
	for(i = 0; i < 8; i++)
	{
		if(!(lo & (1 << i)))
		{
			totalActive++;
			weightSum += i;
			if(i < 4) leftActive++;
			else      rightActive++;
		}
	}
	if(totalActive > 0)
	{
		dev = (float)weightSum / (float)totalActive - 3.5f;	//传感器阵列中心为3.5
	}
	
	// ===== 原地旋转状态(直角弯过弯 / 丢线搜索) =====
	if(gTrackState == TRACK_PIVOT)
	{
		gPivotTimer++;
		//超时保护: 停车并退回正常状态, 下个周期若仍丢线会自动重新进入搜索(计时清零重试)
		if(gPivotTimer > PIVOT_TIMEOUT)
		{
			gTrackState = TRACK_NORMAL;
			gPivotDir = 0;
			Motor_turn_stop();
			return;
		}
		//防过转保护: 线已整体出现在旋转方向的相反侧, 说明转过了头, 立即反向旋转找回(并重新计时)
		if(lo != 0x00)
		{
			if((gPivotDir < 0) && (leftActive == 0) && (rightActive >= 2))
			{
				gPivotDir = 1;
				gPivotTimer = 0;
			}
			else if((gPivotDir > 0) && (rightActive == 0) && (leftActive >= 2))
			{
				gPivotDir = -1;
				gPivotTimer = 0;
			}
		}
		//退出条件: 线重新回到阵列中部(中左/中右传感器检测到线, 且两侧激活都不多)
		//直角弯过弯过程中线会先出现在外侧, 必须继续旋转到线回到中央才算对齐, 避免提前退出导致出弯跑偏;
		//全黑(lo==0x00)是压过角点的过渡状态, 不可作为退出依据, 继续旋转
		if((lo != 0x00) && ((lo & 0x18) != 0) && (leftActive <= 2) && (rightActive <= 2))
		{
			gTrackState = TRACK_NORMAL;
			gPivotDir = 0;
			gLastDev = dev;		//微分历史同步到当前偏差, 避免旋转期间积累的陈旧值在出弯瞬间造成虚假的大变化率
			//清除PID积分残留, 出弯瞬间重新建立速度闭环, 避免飞车或抖动; 随后直接走下面的正常循线逻辑出弯提速
			gL_MotorPID.SetSpeed = 0.0f;
			gR_MotorPID.SetSpeed = 0.0f;
			gL_MotorPID.ActualSpeed = 0.0f;
			gR_MotorPID.ActualSpeed = 0.0f;
			gL_MotorPID.err = gL_MotorPID.err_next = gL_MotorPID.err_last = 0.0f;
			gR_MotorPID.err = gR_MotorPID.err_next = gR_MotorPID.err_last = 0.0f;
		}
		else
		{
			//旋转期间目标速度清零, 防止增量式PID继续积分; PWM由旋转函数直接写入寄存器(不走PID)
			gL_MotorPID.SetSpeed = 0.0f;
			gR_MotorPID.SetSpeed = 0.0f;
			if(gPivotDir > 0)
			{
				Motor_pivot_right(SPEED_PIVOT);	//右转: 右轮刹车, 左轮前进(原地左前右停 -> 逆时针)
			}
			else
			{
				Motor_pivot_left(SPEED_PIVOT);	//左转: 左轮刹车, 右轮前进
			}
			return;
		}
	}
	
	// ===== 丢线(全白0xFF): 按丢线记忆方向原地旋转找线; 无记忆时逆时针赛道默认向左搜索 =====
	if(lo == 0xFF)
	{
		gTrackState = TRACK_PIVOT;
		gPivotTimer = 0;
		gPivotDir = (gLastLineDir == 1) ? 1 : -1;
		gL_MotorPID.SetSpeed = 0.0f;
		gR_MotorPID.SetSpeed = 0.0f;
		if(gPivotDir > 0)
		{
			Motor_pivot_right(SPEED_PIVOT);
		}
		else
		{
			Motor_pivot_left(SPEED_PIVOT);
		}
		return;
	}
	
	// ===== 直角弯触发: 3+1+3分区判定(左3路/中1路/右3路) =====
	//角点特征: 线从中间"跳"到最外侧 -> 边缘亮且中间空; 直线漂移是渐变的 -> 边缘亮时中间仍亮, 不会误触发(留给比例差速大幅纠正)
	{
		uint8_t edgeLeft  = (uint8_t)((lo & 0x03) != 0);  //最左外侧区: bit0,bit1(左3路中的最外两路)
		uint8_t edgeRight = (uint8_t)((lo & 0x60) != 0);  //最右外侧区: bit5,bit6
		uint8_t centerOn  = (uint8_t)((lo & 0x1C) != 0);  //中间区: bit2,bit3,bit4
		
		if((edgeLeft != 0) && (centerOn == 0))		//线突然跳到最左且中间全空 -> 典型左直角弯 -> 原地左转90°
		{
			if(++gEdgeTrigCnt >= 2)					//连续2个周期(40ms)确认, 滤除单帧噪声干扰
			{
				gEdgeTrigCnt = 0;
				gTrackState = TRACK_PIVOT;
				gPivotDir = -1;
				gPivotTimer = 0;
				gLastLineDir = -1;
				gL_MotorPID.SetSpeed = 0.0f;
				gR_MotorPID.SetSpeed = 0.0f;
				Motor_pivot_left(SPEED_PIVOT);
				return;
			}
		}
		else if((edgeRight != 0) && (centerOn == 0))	//线突然跳到最右且中间全空 -> 典型右直角弯 -> 原地右转90°
		{
			if(++gEdgeTrigCnt >= 2)
			{
				gEdgeTrigCnt = 0;
				gTrackState = TRACK_PIVOT;
				gPivotDir = 1;
				gPivotTimer = 0;
				gLastLineDir = 1;
				gL_MotorPID.SetSpeed = 0.0f;
				gR_MotorPID.SetSpeed = 0.0f;
				Motor_pivot_right(SPEED_PIVOT);
				return;
			}
		}
		else
		{
			gEdgeTrigCnt = 0;		//渐变/居中/双侧有线: 不构成跳跃, 清零去抖计数, 交给下方比例差速处理(小幅度纠正)
		}
	}
	
	// ===== 宽线状态(≥6路同时压线): 压过起点交叉点或角点过渡区, 保持直行等待传感器图案变化 =====
	// (7路传感器永远不会出现全黑0x00, 故按激活数量判断而非固定图案)
	if(totalActive >= 6)
	{
		gL_MotorPID.SetSpeed = SPEED_BASE;
		gR_MotorPID.SetSpeed = SPEED_BASE;
		Motor_turn_run(L_Motor_PWM, R_Motor_PWM);
		return;
	}
	
	// ===== 正常循线: 居中死区 + PD比例差速(小偏差小修正, 大偏差大修正; 微分项回中收力防钟摆并补偿帧率滞后) =====
	if(dev < 0) gLastLineDir = -1;	//丢线记忆: 只记录方向, 供丢线和直角弯后搜索方向使用
	else if(dev > 0) gLastLineDir = 1;
	
	{
		float absDev = (dev >= 0.0f) ? dev : -dev;
		float devRate = dev - gLastDev;		//偏差变化率: >0=线正向外侧移动(需加力), <0=正回中(应收力, 防过冲钟摆)
		gLastDev = dev;
		
		if(absDev < LINE_DEV_DEAD)			//居中死区: 双轮同速直行(编码器+PID保证直线性), 不施加任何差速纠正
		{
			gL_MotorPID.SetSpeed = SPEED_BASE;
			gR_MotorPID.SetSpeed = SPEED_BASE;
		}
		else if(absDev >= 2.5f)				//线到最外侧传感器(接近角点前的最后修正): 大差速, 直接覆写PWM立即生效, 同时降速防冲出
		{
			if(dev < 0)	//线在最左, 强力左转(直接覆写立即生效)
			{
				L_Motor_PWM = SPEED_TURN_HARD;
				gL_MotorPID.SetSpeed = SPEED_TURN_HARD;
				gR_MotorPID.SetSpeed = SPEED_CORNER + 60;
			}
			else			//线在最右, 强力右转(直接覆写立即生效)
			{
				gL_MotorPID.SetSpeed = SPEED_CORNER + 60;
				R_Motor_PWM = SPEED_TURN_HARD;
				gR_MotorPID.SetSpeed = SPEED_TURN_HARD;
			}
		}
		else								//中间区域: PD差速, 内侧轮按"偏差+变化率"减速, 外侧轮保持基础速度(平滑无台阶, 防钟摆)
		{
			//入弯降速: 线接近阵列边缘(|dev|>=2.0, 通常预示即将进角点)时两轮整体降到SPEED_CORNER, 从物理上减小冲出距离与反应压力(替代手动降速)
			uint16_t outerSpeed = (absDev >= 2.0f) ? SPEED_CORNER : SPEED_BASE;
			//PD合成修正量: P项按偏差给修正, D项按变化率预判/阻尼(回中时自动收力, 滞后时提前加力)
			float corr = LINE_DEV_GAIN * absDev + LINE_DEV_KD * ((dev >= 0.0f) ? devRate : -devRate);
			if(corr < 0.0f) corr = 0.0f;				//D项收力占主导(正快速回中): 不再施加修正, 避免过冲反向钟摆
			if(corr > (float)(outerSpeed - SPEED_TURN_MID)) corr = (float)(outerSpeed - SPEED_TURN_MID);
			uint16_t innerSpeed = outerSpeed - (uint16_t)corr;
			if(dev < 0)	//线偏左, 左轮减速左转(偏得越多/偏得越快左轮越慢)
			{
				gL_MotorPID.SetSpeed = innerSpeed;
				gR_MotorPID.SetSpeed = outerSpeed;
			}
			else			//线偏右, 右轮减速右转(偏得越多/偏得越快右轮越慢)
			{
				gL_MotorPID.SetSpeed = outerSpeed;
				gR_MotorPID.SetSpeed = innerSpeed;
			}
		}
	}
	
	//前方障碍保护(超声波距离<=150mm时停车)
	if(systemValue.distance <= 150)
	{
		Motor_turn_stop();
		gL_MotorPID.SetSpeed = 0;
		gR_MotorPID.SetSpeed = 0;
	}
	else
	{
		Motor_turn_run(L_Motor_PWM, R_Motor_PWM);
	}
}
//

/**
 * @brief  停止
 * @note
 * @retval None
 */
void Motor_turn_stop(void)
{
  L_MOTOR_IN0(GPIO_PIN_RESET);
  L_MOTOR_IN1(GPIO_PIN_RESET);
  R_MOTOR_IN0(GPIO_PIN_RESET);
  R_MOTOR_IN1(GPIO_PIN_RESET);
  L_MOTOR_PWM_OUT = R_MOTOR_PWM_OUT = 0;
  L_Motor_PWM = R_Motor_PWM = 0;
	systemValue.L_enc = systemValue.R_enc = 0;
}

/**
 * @brief  左转
 * @note
 * @param  l_pwm:
 * @param  r_pwm:
 * @retval None
 */
void Motor_turn_left(uint16_t l_pwm, uint16_t r_pwm)
{
  L_MOTOR_IN0(GPIO_PIN_SET);
  L_MOTOR_IN1(GPIO_PIN_RESET);
  R_MOTOR_IN0(GPIO_PIN_RESET);
  R_MOTOR_IN1(GPIO_PIN_SET);

  L_MOTOR_PWM_OUT = l_pwm;
  R_MOTOR_PWM_OUT = r_pwm;
}
/**
 * @brief  右转
 * @note
 * @param  l_pwm:
 * @param  r_pwm:
 * @retval None
 */
void Motor_turn_right(uint16_t l_pwm, uint16_t r_pwm)
{
  L_MOTOR_IN0(GPIO_PIN_RESET);
  L_MOTOR_IN1(GPIO_PIN_SET);
  R_MOTOR_IN0(GPIO_PIN_SET);
  R_MOTOR_IN1(GPIO_PIN_RESET);

  L_MOTOR_PWM_OUT = l_pwm;
  R_MOTOR_PWM_OUT = r_pwm;
}
/**
 * @brief  前进
 * @note
 * @param  l_pwm:
 * @param  r_pwm:
 * @retval None
 */
void Motor_turn_run(uint16_t l_pwm, uint16_t r_pwm)
{
  L_MOTOR_IN0(GPIO_PIN_SET);
  L_MOTOR_IN1(GPIO_PIN_RESET);
  R_MOTOR_IN0(GPIO_PIN_SET);
  R_MOTOR_IN1(GPIO_PIN_RESET);

  L_MOTOR_PWM_OUT = l_pwm;
  R_MOTOR_PWM_OUT = r_pwm;
}
/**
 * @brief  后退
 * @note
 * @param  l_pwm:
 * @param  r_pwm:
 * @retval None
 */
void Motor_turn_back(uint16_t l_pwm, uint16_t r_pwm)
{
  L_MOTOR_IN0(GPIO_PIN_RESET);
  L_MOTOR_IN1(GPIO_PIN_SET);
  R_MOTOR_IN0(GPIO_PIN_RESET);
  R_MOTOR_IN1(GPIO_PIN_SET);

  L_MOTOR_PWM_OUT = l_pwm;
  R_MOTOR_PWM_OUT = r_pwm;
}
/**
 * @brief  开启电机
 * @note
 * @retval None
 */
void Motor_turn_on(void)
{
  L_MOTOR_IN0(GPIO_PIN_SET);
  L_MOTOR_IN1(GPIO_PIN_SET);
  R_MOTOR_IN0(GPIO_PIN_SET);
  R_MOTOR_IN1(GPIO_PIN_SET);
  L_MOTOR_PWM_OUT = 0;
  R_MOTOR_PWM_OUT = 0;
	L_Motor_PWM = R_Motor_PWM = 0;
//	systemValue.L_enc = systemValue.R_enc = 0;
  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1); // 电机1
  HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1); // 电机2
}
/**
 * @brief  关闭电机
 * @note
 * @retval None
 */
void Motor_turn_off(void)
{
  L_MOTOR_IN0(GPIO_PIN_RESET);
  L_MOTOR_IN1(GPIO_PIN_RESET);
  R_MOTOR_IN0(GPIO_PIN_RESET);
  R_MOTOR_IN1(GPIO_PIN_RESET);
  L_MOTOR_PWM_OUT = R_MOTOR_PWM_OUT = 0;
  L_Motor_PWM = R_Motor_PWM = 0;
//	systemValue.L_enc = systemValue.R_enc = 0;
  HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1); // 电机1
  HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1); // 电机2
}



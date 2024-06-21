#include "headfile.h"

//定义按键引脚
#define KEY1_PIN    P70
#define KEY2_PIN    P71
#define KEY3_PIN    P72
#define KEY4_PIN    P73
//定义拨码开关引脚
#define SW1_PIN     P75
#define SW2_PIN     P76

#define	Hall				P43
#define	Hall1				P26
#define	Mot_A	P62
#define	Mot_B	P65
//拨码开关状态变量
uint8 sw1_status;
uint8 sw2_status;

//开关状态变量
uint8 key1_status = 1;
uint8 key2_status = 1;
uint8 key3_status = 1;
uint8 key4_status = 1;

//上一次开关状态变量
uint8 key1_last_status;
uint8 key2_last_status;
uint8 key3_last_status;
uint8 key4_last_status;

//开关标志位
uint8 key1_flag;
uint8 key2_flag;
uint8 key3_flag;
uint8 key4_flag;

uint8 test1=0,test2=0,test3=0,test4=0;

/*.....舵机参数.....*/
uint16 	steering=910;	
int 		L_max_steering=1020;
int			R_max_steering=810;
int16		xunhuan_flag=0;
int16 	L,M,R,R1,K,J;
int16 	My_Direction_NowError;
int16		My_Direction_lastError;
int16		My_Direction_addError;
int16		My_Direction_SumError;
float		My_Direction_KP=3.5; 
float		My_Direction_KD=2.4;
/*..........*/

int i_add_timer=0;


/*******timer***********/

int timer_2=0;
int timer_4S=0;
int timer_4A=0;
int timer_4huan1=0;
int timer_4huan2=0;
int timer_4huan3=0;
/**********************/


/************ 元素识别 **************/
//环岛
uint16	hd_flag_cy=0;
uint16 	hd_flag_yu1;
uint16	hd_flag_yu2;
uint16 	hd_flag_go;
uint16	hd_flag_y_cy=0;

uint16	hd_flag_a=0;
uint16	hd_flag_b=0;



//右环岛
uint16	hd_flag_cy_r=0;
uint16 	hd_flag_yu_r;
uint16 	hd_flag_go_r;

int16 hd_L_max_y=530;
int16 hd_L_min_y=430;

int16 hd_M_max_y=1000;
int16 hd_M_min_y=850;

int16 hd_R_max_y=650;
int16 hd_R_min_y=500;

int16 hd_K_max_y=930;
int16 hd_K_min_y=600;		

int16	hd_J_max_y=1000;
int16	hd_J_min_y=880;
		/*环岛阈值*/
int16 hd_L_max=900;
int16 hd_L_min=400;

int16 hd_M_max=720;
int16 hd_M_min=390;

int16 hd_R_max=700;
int16 hd_R_min=200;

int16 hd_K_max=1000;
int16 hd_K_min=600;		

int16	hd_J_max=1000;
int16	hd_J_min=230;

//三岔
uint16	sc_y_flag_cy;		//预处理采样flag
uint16	sc_flag_cy;			//采样flag
uint16	sc_flag_r;		
uint16	sc_flag_l;
uint16	sc_flag_yu=0;

  /*预处理阈值*/
int16 	sc_y_L_max=220;
int16 	sc_y_L_min=130;

int16 	sc_y_M_max=330;
int16 	sc_y_M_min=50;

int16 	sc_y_R_max=300;
int16 	sc_y_R_min=150;

int16 	sc_y_K_max=450;
int16 	sc_y_K_min=230;

int16		sc_y_J_max=700;
int16		sc_y_J_min=330;

  /*进三岔阈值*/
int16 	sc_L_max=200;
int16 	sc_L_min=80;

int16 	sc_M_max=150;
int16 	sc_M_min=60;

int16 	sc_R_max=210;
int16 	sc_R_min=100;

int16 	sc_K_max=350;
int16 	sc_K_min=180;	
//进三岔前偏右
int16 	sc_L_max_r=230;
int16 	sc_L_min_r=150;

int16 	sc_M_max_r=80;
int16 	sc_M_min_r=190;

int16 	sc_R_max_r=200;
int16 	sc_R_min_r=90;

int16 	sc_K_max_r=340;
int16 	sc_K_min_r=500;	
//出入库
int16	go_out=0;
int16	go_in=0;
/*...电机参数..*/
#define DIR1 P35
#define DIR2 P53
#define BEEP P77
uint8 Statu=2;
int16 speed1 = 0;
int16 speed2 = 0;
float Current_speed=0;
int16 Out_PWM=0; //输出占空比
float error,error_pre=0;
int I_add=0;  //积分
int16 dutyL=0;//左电机占空比
int16 dutyR=0;//右电机占空比
int zheng_zhuan_flag=0;
int fan_zhuan_flag=0;
int16 duty=2000;
int16 duty_x=2600;
float Motor_P=5;
float Motor_I=0.1;
extern int16 speed=450;
extern int8 RUN;
extern uint8 Dir;
int  ting_flag=0;
/*...............*/


int My_Direction_addError_i=0;
int shizi_i=0;
int L_huandao_i=0;
int R_huandao_i=0;
int16  My_Direction_Pre1_Error[4];
int16 My_Direction_Direct_Parameter;
uint16 Price_PWM;

void DJ_output()
{
    // 使用假定的速度值，或者从其他传感器获取速度值
    int fake_speed1 = 100;  // 假设速度1为100
    int fake_speed2 = 100;  // 假设速度2为100

    // 不再需要读取编码器方向和计数
    // speed1 和 speed2 直接赋值为假定的速度值
    speed1 = fake_speed1;
    speed2 = fake_speed2;

    // 假设的速度计算
    Current_speed = (speed1 + speed2) / 2;
    Current_speed = Current_speed * 20.4 / (2355.2 * 0.02);  // 速度计算公式可以根据需要调整

    // 电机PI控制
    error = (int)(speed - Current_speed);
    duty = duty + (error - error_pre) * Motor_P + error * Motor_I;
    error_pre = error;

    if (duty < 0)
    {
        duty = -duty;
        fan_zhuan_flag = 1;
    }
    else
    {
        fan_zhuan_flag = 0;
    }

    if (duty >= duty_x) duty = duty_x;

    if (My_Direction_NowError > 12)
    {
        pwm_duty(PWMA_CH1P_P60, duty * 0.7);     // 左轮负
        Mot_A = 0;
        pwm_duty(PWMA_CH3P_P64, duty * 1.1);     // 右轮负
        Mot_B = 0;
    }
    if (My_Direction_NowError < -12)
    {
        pwm_duty(PWMA_CH1P_P60, duty * 1.1);     // 左轮正
        Mot_A = 0;
        pwm_duty(PWMA_CH3P_P64, duty * 0.7);     // 右轮正
        Mot_B = 0;
    }
    else
    {
        speed = 200;
        if (fan_zhuan_flag == 0)
        {
            pwm_duty(PWMA_CH1P_P60, duty);  // 左轮正
            Mot_A = 0;
            pwm_duty(PWMA_CH3P_P64, duty);  // 右轮正
            Mot_B = 0;
        }
        else
        {
            pwm_duty(PWMA_CH1P_P60, duty);  // 左轮负
            Mot_A = 0;
            pwm_duty(PWMA_CH4P_P66, duty);  // 右轮正
            Mot_B = 0;
        }
    }
}


void Direction_Out()
{
    // 计算PD控制误差
    My_Direction_SumError = My_Direction_KP * My_Direction_NowError + My_Direction_KD * (My_Direction_NowError - My_Direction_lastError);
    My_Direction_lastError = My_Direction_NowError;

    // 误差滤波
    My_Direction_Pre1_Error[3] = My_Direction_Pre1_Error[2];
    My_Direction_Pre1_Error[2] = My_Direction_Pre1_Error[1];
    My_Direction_Pre1_Error[1] = My_Direction_Pre1_Error[0];
    My_Direction_Pre1_Error[0] = My_Direction_SumError;
    My_Direction_Direct_Parameter = My_Direction_Pre1_Error[0] * 0.8 + My_Direction_Pre1_Error[1] * 0.1 + My_Direction_Pre1_Error[2] * 0.06 + My_Direction_Pre1_Error[3] * 0.04;

    // 控制信号计算
    Price_PWM = (int16)(My_Direction_Direct_Parameter);

    // 假设左轮速度为base_speed - Price_PWM，右轮速度为base_speed + Price_PWM
    int base_speed = 200;
    int left_wheel_speed = base_speed - Price_PWM;
    int right_wheel_speed = base_speed + Price_PWM;

    // 确保速度在合理范围内
    if (left_wheel_speed < 0) left_wheel_speed = 0;
    if (left_wheel_speed > 255) left_wheel_speed = 255;
    if (right_wheel_speed < 0) right_wheel_speed = 0;
    if (right_wheel_speed > 255) right_wheel_speed = 255;

    // 通过PWM输出左右轮速度
    pwm_duty(PWMA_CH1P_P60, left_wheel_speed);   // 左轮
    pwm_duty(PWMA_CH3P_P64, right_wheel_speed);  // 右轮
}


void Lcd_show_message()
{
	lcd_showstr(0,0,"L:");
	lcd_showuint16(15,0,L);
	
	lcd_showstr(0,1,"M:");
	lcd_showuint16(15,1,M);
	
	lcd_showstr(0,2,"R:");
	lcd_showuint16(15,2,R);
	
	lcd_showstr(0,3,"K:");
	lcd_showuint16(15,3,K);
	
	lcd_showstr(0,4,"J:");
	lcd_showuint16(15,4,J);
//	oled_printf_float(15,4,My_Direction_KP,2,3);
	
	lcd_showstr(0,5,"T:");
	lcd_showuint16(15,5,timer_4S);
	
	lcd_showstr(0,6,"L:");
	lcd_showint16(15,6,sc_flag_l);
	
	lcd_showstr(0,7,"Y:");
	lcd_showuint16(15,7,hd_flag_yu1);
	
	lcd_showstr(0,8,"G:");
	lcd_showint16(15,8,hd_flag_go);
	
//	lcd_showstr(60,1,"LH:");
//	lcd_showint16(80,1,L_huandao_i);
//	lcd_showstr(60,2,"SA:");
//	lcd_showint16(80,2,sancha_flge);
//	lcd_showstr(60,3,"XF:");
//	lcd_showint16(80,3,xunhuan_flag);
}

void main()
{
	DisableGlobalIRQ();		
	delay_init();
	
    //sys_clk:30000000, 27000000. 24000000, 22118400, 20000000, 18432000, 12000000, 11059200, 6000000, 5529600
	sys_clk = 30000000;     
    
	board_init();			
	
	oled_init();
	//lcd_init();	
	//ctimer_count_init(CTIM0_P34);  //初始化2个编码器
	//ctimer_count_init(CTIM3_P04);  //初始化2个编码器
	adc_init(ADC_P00,ADC_SYSclk_DIV_2);
	adc_init(ADC_P01,ADC_SYSclk_DIV_2);
	adc_init(ADC_P05,ADC_SYSclk_DIV_2);
	adc_init(ADC_P06,ADC_SYSclk_DIV_2);
	
	
	//adc_init(ADC_P10,ADC_SYSclk_DIV_2);
	//adc_init(ADC_P11,ADC_SYSclk_DIV_2);
	//adc_init(ADC_P13,ADC_SYSclk_DIV_2);
	//adc_init(ADC_P14,ADC_SYSclk_DIV_2);
	
	pwm_init(PWMA_CH2P_P62, 5000, 0); 
	pwm_init(PWMA_CH4P_P66, 5000, 0); 

	//pwm_init(PWMB_CH1_P74, 100, 910); 
	gpio_mode(P4_3,GPIO);
	gpio_mode(P7_7,GPO_PP); 
	gpio_mode(P7_0,GPIO);
	gpio_mode(P7_1,GPIO);
	gpio_mode(P7_2,GPIO);
	gpio_mode(P7_3,GPIO);
	gpio_mode(P6_5,GPO_PP);
	gpio_mode(P6_2,GPO_PP);
	gpio_mode(P2_6,GPIO);
	BEEP=0;
	pwm_duty(PWMA_CH2P_P62, 1800);
	pwm_duty(PWMA_CH4P_P66, 1800);
	Mot_A=0;
	Mot_B=0;
	//pwm_duty(PWMB_CH1_P74, 910);
	delay_ms(255);
	delay_ms(155);
   // delay_ms(255);
	//pwm_duty(PWMB_CH1_P74, 810);
  delay_ms(255);
  delay_ms(255);
//  delay_ms(255);
//  delay_ms(255);
	pit_timer_ms(TIM_4,1);
//	pit_timer_ms(TIM_2,100);
	EnableGlobalIRQ();		

	
	
	while(1)
	{
   	Lcd_show_message();
		BEEP=0;
	
		
		//使用此方法优点在于，不需要使用while(1) 等待，避免处理器资源浪费
		//保存按键状态
//		if(KEY1_PIN==0)
//		{
//			delay_ms(50);
//			if(KEY1_PIN==0)
//			{
//				My_Direction_KD=My_Direction_KD+0.1;
//			}
//		}
//		if(KEY2_PIN==0)
//		{
//			delay_ms(50);
//			if(KEY2_PIN==0)
//			{
//				My_Direction_KD=My_Direction_KD-0.1;
//			}
//		}
//		if(KEY3_PIN==0)
//		{
//			delay_ms(50);
//			if(KEY3_PIN==0)
//			{
//				My_Direction_KP=My_Direction_KP+0.1;
//			}
//		}
//		if(KEY4_PIN==0)
//		{
//			delay_ms(50);
//			if(KEY4_PIN==0)
//			{
//				My_Direction_KP=My_Direction_KP-0.1;
//			}
//		}
	}
}
//void TM2_Isr() interrupt 12
//{
//	TIM2_CLEAR_FLAG;  //清除中断标志
//	timer_2++;
//	if(timer_2==3500)
//	{
//	  speed=300;
//		//timer_2=0;
//		BEEP=1;
//	}
//	if(timer_2==5500)
//	{
//	  speed=200;
//	}
//  if(timer_2==8000)
//	{
//	  speed=0;
//		
//	}
//}
void TM4_Isr() interrupt 20
{
	// 清楚中断标志
	TIM4_CLEAR_FLAG; 
	R=adc_once(ADC_P00, ADC_10BIT);
	L=adc_once(ADC_P06, ADC_10BIT);
	M=adc_once(ADC_P00, ADC_10BIT);
	K=adc_once(ADC_P05, ADC_10BIT);
	J=adc_once(ADC_P13, ADC_10BIT);
//	R1=adc_once(ADC_P06, ADC_10BIT);

	My_Direction_NowError=50*(R-L)/(L+M+R);
	if(R>L)
	{
		if(My_Direction_NowError<0)
		{
			My_Direction_NowError=-My_Direction_NowError;
		}
	}
	if(R<L)
	{
		if(My_Direction_NowError>0)
		{
			My_Direction_NowError=-My_Direction_NowError;
		}
	}
	
/***************元素处理***************/
/*环岛*/
	if((L>hd_L_min_y)&&(L<hd_L_max_y)&&(R>hd_R_min_y)&&(R<hd_R_max_y)&&(M>hd_M_min_y)&&(M<hd_M_max_y)&&(K>hd_K_min_y)&&(K<hd_K_max_y)&&(J>hd_J_min_y)&&(J<hd_J_max_y)&&(hd_flag_yu1==0))
	{
		if(hd_flag_y_cy==0)
		{
			hd_L_max_y=L+80;
			hd_L_min_y=L-80;
			hd_M_max_y=M+80;
			hd_M_min_y=M-80;
			hd_R_max_y=R+80;
			hd_R_min_y=R-80;
			hd_K_max_y=K+100;
			hd_K_min_y=K-100;	
			hd_J_max_y=J+100;
			hd_J_min_y=J-100;
			hd_flag_y_cy=1;
		}
		hd_flag_yu1=1;
		speed=200;
		duty_x=2100;
		DJ_output();
		
	}
	//進左環島
	if((L>hd_L_min)&&(L<hd_L_max)&&(M>hd_M_min)&&(M<hd_M_max)&&(R>hd_R_min)&&(R<hd_R_max)&&(K>hd_K_min)&&(K<hd_K_max)&&(J>hd_J_min)&&(J<hd_J_max)&&(hd_flag_yu1==1))
	{
		speed=200;
		duty_x=2100;
		DJ_output();
		
		if(hd_flag_cy==0)
		{
			speed=200;
			duty_x=2000;
			DJ_output();
			hd_L_max=L+80;
			hd_L_min=L-80;
			hd_M_max=M+80;
			hd_M_min=M-80;
			hd_R_max=R+80;
			hd_R_min=R-80;
			hd_K_max=K+100;
			hd_K_min=K-100;	
			hd_J_max=J+100;
			hd_J_min=J-100;
			hd_flag_cy=1;
		}

		hd_flag_go=1;
		hd_flag_a=1;
		speed=200;
		duty_x=1800;
		DJ_output();
		pwm_duty(PWMB_CH1_P74, 980);
		delay_ms(230);
		timer_4huan1=1;
		hd_flag_go=0;
		hd_flag_yu1=0;
		sc_flag_yu=1;
				
//		speed=200;
//		duty_x=2100;
	}

	/*三岔*/
	//進右三岔
	if((L>sc_y_L_min)&&(L<sc_y_L_max)&&(M>sc_y_M_min)&&(M<sc_y_M_max)&&(R>sc_y_R_min)&&(R<sc_y_R_max)&&(K<sc_y_K_max)&&(K>sc_y_K_min)&&(J<sc_y_J_max)&&(J>sc_y_J_min)&&(sc_flag_yu==1)&&(sc_flag_l==1))
	{
		timer_4huan1=0;
		timer_4huan2=0;
		hd_flag_b=0;
		hd_flag_a=0;
//		sc_flag_yu=1;
//		//speed=200;
//		BEEP=1;
		sc_flag_l=0;
		sc_flag_r=1;
    timer_4S=1;
//		BEEP=1;
		speed=200;
		duty_x=1400;
		pwm_duty(PWMB_CH1_P74, 1020);
		DJ_output();
		delay_ms(230);
	  sc_flag_yu=0;
		go_in=1;
		speed=300;
		duty_x=2700;
		DJ_output();
//		BEEP=0;
	}
	//進左三岔
	if((L>sc_y_L_min)&&(L<sc_y_L_max)&&(M>sc_y_M_min)&&(M<sc_y_M_max)&&(R>sc_y_R_min)&&(R<sc_y_R_max)&&(K<sc_y_K_max)&&(K>sc_y_K_min)&&(J<sc_y_J_max)&&(J>sc_y_J_min)&&(sc_flag_yu==1)&&(sc_flag_l==0))
	{
		timer_4huan1=0;
		timer_4huan2=0;
		hd_flag_b=0;
		hd_flag_a=0;
		sc_flag_r=0;
		sc_flag_l=1;
		speed=200;
		duty_x=1500;
		DJ_output();
		pwm_duty(PWMB_CH1_P74, 810);
		delay_ms(200);
		sc_flag_yu=0;
		speed=400;
		duty_x=2700;
	}
	if(go_in==1&&((Hall==0)||(Hall1==0)))
	{
	
		pwm_duty(PWMB_CH1_P74, 900);
//		Mot_A=0;
//    Mot_B=0;
  	pwm_duty(PWMA_CH1P_P60, 0);  //左轮正
		pwm_duty(PWMA_CH3P_P64, 0);   //右轮正
		
		delay_ms(255);
		delay_ms(255);
		delay_ms(255);
		Mot_A=1;
		Mot_B=1;
		pwm_duty(PWMB_CH1_P74, 800);
		pwm_duty(PWMA_CH1P_P60, 1800);  //左轮正
		pwm_duty(PWMA_CH3P_P64, 1800);   //右轮正
		delay_ms(255);
	  delay_ms(255);
		delay_ms(255);
		delay_ms(255);
		delay_ms(255);
		delay_ms(255);
		pwm_duty(PWMB_CH1_P74, 910);
		pwm_duty(PWMA_CH1P_P60, 1600);  //左轮正
		pwm_duty(PWMA_CH3P_P64, 1600);   //右轮正
		
    delay_ms(200);
		delay_ms(200);
		pwm_duty(PWMA_CH1P_P60, 0);  //左轮正
		pwm_duty(PWMA_CH3P_P64, 0);   //右轮正
    DisableGlobalIRQ();	
		ting_flag=1;

	}
   
	if(timer_4S==1)
	{
	  timer_4A++;
	}
	if(timer_4A==1500)
	{
	  speed=200;
		duty_x=2000;
	}
	
	if(timer_4huan1==1)
	{
	  timer_4huan2++;
	}
	if(timer_4huan2==800)
	{
	  speed=400;
		duty_x=2500;
	}
	
//	if((L>sc_L_min)&&(L<sc_L_max)&&(M>sc_M_min)&&(M<sc_M_max)&&(R>sc_R_min)&&(R<sc_R_max)&&(K<sc_K_max)&&(K>sc_K_min)&&(sc_flag_yu==1)&&(sc_flag_l==1))
//	{
//		sc_flag_l=0;
//		sc_flag_r=1;

//		BEEP=1;
//		pwm_duty(PWMB_CH1_P74, 1000);
//		DJ_output();
//		delay_ms(250);
//		delay_ms(50);
//	  sc_flag_yu=0;
//	}
//	//||((L>sc_L_min_r)&&(L<sc_L_max_r)&&(M>sc_M_min_r)&&(M<sc_M_max_r)&&(R>sc_R_min_r)&&(R<sc_R_max_r)&&(K<sc_K_max_r)&&(K>sc_K_min_r))
//	if((L>sc_L_min)&&(L<sc_L_max)&&(M>sc_M_min)&&(M<sc_M_max)&&(R>sc_R_min)&&(R<sc_R_max)&&(K<sc_K_max)&&(K>sc_K_min)&&(sc_flag_yu==1)&&(sc_flag_l==0))
//	{
//		sc_flag_r=0;
//		sc_flag_l=1;
//		pwm_duty(PWMB_CH1_P74, 810);
//		DJ_output();
//		BEEP=1;
//		delay_ms(250);
//		delay_ms(50);
//		sc_flag_yu=0;
//	}
//		if(R<=20&&L<=20&&M<=20&&K<=20&&J<=20)
//		{
//			  delay_ms(255);
//			if(R<=20&&L<=20&&M<=20&&K<=20&&J<=20)
//		  {
//		  	pwm_duty(PWMA_CH1P_P60, 0);  //左轮正
//		    pwm_duty(PWMA_CH3P_P64, 0);   //右轮正
//			  DisableGlobalIRQ();
//			}
//		}
/******************************/
	Direction_Out();
	if(ting_flag==0)
	{
	  DJ_output();
	}
	
}


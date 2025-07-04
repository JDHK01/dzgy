#include "car_control.h"
#include "motor_drive.h"
#include "speed_encoder.h"
#include "UltrasonicWave.h"
#include "oled_i2c.h"
#include "ctrl_menu.h"
#define CHA +11
int mission=0;
int a=0;
int b=0;
__IO CarCtrl_State_TypeDef g_car_ctrl_state = CarCtrl_STOP ;

avoid_ctrl_t g_avoid_ctrl = {0, 0, 0, -1, 0, {0, 0}};


car_config_t g_CarConfig = 
{
	.speed_KP = 8,
	.speed_KI = 0.1 ,
	.speed_KD = 0
};
car_ctrl_t 	g_CarCtrl;

car_plan_t* g_CarPlan_Ptr;

/*car_plan_t g_CarPlan_Base[] =
{
	{ 55  , { 0 , 0} , 0 , 100 } ,   		// test steer moto
	{ -55  , { 0 , 0} , 0 , 100 } ,  		// test steer moto
	
	{ 16  , { 500 , 500} , 0 , 200 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ -55  , { 500 , 500} , 0 , 110 } ,		// turn right 1.1s 
	
	{ 16  , { 500 , 500} , 0 , 150 } ,		// run 1.5s with 500mm/s speed straightly
	{ -55  , { 500 , 500} , 0 , 110 } ,   // turn right 1.1s 
	
	{ 16  , { 500 , 500} , 0 , 120 } ,		// run 1.2s with 500mm/s speed straightly
	{ -55  , { 500 , 500} , 0 , 135 } ,		// turn right 1.35s 
	
	{ 16  , { 500 , 500} , 0 , 180 } ,		// run 1.8s with 500mm/s speed straightly
	{ 16  , { 0   , 0  } , 0 , 0 } ,		// stop
};
*/
//7du
/*beifen 
	{ CHA, { 2000, 2000} , 0 , 120 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA+75, {500 , 550} , 0 , 200 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA, { 2000 , 2000} , 0 , 85 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA+75, {600 , 650} , 0 , 160 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA-20, { 500, 500} , 0 , 0 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA  , { 0   , 0  } , 0 , 0 } ,		// stop
	*/
	
car_plan_t g_CarPlan_Base[] =
{	/*
	{ CHA, { 2000, 2000} , 0 , 110 } ,  	// run 2s with 500mm/s speed straightly 1m前为左
	{ CHA+75,{520 , 1350} , 0 , 160 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA, { 2000 , 2000} , 0 , 60} ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA+75, {550 , 1350} , 0 , 90 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA-25, { 2000, 2000} , 0 ,0 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA  , { 0   , 0  } , 0 , 0 }		// stop
	/*
	{ CHA, { 2500, 2500} , 0 , 110 } ,  	// run 2s with 500mm/s speed straightly 1m前为左
	{ CHA+75,{700 , 2000} , 0 , 80 } ,  	// run 2s with 500mm/s speed straightly 1m
	//{ CHA, { 2500 , 2500} , 0 , 30 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA+75, {700 , 1500} , 0 , 85 } ,  	// run 2s with 500mm/s speed straightly 1m
	//{ CHA-30, { 2500, 2500} , 0 ,5 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA  , { 0   , 0  } , 0 , 0 } ,		// stop
	*/
	{ CHA-8, { 2000, 2000} , 0 , 120 } ,  	// run 2s with 500mm/s speed straightly 1m前为左
	{ CHA+75,{480 , 1350} , 0 , 148  } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA, { 2000 , 2000} , 0 , 90} ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA+75, {480 , 1350} , 0 ,80     } ,  	// run 2s with 500mm/s speed straightly 1m
	// { CHA+25, { 2000, 2000} , 0 , 10} ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA  , { 0   , 0  } , 0 , 0 }		// stop
	//方向一（8.07V），CHA ：-11，后面的代码是方向二
	/*
		{ CHA-10, { 2000, 2000} , 0 , 120 } ,  	// run 2s with 500mm/s speed straightly 1m前为左
	{ CHA+75,{480 , 1350} , 0 , 150 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA, { 2000 , 2000} , 0 , 80} ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA+75, {550 , 1350} , 0 ,75     } ,  	// run 2s with 500mm/s speed straightly 1m
	// { CHA+25, { 2000, 2000} , 0 , 10} ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA  , { 0   , 0  } , 0 , 0 }		// stop
	*/
	
	//方向二  8。154  
	/*
		{ CHA+10, { 2000, 2000} , 0 , 120 } ,  	// run 2s with 500mm/s speed straightly 1m前为左
	{ CHA-75,{480 , 1350} , 0 , 150 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA, { 2000 , 2000} , 0 , 85} ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA-75, {550 , 1350} , 0 , 75 } ,  	// run 2s with 500mm/s speed straightly 1m
	//{ CHA+25, { 2000, 2000} , 0 , 10} ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA  , { 0   , 0  } , 0 , 0 }		// stop
	*/
};
car_plan_t g_CarPlan_Supper[] =
{
	/*
	{ 55  , { 0 , 0} , 0 , 100 } ,   		// test sreer moto
	{ -55  , { 0 , 0} , 0 , 100 } ,  		// test sreer moto
	
	{ 17  , { 500 , 500} , 20000 , 100 } ,  	// run 20s with 50mm/s speed straightly or block less than 200mm
	{ -55  , { 500 , 500} , 0 , 110 } ,		// turn right 1.1s 
	
	{ 17  , { 300 , 300} , 0 , 30 } ,		// run 1s with 50mm/s speed straightly
	{ 17  , { 0 ,  0} , 0 , 100 } ,		// run 1s with 50mm/s speed straightly
	{ 80  , { 200 , 400} , 0 , 150 } ,     // turn left 1.1s 
	
	{ 17  , { 100 , 100} , 0 , 30 } ,		// run 1s with 50mm/s speed straightly
	{ 80  , { 200 , 400} , 0 , 150 } ,		// turn left 1.1s 
	
	{ 17  , { 100 , 100} , 0 , 30 } ,		// run 1s with 50mm/s speed straightly
	{ 17  , { 0 ,  0} , 0 , 100 } ,		// run 1s with 50mm/s speed straightly
	{ -55  , { 500 , 500} , 0 , 110 } ,		// turn right 1.1s 
	
	{ 17  , { 500 , 500} , 0 , 100 } ,		// run 1s with 50mm/s speed straightly
	{ 17  , { 0   , 0  } , 0 , 0 } ,		// stop
	*/


	/*终极版本（无停止）
	{ CHA  , {1000 , 1000} , 40000 , 100 } ,  	// run 20s with 50mm/s speed straightly or block less than 200mm
	{ CHA-75, {500 , 500} , 0 , 100 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA+75, {500 , 500} , 0 , 200 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA-50, {250 , 250} , 0 , 100 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA-50  , {500 , 500} , 0,70 } ,  	// run 20s with 50mm/s speed straightly or block less than 200mm
	{ CHA  , {1000 , 1000} , 0 , 100 } ,  	// run 20s with 50mm/s speed straightly or block less than 200mm
	*/
/*任务2方法1

{ CHA  , {200 ,200} , 30000, 1000 } ,  	//0 run 20s with 50mm/s speed straightly or block less than 200mm
	{ CHA, {-200 , -200} , 0 , 200 } ,  	//1run 2s with 500mm/s speed straightly 1m
	{ CHA+75, {200 , 200} ,  35000 , 100 } ,  	//2 run 2s with 500mm/s speed straightly 1m
	{ CHA, {250 , 250} ,  35000, 220 } ,  	// 3run 2s with 500mm/s speed straightly 1m
	{ CHA-50 , {250 , 250} ,35000,65} ,//4
	{ CHA, {250 , 250} ,35000,200 },
  { CHA+15 , {250 , 250} ,35000,200 } ,
  {CHA , { 0   , 0  } , 0 , 0 } ,	
	*/
	//任务2方法2
	
	{ CHA  , {400 ,400} , 40000, 1000 } ,  	//0 run 20s with 50mm/s speed straightly or block less than 200mm
	  {CHA , { 0   , 0  } , 0 , 50 } ,	

	{ CHA, {600 , -500} , 0 , 180} ,  	//1run 2s with 500mm/s speed straightly 1m   1  转向
		  {CHA , { 0   , 0  } , 0 , 50 } ,	

	{ CHA, {200 , 200} ,  30000 , 200 } ,  	//2 run 2s with 500mm/s speed straightly 1m 2转向后走
		  {CHA , { 0   , 0  } , 0 , 50 } ,	

	{ CHA, {-500 , 600} ,  0, 140 } ,  	// 3run 2s with 500mm/s speed straightly 1m    3回转侦测
		  {CHA , { 0   , 0  } , 0 , 50 } ,	

	{ CHA  , {500 ,500} , 30000, 150 } ,                                              // 4 安全执直行
		  {CHA , { 0   , 0  } , 0 , 100 } ,	

	{ CHA, {-500 , 600} ,  0, 140 } ,  	// 3run 2s with 500mm/s speed straightly 1m    3回转侦测
		  {CHA , { 0   , 0  } , 0 , 50 } ,	

	{ CHA  , {200 ,200} , 30000, 140 } ,                                               // 6 回转后执行
		  {CHA , { 0   , 0  } , 0 , 50 } ,	

	{ CHA, {600 , -500} , 0 , 180} ,                                                  //7 转向复位
	  {CHA , { 0   , 0  } , 0 , 50 } ,	

		{ CHA  , {200 ,200} , 30000, 200 } ,//       


  {CHA , { 0   , 0  } , 0 , 0 } ,	
 	//4 run 20s with 50mm/s speed straightly or block less than 200mm
				// 5stop
	  	// run 20s with 50mm/s speed straightly or block less than 200mm
	// { CHA-75, {550 , 500} , 0 , 200 } ,  	// run 2s with 500mm/s speed straightly 1m
	// { CHA  , {500 , 500} ,  0 , 100 } ,  	// run 20s with 50mm/s speed straightly or block less than 200mm
	
		// 单纯测试使用
	//{ CHA   , { 500 , 500} , 0 , 10000 } ,  	// run 20s with 50mm/s speed straightly or block less than 200mm

};

car_plan_t g_CarPlan_AvoidDemo[] =
{
    { 0,    {300, 300}, 25000, 500},
    { 0,    {400, 400}, 25000, 500},
    { 20,   {300, 300}, 25000, 300},
    { -20,  {300, 300}, 25000, 300},
    { 0,    {0, 0},     0,     0},
};



void CarCtrl_AvoidObstacle(void)
{
    // ??????
    #define AVOID_IDLE          0
    #define AVOID_DETECTED      1
    #define AVOID_BACKUP        2
    #define AVOID_TURN_OUT      3
    #define AVOID_FORWARD_OUT   4
    #define AVOID_TURN_BACK     5
    #define AVOID_FORWARD_BACK  6
    #define AVOID_RECOVER       7
    
    // ??????
    const uint32_t OBSTACLE_DIST = 25000;   // 250mm
    const int16_t AVOID_ANGLE = 45;         // ????
    const uint16_t TURN_OUT_TIME = 80;      // ???????
    const uint16_t FORWARD_OUT_TIME = 60;   // ??????
    const uint16_t TURN_BACK_TIME = 80;     // ??????
    const uint16_t FORWARD_BACK_TIME = 60;  // ??????
    
    switch(g_avoid_ctrl.state) {
        case AVOID_IDLE:
            // ?????
            if (g_ultrawave_data[0].distance < OBSTACLE_DIST && 
                g_ultrawave_data[0].distance > 100) {
                // ??????
                g_avoid_ctrl.original_angle = g_CarCtrl.car_angle;
                g_avoid_ctrl.original_speed[0] = g_CarCtrl.car_speed_set[0];
                g_avoid_ctrl.original_speed[1] = g_CarCtrl.car_speed_set[1];
                
                // ??
                g_CarCtrl.car_speed_set[0] = 0;
                g_CarCtrl.car_speed_set[1] = 0;
                g_avoid_ctrl.state = AVOID_DETECTED;
                g_avoid_ctrl.timer = 0;
                
                // ????
                g_avoid_ctrl.turn_direction *= -1;
            }
            break;
            
        case AVOID_DETECTED:
            // ????
            if (++g_avoid_ctrl.timer > 10) {  // 100ms
                g_avoid_ctrl.state = AVOID_BACKUP;
                g_avoid_ctrl.timer = 0;
            }
            break;
            
        case AVOID_BACKUP:
            // ????
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
            // ?????(???)
            g_CarCtrl.car_angle = g_avoid_ctrl.original_angle + 
                                  (g_avoid_ctrl.turn_direction * AVOID_ANGLE);
            // ??????
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
            // ??????
            g_CarCtrl.car_speed_set[0] = 350;
            g_CarCtrl.car_speed_set[1] = 350;
            
            if (++g_avoid_ctrl.timer > FORWARD_OUT_TIME) {
                g_avoid_ctrl.state = AVOID_TURN_BACK;
                g_avoid_ctrl.timer = 0;
            }
            break;
            
        case AVOID_TURN_BACK:
            // ????
            g_CarCtrl.car_angle = g_avoid_ctrl.original_angle - 
                                  (g_avoid_ctrl.turn_direction * AVOID_ANGLE);
            // ??????
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
            // ?????????
            g_CarCtrl.car_speed_set[0] = 350;
            g_CarCtrl.car_speed_set[1] = 350;
            
            if (++g_avoid_ctrl.timer > FORWARD_BACK_TIME) {
                g_avoid_ctrl.state = AVOID_RECOVER;
                g_avoid_ctrl.timer = 0;
            }
            break;
            
        case AVOID_RECOVER:
            // ?????????
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



/////////////////////////////////////////////////////////////////////////////////
// Menu control
//
/////////////////////////////////////////////////////////////////////////////////
void CarCtrl_Start( void )
{
	g_car_ctrl_state = CarCtrl_IDLE ;
}

void CarCtrl_Stop( void )
{
	g_car_ctrl_state = CarCtrl_STOP ;
	// Ctrl_Menu_Show();
}

void CarCtrl_SpeedUp( void )
{
	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
		g_CarCtrl.car_speed_set[i] += 5 ;
}

void CarCtrl_SpeedDown( void )
{
	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
		g_CarCtrl.car_speed_set[i] -= 5 ;
}

void CarCtrl_SpeedStop( void )
{
	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
		g_CarCtrl.car_speed_set[i] = 0 ;
}

void CarCtrl_Forward( void )
{
	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
		g_CarCtrl.car_speed_set[i] = abs( g_CarCtrl.car_speed_set[i] );
}

void CarCtrl_Backward( void )
{
	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
		g_CarCtrl.car_speed_set[i] = -1*abs( g_CarCtrl.car_speed_set[i] );
}

void CarCtrl_Straight( void )
{
	g_CarCtrl.car_angle = CHA;
	Steer_Moto_Ctrl( STEER_MOTO_POS , g_CarCtrl.car_angle );
}

void CarCtrl_Right( void )
{
	g_CarCtrl.car_angle -= 1 ;
	// if ( g_CarCtrl.car_angle < -45 ) g_CarCtrl.car_angle = -45 ;
	if ( g_CarCtrl.car_angle < -90 ) g_CarCtrl.car_angle = -90 ;
	Steer_Moto_Ctrl(STEER_MOTO_POS , g_CarCtrl.car_angle );
}

void CarCtrl_Left( void )
{
	g_CarCtrl.car_angle += 1 ;
	// if ( g_CarCtrl.car_angle > 45 ) g_CarCtrl.car_angle = 45 ;
	if ( g_CarCtrl.car_angle > 90 ) g_CarCtrl.car_angle = 90 ;
	Steer_Moto_Ctrl(STEER_MOTO_POS , g_CarCtrl.car_angle );
}



/////////////////////////////////////////////////////////////////////////////////

void CarCtrl_Init( void )
{
	car_config_t *p_car_cfg = & g_CarConfig ;
	memset( &g_CarCtrl , 0
	, sizeof(g_CarCtrl) );
	g_CarPlan_Ptr = g_CarPlan_Base;
		mission=0;
 
}

void CarCtrl_Speed_PID( )
{
	static int32_t last_speed[DRIVE_MOTO_NUM] = {0,0};
	static int32_t speed_intergrade[DRIVE_MOTO_NUM] = {0,0};
	int32_t speed_error[DRIVE_MOTO_NUM];
	int32_t speed_diff[DRIVE_MOTO_NUM];

	for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
	{
		speed_error[i] = g_CarCtrl.car_speed_set[i] - g_speed_encoder[i].speed ;
		speed_intergrade[i] = speed_intergrade[i] + speed_error[i] ; 
		speed_diff[i] = last_speed[i] - g_speed_encoder[i].speed  ;
		last_speed[i] = g_speed_encoder[i].speed ;		
		g_CarCtrl.moto_drive[i] = speed_error[i]*g_CarConfig.speed_KP +
															speed_intergrade[i] * g_CarConfig.speed_KI +
															speed_diff[i] * g_CarConfig.speed_KD  ;
		
		//Drive_Moto_Ctrl( i , g_CarCtrl.moto_drive[i] );
	}
	
	Drive_Moto_Ctrl( 0 , g_CarCtrl.moto_drive[0] );
	Drive_Moto_Ctrl( 1 , -g_CarCtrl.moto_drive[1] );	
}

void CarCtrl_PlanSet( void )
{
	car_plan_t* car_plan_ptr ;
	
	car_plan_ptr = g_CarPlan_Ptr+a ;
	
	if ( car_plan_ptr->run_time_set == 0 )
	{
		g_car_ctrl_state = CarCtrl_STOP;
		Steer_Moto_Ctrl(STEER_MOTO_POS ,car_plan_ptr->car_angle_set);
		for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
			Drive_Moto_Ctrl( i , 0);
		memset( &g_CarCtrl , 0 , sizeof(g_CarCtrl) );
		return ;
	}
	
	if ( g_CarCtrl.run_time == 0 )  // load plan 
	{
		g_CarCtrl.run_time ++ ;
		for( int32_t i = 0 ; i < DRIVE_MOTO_NUM ; i++)
			g_CarCtrl.car_speed_set[i] = car_plan_ptr->car_speed_set[i] ;
		Steer_Moto_Ctrl(STEER_MOTO_POS , car_plan_ptr->car_angle_set );		
	}
	else														// execute plan 
	{
		g_CarCtrl.run_time ++ ;
		if (								// plan over
			   g_ultrawave_data[0].distance < car_plan_ptr->front_distance_set&&g_ultrawave_data[0].distance >0)   // distance too close
		{
			//if(g_ultrawave_data[0].distance != -1){
				g_CarCtrl.run_time = 0 ;
				a=1;	
     			
			//}
		}
		if (g_CarCtrl.run_time == car_plan_ptr->run_time_set )   // distance too close
		{
			//if(g_ultrawave_data[0].distance != -1){
				g_CarCtrl.run_time = 0 ;
			if(a==1){
			b++;
			}
			if((a==12)&&(b!=0)){
			b--;
				
			}else{
			a++;
			}
						
			//}
		}
		
		
	}
}


void CarCtrl_Show( void ) 
{
	static uint8_t  index = 0 ;
	static int32_t  speed[DRIVE_MOTO_NUM] = { 0 , 0 } ;
	static int32_t  pwm[DRIVE_MOTO_NUM] = { 0 , 0 } ;
	uint8_t 				buf[17];
	
	for (int i = 0 ; i < DRIVE_MOTO_NUM ; i++) 
	{	
		speed[i] += g_speed_encoder[ i ].speed ;
		pwm[i] += g_CarCtrl.moto_drive[ i ] ;
	}
	
	if ( index < 9 ) 
	{
		index++;
	}
	else
	{
		index = 0 ;
		speed[0] = speed[0] / 10 ;
		speed[1] = speed[1] / 10 ;
		pwm[0] = pwm[0] / 10 ;
		pwm[1] = pwm[1] / 10 ;
	
		
		 sprintf( buf , "%d" ,mission);
		OLED_ShowAscii( 0,0, buf , 16 ,0 );
		sprintf( buf , "%d" , g_ultrawave_data[0].distance);
		OLED_ShowAscii( 1,0, buf , 16 ,0 );

		speed[0] = 0 ;
		speed[1] = 0 ;		
		pwm[0] = 0 ;
		pwm[1] = 0 ;
	}
}

void CarCtrl_Process( void )
{
	if ( g_car_ctrl_state == CarCtrl_STOP ) return ;
	if ( g_car_ctrl_state == CarCtrl_START ) 
	{
		g_car_ctrl_state = CarCtrl_IDLE ;
		Speed_Calculate();
		CarCtrl_Speed_PID();
	 CarCtrl_Show();
		
		CarCtrl_PlanSet();
	}
}

// ? car_control.c ?????????
car_plan_t g_CarPlan_Mission1[] = 
{
	{ CHA, { 2000, 2000} , 0 , 120 } ,  	// run 2s with 500mm/s speed straightly 1m前为左
	{ CHA+75,{520 , 1350} , 0 , 160 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA, { 2000 , 2000} , 0 , 70} ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA+75, {550 , 1350} , 0 , 90 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA-25, { 2000, 2000} , 0 ,0 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA  , { 0   , 0  } , 0 , 0 }	,	// stop
};

car_plan_t g_CarPlan_Mission2[] = 
{
	{ CHA  , {1000 , 1000} , 40000 , 100 } ,  	// run 20s with 50mm/s speed straightly or block less than 200mm
	{ CHA-75, {500 , 500} , 0 , 100 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA+75, {500 , 500} , 0 , 200 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA-50, {250 , 250} , 0 , 100 } ,  	// run 2s with 500mm/s speed straightly 1m
	{ CHA-50  , {500 , 500} , 0,70 } ,  	// run 20s with 50mm/s speed straightly or block less than 200mm
	{ CHA  , {1000 , 1000} , 0 , 100 } ,  	// run 20s with 50mm/s speed straightly or block less than 200mm
	{CHA , { 0   , 0  } , 0 , 0 }
};

car_plan_t g_CarPlan_Mission3[] = 
{
	{ CHA  , {400 ,400} , 40000, 1000 } ,  	//0 run 20s with 50mm/s speed straightly or block less than 200mm
	  {CHA , { 0   , 0  } , 0 , 50 } ,	

	{ CHA, {600 , -500} , 0 , 180} ,  	//1run 2s with 500mm/s speed straightly 1m   1  转向
		  {CHA , { 0   , 0  } , 0 , 50 } ,	

	{ CHA, {200 , 200} ,  30000 , 200 } ,  	//2 run 2s with 500mm/s speed straightly 1m 2转向后走
		  {CHA , { 0   , 0  } , 0 , 50 } ,	

	{ CHA, {-500 , 600} ,  0, 140 } ,  	// 3run 2s with 500mm/s speed straightly 1m    3回转侦测
		  {CHA , { 0   , 0  } , 0 , 50 } ,	

	{ CHA  , {500 ,500} , 30000, 150 } ,                                              // 4 安全执直行
		  {CHA , { 0   , 0  } , 0 , 100 } ,	

	{ CHA, {-500 , 600} ,  0, 140 } ,  	// 3run 2s with 500mm/s speed straightly 1m    3回转侦测
		  {CHA , { 0   , 0  } , 0 , 50 } ,	

	{ CHA  , {200 ,200} , 30000, 140 } ,                                               // 6 回转后执行
		  {CHA , { 0   , 0  } , 0 , 50 } ,	

	{ CHA, {600 , -500} , 0 , 180} ,                                                  //7 转向复位
	  {CHA , { 0   , 0  } , 0 , 50 } ,	

		{ CHA  , {200 ,200} , 30000, 200 } ,//       


  {CHA , { 0   , 0  } , 0 , 0 } ,	
};

// ????????
void CarCtrl_Mission1(void)
{
		CarCtrl_Init( );
		g_CarPlan_Ptr = g_CarPlan_Mission1;
	mission	= 1;
		g_car_ctrl_state = CarCtrl_IDLE ;
    
    // OLED_clear();
    // OLED_ShowAscii(0, 0, "Mission 1", 16, 0);
    // HAL_Delay(1000);
}

void CarCtrl_Mission2(void)
{
		CarCtrl_Init(  );
	  g_CarPlan_Ptr = g_CarPlan_Mission2;
		
	mission=2;
    // g_CarCtrl.run_step = 0;
    // g_CarCtrl.run_time = 0;
    g_car_ctrl_state = CarCtrl_IDLE ;

    
    // OLED_clear();
    // OLED_ShowAscii(0, 0, "Mission 2", 16, 0);
    // HAL_Delay(1000);
}

void CarCtrl_Mission3(void)
{
		CarCtrl_Init( );
    g_CarPlan_Ptr = g_CarPlan_Mission3;
	mission=3;
    // g_CarCtrl.run_step = 0;
    // g_CarCtrl.run_time = 0;
    g_car_ctrl_state = CarCtrl_IDLE ;

    
    // OLED_clear();
    // OLED_ShowAscii(0, 0, "Mission 3", 16, 0);
    // OLED_ShowAscii(1, 0, "Avoid ON", 16, 0);
    // HAL_Delay(1000);
}

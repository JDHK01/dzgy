#include <stdio.h>
#include <string.h>
#include <math.h>
#include "main.h"
#include "i2c.h"
//#include "i2c_gpio.h"
#include "qmc5883.h"

I2C_HandleTypeDef * g_QMC5883_i2c = &hi2c2;
__IO uint8_t  g_qmc5883_flag  = 0 ;
uint8_t g_qmc5883_exist = 0 ;

uint8_t qmc5883_check( void )
{
	uint8_t buf ; 
	uint8_t try = 0 ; 
	uint8_t ret = 0 ;
	
	while ( try < 10 )
	{
		if( HAL_I2C_Mem_Read( g_QMC5883_i2c, QMC5883_DEV, 0x0d, I2C_MEMADD_SIZE_8BIT, (uint8_t *) &buf , 1 , 100 ) == HAL_OK )		
		//if( iic_read_regs(QMC5883_DEV,0x0d, &buf , 0x01) )
		{		
			if ( buf == 0xFF ) 
			{
				printf("QMC5883 Find!\n"  );
				ret = 1 ;
			}
			else
			{
				printf("QMC5883 not exist!\n"  );
			}
			break ;
		}
		else
		{
			try ++;
			printf("QMC5883 error!\n"  );
//			HAL_Delay(1000);
		}
	}
	return ret ;
}

void qmc5883_init(void)
{
	uint8_t buf ; 
	
	if ( qmc5883_check() == 0 ) return ;
	
	buf = 0x0d;
	if ( HAL_I2C_Mem_Write( g_QMC5883_i2c, QMC5883_DEV, 0x09, I2C_MEMADD_SIZE_8BIT, (uint8_t *) &buf , 1 , 100 ) != HAL_OK )	return;	
	buf = 0x01;
	if ( HAL_I2C_Mem_Write( g_QMC5883_i2c, QMC5883_DEV, 0x0b, I2C_MEMADD_SIZE_8BIT, (uint8_t *) &buf , 1 , 100 ) != HAL_OK )	return;	
	buf = 0x40;
	if ( HAL_I2C_Mem_Write( g_QMC5883_i2c, QMC5883_DEV, 0x20, I2C_MEMADD_SIZE_8BIT, (uint8_t *) &buf , 1 , 100 ) != HAL_OK )	return;	
	buf = 0x01;
	if ( HAL_I2C_Mem_Write( g_QMC5883_i2c, QMC5883_DEV, 0x21, I2C_MEMADD_SIZE_8BIT, (uint8_t *) &buf , 1 , 100 ) != HAL_OK )	return;	
	
	// if ( iic_write_reg(QMC5883_DEV,0x09,0x0d)== 0 ) return; //控制寄存器配置
	// if ( iic_write_reg(QMC5883_DEV,0x0b,0x01)== 0 ) return;  //设置清除时间寄存器
	// if ( iic_write_reg(QMC5883_DEV,0x20,0x40)== 0 ) return;  //
	// if ( iic_write_reg(QMC5883_DEV,0x21,0x01)== 0 ) return;  //	 
	g_qmc5883_exist = 1 ;
	printf("QMC5883 init OK!\n"  );
}

void GetQMC5883Data()
{
	int16_t mxyz[10][3] = {0};
	int32_t avermxyz[3]={0};
        uint8_t buffer[6]={0};
	int32_t angle ;
	static uint8_t cnt=0;
	
	HAL_I2C_Mem_Read( g_QMC5883_i2c, QMC5883_DEV, 0x00, I2C_MEMADD_SIZE_8BIT, (uint8_t *) &buffer , 6 , 100 );
	//iic_read_regs( QMC5883_DEV, 0, buffer, 6 );
	
	//printf("Xl=%d,Xh=%d,Yl=%d,Yh=%d,Zl=%d,Zh=%d\r\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5]);
	mxyz[cnt][0] = (int16_t)((buffer[1] << 8) | buffer[0]);	
	mxyz[cnt][1] = (int16_t)((buffer[3] << 8) | buffer[2]);			
	mxyz[cnt][2] = (int16_t)((buffer[5] << 8) | buffer[4]); 
	cnt++;
	if(cnt>=10)
	{
		cnt=0;
		for(uint8_t i=0,j=0;i<3;i++)
		{
			for(j=0;j<10;j++)
			{
				avermxyz[i]+=mxyz[j][i];
			}
			avermxyz[i]=avermxyz[i]/10;
		}
		printf("average original data :x=%d,y=%d,z=%d\r\n",avermxyz[0],avermxyz[1],avermxyz[2]);
		angle = atan2(avermxyz[1], avermxyz[0]) * 180 /3.1415926;
		
		printf("Angle : %d \n\n",angle);
	}
}

void qmc5883_test( void )
{
	
	uint8_t buf[6] ;

	if ( g_qmc5883_exist == 1 && g_qmc5883_flag == 1 )
	{
		g_qmc5883_flag = 0 ;
		if( HAL_I2C_Mem_Read( g_QMC5883_i2c, QMC5883_DEV, 0x06, I2C_MEMADD_SIZE_8BIT, (uint8_t *) &buf , 1 , 100 ) == HAL_OK ) return ;		
		// if ( iic_read_regs( QMC5883_DEV , 0x06 , buf , 1 ) ==0 ) return ;
		if ( ( buf[0] & 0x01 ) == 0 ) return ;
		GetQMC5883Data();

	}
}
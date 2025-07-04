#include "main.h"

#define I2C_SDA_Pin GPIO_PIN_11
#define I2C_SDA_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_10
#define I2C_SCL_GPIO_Port GPIOB


#define IIC_SCL(x)    do{ x ? \
	HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_SET) : \
	HAL_GPIO_WritePin(I2C_SCL_GPIO_Port, I2C_SCL_Pin, GPIO_PIN_RESET); \
                          }while(0)       /* SCL */
#define IIC_SDA(x)    do{ x ? \
	HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_SET) : \
	HAL_GPIO_WritePin(I2C_SDA_GPIO_Port, I2C_SDA_Pin, GPIO_PIN_RESET); \
                          }while(0)       /* SDA */
/* 读取SDA */
#define IIC_READ_SDA     HAL_GPIO_ReadPin(I2C_SDA_GPIO_Port, I2C_SDA_Pin) 

void IIC_Init(uint8_t mode )
{					     

}

static void iic_delay(void)
{
  uint32_t Delay = 50 ;
  do
  {
		__NOP();
  } while (Delay--);
}

/**
 * @brief       产生IIC起始信号
 * @param       无
 * @retval      无
 */
void iic_start(void)
{
  IIC_SDA(1);
  iic_delay();
  IIC_SCL(1);
  iic_delay();
  IIC_SDA(0);     /* START信号: 当SCL为高时, SDA从高变成低, 表示起始信号 */
  iic_delay();
  IIC_SCL(0);     /* 钳住I2C总线，准备发送或接收数据 */
  iic_delay();
}
 
/**
 * @brief       产生IIC停止信号
 * @param       无
 * @retval      无
 */
void iic_stop(void)
{
    IIC_SDA(0);     /* STOP信号: 当SCL为高时, SDA从低变成高, 表示停止信号 */
    iic_delay();
    IIC_SCL(1);
    iic_delay();
    IIC_SDA(1);     /* 发送I2C总线结束信号 */
    iic_delay();
}
 
/**
 * @brief       等待应答信号到来
 * @param       无
 * @retval      1，接收应答失败
 *              0，接收应答成功
 */
uint8_t iic_wait_ack(void)
{
    uint8_t waittime = 0;
    uint8_t rack = 0;
 
    IIC_SDA(1);     /* 主机释放SDA线(此时外部器件可以拉低SDA线) */
    iic_delay();
   	IIC_SCL(1);     /* SCL=1, 此时从机可以返回ACK */
    iic_delay();
    while (IIC_READ_SDA)    
    {
        waittime++;
        if (waittime > 250)
        {
            //iic_stop();
            rack = 1;
            break;
        }
    }

    IIC_SCL(0);     /* SCL=0, 结束ACK检查 */
    iic_delay();
    return rack;
}
 
/**
 * @brief       产生ACK应答
 * @param       无
 * @retval      无
 */
void iic_ack(void)
{
    IIC_SDA(0);     /* SCL 0 -> 1 时 SDA = 0,表示应答 */
    iic_delay();
    IIC_SCL(1);     /* 产生一个时钟 */
    iic_delay();
    IIC_SCL(0);
    iic_delay();
    IIC_SDA(1);     /* 主机释放SDA线 */
    iic_delay();
}
 
/**
 * @brief       不产生ACK应答
 * @param       无
 * @retval      无
 */
void iic_nack(void)
{
    IIC_SDA(1);     /* SCL 0 -> 1  时 SDA = 1,表示不应答 */
    iic_delay();
    IIC_SCL(1);     /* 产生一个时钟 */
    iic_delay();
    IIC_SCL(0);
    iic_delay();
}
 
/**
 * @brief       IIC发送一个字节
 * @param       data: 要发送的数据
 * @retval      无
 */
void iic_send_byte(uint8_t data)
{
    uint8_t t;
    
    for (t = 0; t < 8; t++)
    {
        IIC_SDA((data & 0x80) >> 7);    /* 高位先发送 */
        iic_delay();
        IIC_SCL(1);
        iic_delay();
        IIC_SCL(0);
        data <<= 1;     /* 左移1位,用于下一次发送 */
    }
    IIC_SDA(1);         /* 发送完成, 主机释放SDA线 */
}
 
/**
 * @brief       IIC读取一个字节
 * @param       ack:  ack=1时，发送ack; ack=0时，发送nack
 * @retval      接收到的数据
 */
uint8_t iic_read_byte(uint8_t ack)
{
    uint8_t i, receive = 0;
	
    for (i = 0; i < 8; i++ )    /* 接收1个字节数据 */
    {
        receive <<= 1;  /* 高位先输出,所以先收到的数据位要左移 */
        IIC_SCL(1);
        iic_delay();
 
        if (IIC_READ_SDA)
        {
            receive++;
        }
        
        IIC_SCL(0);
        iic_delay();
    }
 
    if (!ack)
    {
        iic_nack();     /* 发送nACK */
    }
    else
    {
        iic_ack();      /* 发送ACK */
    }
 
    return receive;
}

uint8_t iic_write_reg( uint8_t dev , uint8_t reg , uint8_t value  )
{
	uint8_t ret = 1 ;

	iic_start();
	iic_send_byte(dev);
	if ( iic_wait_ack() ) ret = 0 ;
	iic_send_byte(reg);
	if ( iic_wait_ack() ) ret = 0 ;
	iic_send_byte(value) ;	
	if ( iic_wait_ack() ) ret = 0 ;
	iic_stop();
	return ret ;
}


uint8_t iic_read_regs( uint8_t dev , uint8_t reg , uint8_t* value , uint8_t len )
{
	uint32_t i ;
	uint8_t ret = 1 ;
	iic_start();
	iic_send_byte(dev);
	if ( iic_wait_ack() ) ret = 0;
	iic_send_byte(reg);
	if ( iic_wait_ack() ) ret = 0;
	
	iic_start();
	iic_send_byte(dev+1);
	if ( iic_wait_ack() ) ret = 0;
	
	for ( i = 0 ; i < (len-1) ; i++ )
	{
		*value=iic_read_byte(1);
		value++;		
	}
	*value=iic_read_byte(0);
	iic_stop();
	return ret ;
}


#ifndef I2C_GPIO_H
#define I2C_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif
void IIC_Init(void);
uint8_t iic_write_reg( uint8_t dev , uint8_t reg , uint8_t value  );
uint8_t iic_read_regs( uint8_t dev , uint8_t reg , uint8_t* value , uint8_t len );	

#ifdef __cplusplus
}
#endif

#endif /* I2C_GPIO_H */


#ifndef QMC5883_H
#define QMC5883_H

#ifdef __cplusplus
extern "C" {
#endif

#define QMC5883_DEV   0x1A
	
void qmc5883_init(void);
void qmc5883_test( void );
	
extern __IO uint8_t  g_qmc5883_flag  ;
#ifdef __cplusplus
}
#endif

#endif /* QMC5883_H */

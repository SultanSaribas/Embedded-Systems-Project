#ifndef __HAL_TC72_H
#define __HAL_TC72_H

#ifdef __cplusplus
 extern "C" {
#endif

#define CONTROL_REG 0x80
#define START_CONV 0x10
#define TEMPR_REG 0x02
#define FRAC_FLAG 0x00C0


void tc72_init(void);
	
#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_HAL_FLASH_H */
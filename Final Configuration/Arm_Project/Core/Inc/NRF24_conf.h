#ifndef _NRF_24_CONF_H_
#define _NRF_24_CONF_H_

#define hspiX hspi1          // ← changed from hspi1 to hspi2

#define spi_w_timeout 1000
#define spi_r_timeout 1000
#define spi_rw_timeout 1000

#define csn_gpio_port GPIOA
#define csn_gpio_pin GPIO_PIN_15

#define ce_gpio_port GPIOB
#define ce_gpio_pin GPIO_PIN_6

#define htimX htim1          // ← keep this, just make sure TIM1 is enabled in CubeMX

#endif

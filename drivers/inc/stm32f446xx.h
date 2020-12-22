#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_
#include<stdint.h>
#define __vo volatile
/*
 * base addresses of Flash and SRAM memories
 */
#define FLASH_BASEADDR          0x08000000U
#define SRAM1_BASEADDR			0x20000000U
#define SRAM2_BASEADDR			0x2001C000U
#define ROM			            0x1FFF0000U
#define SRAM                    SRAM1_BASEADDR

/*
 * AHBx and APBx Bus peripheral base addresses
 */
#define PERIPH_BASE            0x40000000U
#define APB1PERIPH_BASE        PERIPH_BASE
#define APB2PERIPH_BASE        0x40010000U
#define AHB1PERIPH_BASE        0x40020000U
#define AHB2PERIPH_BASE        0x50000000U

/*
 * Base addresses of peripherals hanging on the AHB1 Bus
 */
#define GPIOA_BASEADDR        (AHB1PERIPH_BASE + 0x000)
#define GPIOB_BASEADDR        (AHB1PERIPH_BASE + 0x400)
#define GPIOC_BASEADDR        (AHB1PERIPH_BASE + 0x800)
#define GPIOD_BASEADDR        (AHB1PERIPH_BASE + 0xC00)
#define GPIOE_BASEADDR        (AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR        (AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR        (AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR        (AHB1PERIPH_BASE + 0x1C00)
#define RCC_BASEADDR          (AHB1PERIPH_BASE + 0x3800)

/*
 * Base addresses of peripherals hanging  APB1 Bus
 */

#define RTC_BASEADDR          (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASEADDR         (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASEADDR         (APB1PERIPH_BASE + 0x3000)

#define SPI2_BASEADDR         (APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR         (APB1PERIPH_BASE + 0x3C00)
#define SPIFRX_BASEADDR       (APB1PERIPH_BASE + 0x4000)

#define USART2_BASEADDR       (APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR       (APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR        (APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR        (APB1PERIPH_BASE + 0x5000)

#define I2C1_BASEADDR         (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR         (APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR         (APB1PERIPH_BASE + 0x5C00)
#define CAN1_BASEADDR         (APB1PERIPH_BASE + 0x6400)


/*
 * Base addresses of peripherals hanging  APB2 Bus
 */
#define EXTI_BASEADDR     (APB2PERIPH_BASE + 0x3C00)

#define SYSCFG_BASEADDR   (APB2PERIPH_BASE + 0x3800)

#define SPI1_BASEADDR     (APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR     (APB2PERIPH_BASE + 0x3400)

#define USART1_BASEADDR   (APB2PERIPH_BASE +0x11000)
#define USART6_BASEADDR   (APB2PERIPH_BASE +0x11400)

/**************************Peripheral register definition**************************/

typedef struct
{
	__vo uint32_t    MODER;   //GPIO port mode register
	__vo uint32_t    OTYPER2; //GPIO port output type register
	__vo uint32_t    OSPEEDR; //GPIO port output speed register
	__vo uint32_t    PUPDR;   //GPIO port pull-up/pull-down register
	__vo uint32_t    IDR;     //GPIO port input data register
	__vo uint32_t    ODR ;    //GPIO port output data register
	__vo uint32_t    BSRR;    //GPIO port bit set/reset register
	__vo uint32_t    LCKR;    //GPIO port configuration lock register
	__vo uint32_t    AFR[2];  //GPIO alternate function AFR[0] low N AFR[1]High register
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t    CR;
	__vo uint32_t    PLLCFGR;
	__vo uint32_t    CFGR;
	__vo uint32_t    CIR;
	__vo uint32_t    AHB1RSTR;
	__vo uint32_t    AHB2RSTR;
	__vo uint32_t    AHB3RSTR;
	     uint32_t    RESERVED0;
	__vo uint32_t    APB1RSTR;
	__vo uint32_t    APB2RSTR;
	     uint32_t    RESERVED1;
	     uint32_t    RESERVED2;
	__vo uint32_t    AHB1ENR;
	__vo uint32_t    AHB2ENR;
	__vo uint32_t    AHB3ENR;
	     uint32_t    RESERVED3;
	__vo uint32_t    APB1ENR;
	__vo uint32_t    APB2ENR;
	     uint32_t    RESERVED4;
	     uint32_t    RESERVED5;
	__vo uint32_t    AHB1LPENR;
	__vo uint32_t    AHB2LPENR;
	__vo uint32_t    AHB3LPENR;
	     uint32_t    RESERVED10;
	__vo uint32_t    APB1LPENR;
	__vo uint32_t    APB2LPENR;
	     uint32_t    RESERVED6;
	     uint32_t    RESERVED7;
	__vo uint32_t    BDCR;
	__vo uint32_t    CSR;
	     uint32_t    RESERVED8;
	     uint32_t    RESERVED9;
	__vo uint32_t    SSCGR;
	__vo uint32_t    PLLI2SCFGR;
	__vo uint32_t    PLLSAICFGR;
	__vo uint32_t    DCKCFGR;
	__vo uint32_t    CKGATENR;
	__vo uint32_t    DCKCFGR2;
}RCC_RegDef_t;

/*
 * Peripheral Definitions(Peripheral base addresses type casted to RegDef
 */

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC    ((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Clock Enable macros for GPIOX peripherals
 */
#define GPIOA_PCLK_EN()      (RCC->AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()      (RCC->AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()      (RCC->AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()      (RCC->AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()      (RCC->AHB1ENR |= (1<<4))
#define GPIOF_PCLK_EN()      (RCC->AHB1ENR |= (1<<5))
#define GPIOG_PCLK_EN()      (RCC->AHB1ENR |= (1<<6))
#define GPIOH_PCLK_EN()      (RCC->AHB1ENR |= (1<<7))

/*
 * Clock Enable macros for I2C peripherals
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |=(1<<21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |=(1<<22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |=(1<<23))

/*
 * Clock Enable macros for SPI peripherals
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |=(1<<12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |=(1<<14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |=(1<<15))

/*
 * Clock Enable macros for USART peripherals
 */
#define USART1_PCLK_EN() (RCC->APB2ENR |=(1<<4))
#define USART2_PCLK_EN() (RCC->APB1ENR |=(1<<17))
#define USART3_PCLK_EN() (RCC->APB1ENR |=(1<<18))
#define USART6_PCLK_EN() (RCC->APB2ENR |=(1<<5))

/*
 * Clock Enable macros for UART peripherals
 */
#define UART4_PCLK_EN() (RCC->APB1ENR |=(1<<19))
#define UART5_PCLK_EN() (RCC->APB1ENR |=(1<<20))
/*
 * Clock Enable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |=(1<<14))

/*
 * Clock Enable macros for GPIOX peripherals
 */
#define GPIOA_PCLK_DI()      (RCC->AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()      (RCC->AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()      (RCC->AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()      (RCC->AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()      (RCC->AHB1ENR &= ~(1<<4))
#define GPIOF_PCLK_DI()      (RCC->AHB1ENR &= ~(1<<5))
#define GPIOG_PCLK_DI()      (RCC->AHB1ENR &= ~(1<<6))
#define GPIOH_PCLK_DI()      (RCC->AHB1ENR &= ~(1<<7))

/*
 * Clock Enable macros for I2C peripherals
 */
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1<<23))

/*
 * Clock Enable macros for SPI peripherals
 */
#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1<<15))

/*
 * Clock Enable macros for USART peripherals
 */
#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1<<4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1<<18))
#define USART6_PCLK_DI() (RCC->APB2ENR &= ~(1<<5))

/*
 * Clock Enable macros for UART peripherals
 */
#define UART4_PCLK_DI() (RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI() (RCC->APB1ENR &= ~(1<<20))
/*
 * Clock Enable macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1<<14))

#endif /* INC_STM32F446XX_H_ */

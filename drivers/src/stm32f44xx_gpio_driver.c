#include "stm32f44xx_gpio_drivers.h"
#include <stdint.h>

/*
 * Peripheral Clock set up
 */
/**************************************************************
 * @fn                -GPIO_PeriClockControl
 *
 * @brief              -Enables or disables the periclock of the given gpio
 *
 * @param[im]          -Base address of the GPIO peripheral
 * @param[im]          -ENABLE or DISABLE macros
 * @param[im]          -
 *
 * @return             -none
 *
 * @Note               -
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
	    }else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}

    }else{
    	if(pGPIOx == GPIOA){
    		GPIOA_PCLK_DI();
    	}else if(pGPIOx == GPIOB){
    		GPIOB_PCLK_DI();
    	}else if(pGPIOx == GPIOC){
    	    GPIOC_PCLK_DI();
    	}else if(pGPIOx == GPIOD){
    		GPIOD_PCLK_DI();
    	}else if(pGPIOx == GPIOE){
    		GPIOE_PCLK_DI();
    	}else if(pGPIOx == GPIOF){
    		GPIOF_PCLK_DI();
    	}else if(pGPIOx == GPIOG){
    	    GPIOG_PCLK_DI();
    	}else if(pGPIOx == GPIOH){
    		GPIOH_PCLK_DI();
    	}
    }
}
/*
 * Init and De-Init
 */

void GPIO_Init(GPIOx_HANDLE_t *pGPIOHandle)
{
	uint32_t temp=0; //temp. register
	//Configure the mode of the gpio pin
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
		{
			temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOXr->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		    pGPIOHandle->pGPIOXr->MODER |= temp;
		}
		else{
			//Interrupt mode
			if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
			{
				//1. Configure FTSR
				EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				//Clear the corresponding RTSR BIT
				EXTI->EXTI_RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
			{
				//2. Configure RTSR
				EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				//Clear the corresponding FTSR BIT
				EXTI->EXTI_FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
			{
				//3. Configure both registers
				EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
				EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			}
			//4. Configure the GPIO port selection in SYSCFG_EXTICR
				uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
				uint8_t temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4);

				uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOXr);
				SYSCFG_PCLK_EN();
				SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

			//5. Enable the ext1 interrupt delivery using IMR
			EXTI->EXTI_IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
		}
	    temp = 0;
	// Configure the speed
	    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	    pGPIOHandle->pGPIOXr->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	    pGPIOHandle->pGPIOXr->OSPEEDR |= temp;

	    temp = 0;
    // configure the pupd settings
	    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	    pGPIOHandle->pGPIOXr->PUPDR |= temp;

	    temp =0;
	//Configure  the optype
	    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	    pGPIOHandle->pGPIOXr->OTYPER2 &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	    pGPIOHandle->pGPIOXr->OTYPER2 |= temp;

	    temp =0;
	//Configure alternate function
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
		pGPIOHandle->pGPIOXr->AFR[temp1] &= ~(0xf <<(4*temp2));
		pGPIOHandle->pGPIOXr->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*temp2));

	}
}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_REG_RESET();
		}else if(pGPIOx == GPIOB){
			GPIOB_REG_RESET();
	    }else if(pGPIOx == GPIOC){
			GPIOC_REG_RESET();
		}else if(pGPIOx == GPIOD){
			GPIOD_REG_RESET();
	    }else if(pGPIOx == GPIOE){
			GPIOE_REG_RESET();
		}else if(pGPIOx == GPIOF){
			GPIOF_REG_RESET();
	    }else if(pGPIOx == GPIOG){
			GPIOG_REG_RESET();
	    }else if(pGPIOx == GPIOH){
	        GPIOH_REG_RESET();
        }
	}
}
/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & (0x00000001));
	return value;

}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
	pGPIOx->ODR |=  (0x1 << PinNumber);
	}else{
	pGPIOx->ODR &= ~(0x1 << PinNumber);

	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}
/*
 * IRQ configuration
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
		if(EnorDi == ENABLE)
		{
			if(IRQNumber <= 31)
			{
				*NVIC_ISER0 |= (1 << IRQNumber);
			}else if(32 <= IRQNumber && IRQNumber < 64)
			{
				*NVIC_ISER1 |= (1 << (IRQNumber%32));
			}else if(IRQNumber >= 64 && IRQNumber < 96)
			{
				*NVIC_ISER2 |= (1 << (IRQNumber%64));
			}
		}else{
			if(IRQNumber <= 31)
			{
				*NVIC_ICER0 |= (1 << IRQNumber);
			}else if(32 <= IRQNumber && IRQNumber < 64)
			{
				*NVIC_ICER1 |= (1 << (IRQNumber%32));
			}else if(IRQNumber >= 64 && IRQNumber < 96)
			{
				*NVIC_ICER2 |= (1 << (IRQNumber%64));
			}
		}

}
void GPIO_Priority_Config(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASE_ADDR + (iprx + 4)) |= (IRQPriority << shift_amount);

}
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->EXTI_PR & (1 << PinNumber))
	{
		//clear pending register bit
		EXTI->EXTI_PR |= (1 << PinNumber);
	}
}

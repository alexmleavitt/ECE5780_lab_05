/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
uint32_t Read(int bytes);
void Write(int bytes, uint32_t address, uint32_t data);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Configure the system clock */
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR  |= RCC_APB1ENR_I2C2EN;
  SystemClock_Config();

	GPIOB->MODER |= (1<<23) | (1<<27) | (1<<28);
	GPIOB->MODER &= ~((1<<22)| (1<<26) | (1<<29));
	GPIOB->OTYPER |= (1<<11) | (1<<13);
	GPIOB->OTYPER &= ~(1<<14);
	GPIOB->AFR[1] |= (1<<GPIO_AFRH_AFSEL11_Pos);
	GPIOB->AFR[1] |= (5<<GPIO_AFRH_AFSEL13_Pos);
	GPIOC->MODER |= (1<<0);
	GPIOC->MODER &= ~(1<<1);
	GPIOC->OTYPER &= ~(1<<0);
	
	GPIOB->ODR |= (1<<14);
	GPIOC->ODR |= (1<<0);
	
	I2C2->TIMINGR |= (1<<28) | (13<<0) | (0xF<<8) | (2<<16)
								| (4<<20);
	I2C2->CR1 |= (1<<0);
	
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	I2C2->CR2 |= (0x69<<1); // Slave address = 0x69
	I2C2->CR2 |= (1<<16); // Transmit 1 byte
	I2C2->CR2 &= ~(1<<10); // Set to write
	I2C2->CR2 |= (1<<13); // Start
	
	// Configure the leds
	GPIOC->MODER |= (1<<12) | (1<<14) | (1<<16) | (1<<18);
	GPIOC->MODER &= ~((1<<13) | (1<<15) | (1<<17) | (1<<19));
	GPIOC->OTYPER &= ~((1<<6) | (1<<7) | (1<<8) | (1<<9));
	GPIOC->OSPEEDR &= ~((1<<12) | (1<<14) | (1<<16) | (1<<18));
	GPIOC->PUPDR &= ~((1<<12) | (1<<14) | (1<<16) | (1<<18)
									| (1<<13) | (1<<15) | (1<<17) | (1<<19));
	GPIOC->OSPEEDR &= ~((1<<0) | (1<<1));

	GPIOC->ODR &= ~((1<<6) | (1<<7) | (1<<8) | (1<<9));
	
/*
	while(!(I2C2->ISR&I2C_ISR_NACKF) && !(I2C2->ISR&I2C_ISR_TXIS))
	{
	}
	
	if(I2C2->ISR&I2C_ISR_NACKF)
	{
		GPIOC->ODR ^= (1<<6);
	}
	else
	{
		I2C2->TXDR = 0x0F;
		while(!(I2C2->ISR&I2C_ISR_TC))
		{
		}
		
		//GPIOC->ODR ^= (1<<8);
		
		I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
		I2C2->CR2 |= (0x69<<1); // Slave address = 0x69
		I2C2->CR2 |= (1<<16); // Transmit 1 byte
		I2C2->CR2 |= (1<<10); // Set to read
		I2C2->CR2 |= (1<<13); // Start
		
		while(!(I2C2->ISR&I2C_ISR_NACKF) && !(I2C2->ISR&I2C_ISR_RXNE))
	{
	}
		if(I2C2->ISR&I2C_ISR_NACKF)
		{
			GPIOC->ODR ^= (1<<6);
		}
		else
		{
			while(!(I2C2->ISR&I2C_ISR_TC))
			{
			}
			
			if((I2C2->RXDR)==0xD3)
				{
					GPIOC->ODR ^= (1<<9);
				}
				else
				{
					GPIOC->ODR ^= (1<<6);
				}
				I2C2->CR2 |= (1<<14);
		
		}
	}
	*/
	
	/*
	Write(1, 0x0F, 0);
	uint32_t result = Read(1);
	
	if((result)==0xD3)
				{
					GPIOC->ODR ^= (1<<9);
				}
				else
				{
					GPIOC->ODR ^= (1<<6);
				}
*/
	
	/*
	Write(1, 0x20, 0);
	uint32_t ctrl = Read(1);
	ctrl |= (1<<3);
	Write(2, 0x20, ctrl);
		
		 //Check that the register is being changed
	Write(1, 0x20, 0);				
	uint32_t test = Read(1);
				if((test)==0b00000111)
				{
					GPIOC->ODR ^= (1<<8);
				}
				else if((test)==ctrl)
				{
					GPIOC->ODR ^= (1<<9);
				}
				else
				{
					GPIOC->ODR ^= (1<<6);
				}
				I2C2->CR2 |= (1<<14);
	*/			
				
	int xVal;
	int yVal;
	int xAbs;
	int yAbs;
	int threshold = 10000;

				
  while (1)
  {
		
		Write(1,0xA8,0);
		xVal = Read(2);
		Write(1,0xAA,0);
		yVal = Read(2);
		
		if((xVal>>15) == 1)
			xAbs = (xVal^0xFFFF)+1;
		else
			xAbs = xVal;
		if((yVal>>15) == 1)
			yAbs = (yVal^0xFFFF)+1;
		else
			yAbs = yVal;
		
		
		if(xAbs > yAbs && xAbs > threshold)
		{
			GPIOC->ODR &= ~((1<<6) | (1<<7) | (1<<8) | (1<<9));
			if((xVal>>15) == 1)
				GPIOC->ODR |= (1<<8);
			else
				GPIOC->ODR |= (1<<9);	
		}
		
		else if(yAbs > xAbs && yAbs > threshold)
		{
			GPIOC->ODR &= ~((1<<6) | (1<<7) | (1<<8) | (1<<9));
			if((yVal>>15) == 1)
				GPIOC->ODR |= (1<<7);
			else
				GPIOC->ODR |= (1<<6);	
		}
			
   HAL_Delay(100);
	 
  }
	
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

uint32_t Read(int bytes)
{
	uint32_t data = 0;
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
		I2C2->CR2 |= (0x69<<1); // Slave address = 0x69
		I2C2->CR2 |= (bytes<<16); // Transmit 1 byte
		I2C2->CR2 |= (1<<10); // Set to read
		I2C2->CR2 |= (1<<13); // Start
		
	
		while(!(I2C2->ISR&I2C_ISR_NACKF) && !(I2C2->ISR&I2C_ISR_RXNE))
	{
	}
		if(I2C2->ISR&I2C_ISR_NACKF)
		{
			GPIOC->ODR ^= (1<<6);
		}
		else
		{
			
			data = I2C2->RXDR;
			 if(bytes > 1)
			 {
				 //data = (data<<8);
				 while(!(I2C2->ISR&I2C_ISR_RXNE)){}
					 data = data | (I2C2->RXDR<<8);
			 }
			 
			while(!(I2C2->ISR&I2C_ISR_TC))
			{
			}
		}
	
		return data;
	
}

void Write(int bytes, uint32_t address, uint32_t data)
{
	
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	I2C2->CR2 |= (0x69<<1); // Slave address = 0x69
	I2C2->CR2 |= (bytes<<16); // Transmit bytes
	I2C2->CR2 &= ~(1<<10); // Set to write
	I2C2->CR2 |= (1<<13); // Start
	
	while(!(I2C2->ISR&I2C_ISR_NACKF) && !(I2C2->ISR&I2C_ISR_TXIS))
	{
	}
	
	if(I2C2->ISR&I2C_ISR_NACKF)
	{
		GPIOC->ODR ^= (1<<6);
	}
	else
	{
		I2C2->TXDR = address;
	}
	
	if(bytes > 1){
		while(!(I2C2->ISR&I2C_ISR_TXIS))
			{
			}
			
			I2C2->TXDR = data;
	}
		
	while(!(I2C2->ISR&I2C_ISR_TC))
		{
		}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

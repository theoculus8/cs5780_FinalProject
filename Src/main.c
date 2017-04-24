/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

void SystemClock_Config(void);
void Error_Handler(void);

uint8_t Kp;
uint8_t Ki;
uint8_t Kd;
volatile int16_t x;
volatile int16_t y;
const int16_t THRESHOLD = 10000;

void I2C_init(void)
{
    // Setting the GPIO Modes
    {
        // Enable GPIOB, GPIOC in RCC
        RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
        RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
        
        GPIO_InitTypeDef initStr = {GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
                                                                GPIO_MODE_OUTPUT_PP,
                                                                GPIO_SPEED_FREQ_LOW,
                                                                GPIO_NOPULL};
        HAL_GPIO_Init(GPIOC, &initStr);
        
        // Set PB11 to alternate function mode
        GPIOB->MODER |= (2 << (2 * 11));
        GPIOB->MODER &= ~(((3 - 2) << (2 * 11)));
        // Set PB11 to open drain
        GPIOB->OTYPER |= (1 << 11);
        // Set PB11 alternate function to I2C2_SDA
        GPIOB->AFR[1] |= (1 << (4 * (11 - 8))); // AF1
        GPIOB->AFR[1] &= ~((0xF - 1) << (4 * (11 - 8)));
        
        // Set PB13 to alternate function mode
        GPIOB->MODER |= (2 << (2 * 13));
        GPIOB->MODER &= ~(((3 - 2) << (2 * 13)));
        // Set PB13 to open drain
        GPIOB->OTYPER |= (1 << 13);
        // Set PB13 alternate function to I2C2_SCL
        GPIOB->AFR[1] |= (5 << (4 * (13 - 8))); // AF5
        GPIOB->AFR[1] &= ~(((0xF - 5) << (4 * (13 - 8))));        
        
        // Set PB14 to output mode
        GPIOB->MODER |= (1 << (2 * 14));
        GPIOB->MODER &= ~((3 - 1) << (2 * 14));
        // Set PB14 to push-pull
        GPIOB->OTYPER &= ~(1 << 14);
        // Set PB14 high
        GPIOB->ODR |= (1 << 14);
        
        // Set PC0 to output mode
        GPIOC->MODER |= (1 << (2 * 0));
        GPIOC->MODER &= ~(((3 - 1) << (2 * 0)));
        // Set PC0 to push-pull
        GPIOC->OTYPER &= ~(1 << 0);
        // Set PC0 high
        GPIOC->ODR |= (1 << 0);    
    }

    // Initializing the I2C Peripheral
    {
        // Enable I2C2 in RCC
        RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
        
        // Set 100kHz standard-mode
        I2C2->TIMINGR |= (1 << 28); // Prescaler = 1
        I2C2->TIMINGR &= ~((0xFU - 1) << 28);
        
        I2C2->TIMINGR |= (0x13 << 0); // SCL low period = 0x13
        I2C2->TIMINGR &= ~((0xFF - 0x13) << 0);
        
        I2C2->TIMINGR |= (0xF << 8); // SCL high period = 0xF
        I2C2->TIMINGR &= ~((0xFF - 0xF) << 8);
        
        I2C2->TIMINGR |= (2 << 16); // Data hold time = 0x2
        I2C2->TIMINGR &= ~((0xF - 2) << 16);
        
        I2C2->TIMINGR |= (4 << 20); // Data setup time = 0x4
        I2C2->TIMINGR &= ~((0xF - 2) << 20);
        
        // Enable I2C peripheral
        I2C2->CR1 |= (1 << 0);
    }

    // Write to CTRL_REG
    {
        // Set transaction parameters for write
        I2C2->CR2 = 0;
        I2C2->CR2 |= (0x6B << 1); // Slave address = 0x6B
        //I2C2->CR2 &= ~((0x7F - 0x6B) << 1);
        
        I2C2->CR2 |= (2 << 16); // Number of bytes = 2        
        //I2C2->CR2 &= ~((0xFF - 2) << 16);
        
        I2C2->CR2 &= ~(1 << 10); // Transfer direction = write
        
        I2C2->CR2 |= (1 << 13); // Start generation

        // Wait until transmit ready
        while(1)
        {
            // Continue if TXIS set
            if((I2C2->ISR >> 1) & 1)
            {
                break;
            }
            
            // Indicate if NACKF set
            if((I2C2->ISR >> 4) & 1)
            {
                while(1)
                {
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
                    HAL_Delay(2000);                    
                }
            }
        }
        
        // Write CTRL_REG1 address to I2C transmit register
        I2C2->TXDR = 0x20;
    }
    
    // Write value to CTRL_REG1 register
    {
        // Wait until transmit ready
        while(1)
        {
            // Continue if TXIS set
            if((I2C2->ISR >> 1) & 1)
            {
                break;
            }
            
            // Indicate if NACKF set
            if((I2C2->ISR >> 4) & 1)
            {
                while(1)
                {
                    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
                    HAL_Delay(2000);                    
                }
            }
        }
        
        I2C2->TXDR = 0x0B; //0b00001011;
        
        // Wait until transfer complete
        while(!((I2C2->ISR >> 6) & 1)) {}
    }
		
		
}

void I2C(void)
{
    // Repeatedly read gyroscope registers
    {
        unsigned out_x_low = 0;
        unsigned out_x_high = 0;
        unsigned out_y_low = 0;
        unsigned out_y_high = 0;
        
        // Setup for write to OUT_X
        {
            // Set transaction parameters for write
            I2C2->CR2 |= (0x6B << 1); // Slave address = 0x6B
            I2C2->CR2 &= ~((0x7F - 0x6B) << 1);
            
            I2C2->CR2 |= (1 << 16); // Number of bytes = 1        
            I2C2->CR2 &= ~((0xFF - 1) << 16);
            
            I2C2->CR2 &= ~(1 << 10); // Transfer direction = write
            
            I2C2->CR2 |= (1 << 13); // Start generation

            // Wait until transmit ready
            while(1)
            {
                // Continue if TXIS set
                if((I2C2->ISR >> 1) & 1)
                {
                    break;
                }
                
                // Indicate if NACKF set
                if((I2C2->ISR >> 4) & 1)
                {
                    while(1)
                    {
                        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
                        HAL_Delay(2000);                    
                    }
                }
            }

            // Write OUT_X address to I2C transmit register
            I2C2->TXDR = 0xA8;
            //I2C2->TXDR &= ~(0xFF - 0xA8);
            
            // Wait until transfer complete
            while(!((I2C2->ISR >> 6) & 1)) {}
        }
            
        // Want to read from OUT_X
        {
            // Set transaction parameters for read
            I2C2->CR2 |= (0x6B << 1); // Slave address = 0x6B
            I2C2->CR2 &= ~((0x7F - 0x6B) << 1);
                
            I2C2->CR2 |= (2 << 16); // Number of bytes = 2
            I2C2->CR2 &= ~((0xFF - 2) << 16);
                
            I2C2->CR2 |= (1 << 10); // Transfer direction = read
                
            I2C2->CR2 |= (1 << 13); // Start generation

            // Wait until transmit ready or not ack
            while(1)
            {
                // Continue if RXNE set
                if((I2C2->ISR >> 2) & 1)
                {
                    break;
                }
                
                // Indicate if NACKF set
                if((I2C2->ISR >> 4) & 1)
                {
                    while(1)
                    {
                        HAL_Delay(2000);
                        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
                    }
                }
            }

            out_x_low = I2C2->RXDR;
            
            // Wait until transfer complete
            //while((I2C2->ISR >> 6) & 1) {}
                
            // Wait until transmit ready or not ack
            while(1)
            {
                // Continue if RXNE set
                if((I2C2->ISR >> 2) & 1)
                {
                    break;
                }
                
                // Indicate if NACKF set
                if((I2C2->ISR >> 4) & 1)
                {
                    while(1)
                    {
                        HAL_Delay(2000);
                        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
                    }
                }
            }
            
            out_x_high = I2C2->RXDR;
            
            // Wait until transfer complete
            while(!((I2C2->ISR >> 6) & 1)) {}
        }

        // Setup for write to OUT_Y
        {
            // Set transaction parameters for write
            I2C2->CR2 |= (0x6B << 1); // Slave address = 0x6B
            I2C2->CR2 &= ~((0x7F - 0x6B) << 1);
            
            I2C2->CR2 |= (1 << 16); // Number of bytes = 1        
            I2C2->CR2 &= ~((0xFF - 1) << 16);
            
            I2C2->CR2 &= ~(1 << 10); // Transfer direction = write
            
            I2C2->CR2 |= (1 << 13); // Start generation

            // Wait until transmit ready
            while(1)
            {
                // Continue if TXIS set
                if((I2C2->ISR >> 1) & 1)
                {
                    break;
                }
                
                // Indicate if NACKF set
                if((I2C2->ISR >> 4) & 1)
                {
                    while(1)
                    {
                        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
                        HAL_Delay(2000);                    
                    }
                }
            }

            // Write OUT_X address to I2C transmit register
            I2C2->TXDR = 0xAA;
            
            // Wait until transfer complete
            while(!((I2C2->ISR >> 6) & 1)) {}
        }
        
        // Want to read from OUT_Y
        {
            // Set transaction parameters for read
            I2C2->CR2 |= (0x6B << 1); // Slave address = 0x6B
            I2C2->CR2 &= ~((0x7F - 0x6B) << 1);
                
            I2C2->CR2 |= (2 << 16); // Number of bytes = 2
            I2C2->CR2 &= ~((0xFF - 2) << 16);
                
            I2C2->CR2 |= (1 << 10); // Transfer direction = read
                
            I2C2->CR2 |= (1 << 13); // Start generation

            // Wait until transmit ready or not ack
            while(1)
            {
                // Continue if RXNE set
                if((I2C2->ISR >> 2) & 1)
                {
                    break;
                }
                
                // Indicate if NACKF set
                if((I2C2->ISR >> 4) & 1)
                {
                    while(1)
                    {
                        HAL_Delay(2000);
                        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
                    }
                }
            }
            
            out_y_low = I2C2->RXDR;
                
            // Wait until transmit ready or not ack
            while(1)
            {
                // Continue if RXNE set
                if((I2C2->ISR >> 2) & 1)
                {
                    break;
                }
                
                // Indicate if NACKF set
                if((I2C2->ISR >> 4) & 1)
                {
                    while(1)
                    {
                        HAL_Delay(2000);
                        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
                    }
                }
            }
            
            out_y_high = I2C2->RXDR;
            
            // Wait until transfer complete
            while(!((I2C2->ISR >> 6) & 1)) {}
        }
				
        // Check orientation
        x = out_x_low | (out_x_high << 8);
        y = out_y_low | (out_y_high << 8);
            
        
        
        if(y >= THRESHOLD)
        {
            GPIOC->BSRR |= GPIO_PIN_7;
            GPIOC->BSRR |= GPIO_PIN_6 << 16;
        }
        if(y <= -THRESHOLD)
        {
            GPIOC->BSRR |= GPIO_PIN_6;
            GPIOC->BSRR |= GPIO_PIN_7 << 16;            
        }
        if(x <= -THRESHOLD)
        {
            GPIOC->BSRR |= GPIO_PIN_9;
            GPIOC->BSRR |= GPIO_PIN_8 << 16;
        }
        if(x >= THRESHOLD)
        {
            GPIOC->BSRR |= GPIO_PIN_8;
            GPIOC->BSRR |= GPIO_PIN_9 << 16;
        }
        
        HAL_Delay(10);
    }
}

void LED_init(void) {
    // Initialize PC8 and PC9 for LED's
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;                                          // Enable peripheral clock to GPIOC
    GPIOC->MODER |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;                  // Set PC8 & PC9 to outputs
    GPIOC->OTYPER &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9);                    // Set to push-pull output type
    GPIOC->OSPEEDR &= ~((GPIO_OSPEEDR_OSPEEDR8_0 | GPIO_OSPEEDR_OSPEEDR8_1) |
                        (GPIO_OSPEEDR_OSPEEDR9_0 | GPIO_OSPEEDR_OSPEEDR9_1));   // Set to low speed
    GPIOC->PUPDR &= ~((GPIO_PUPDR_PUPDR8_0 | GPIO_PUPDR_PUPDR8_1) |
                      (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR9_1));             // Set to no pull-up/down
    GPIOC->ODR &= ~(GPIO_ODR_8 | GPIO_ODR_9);                                   // Shut off LED's
}

// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init(void) {
    /*
     * ENABLE = PB8
     * DIR A = PB4
     * DIR B = PB5
     */

    // Set up a pin for H-bridge PWM output (TIMER 16 CH1)
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    
    GPIOB->MODER |= 1 << 17;
    GPIOB->MODER &= ~(1 << 16); //alternate
    
    GPIOB->AFR[1] &= ~13;
    GPIOB->AFR[1] |= 2;
    // Set up a few GPIO output pins for direction control
    GPIOB->MODER |= (1 << 8) | (1 << 10);
    GPIOB->MODER &= ~((1 << 9) | (1 << 11));
    // Initialize one direction pin to high, the other low
    GPIOB->BSRR |= 1 << 4;
    GPIOB->BSRR |= 1 << 21;

    // Set up PWM timer
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
    TIM16->CR1 = 0;                         // Clear control register

    // Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
    TIM16->CCMR1 = (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
    TIM16->CCER = TIM_CCER_CC1E;           // Enable capture-compare channel 1
    TIM16->PSC = 7;                         // Run timer on 1Mhz
    TIM16->ARR = 50;                        // PWM at 20kHz
    TIM16->CCR1 = 0;                        // Start PWM at 0% duty cycle
        
    TIM16->BDTR |= TIM_BDTR_MOE;  // Set master output enable (only for timers that have complementary outputs)
    
    TIM16->CR1 |= TIM_CR1_CEN;              // Enable timer
}

void motor_init(void)
{
    Kp = 51;     // Set default proportional gain
    Ki = 3;     // Set default integral gain
    Kd = 2;

    pwm_init();
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(uint8_t duty)
{
    if (duty <= 100)
    {
        TIM16->CCR1 = ((uint32_t)duty*TIM16->ARR)/100;  // Use linear transform to produce CCR1 value
    }
}

void PI_update(void)
{
    static int16_t last_error = 0;
    int16_t error;
    // Calculate error signal and write to "error" variable
    //if(y <= -THRESHOLD || y >= THRESHOLD)
    {
        error = y;
    }
		//else
    //    error = 0;

    // Calculate integral portion of PI controller, write to "error_integral" variable
    static int16_t error_integral = 0;
    error_integral += Ki * error;
    // Clamp the value of the integral to a limited positive range
    if (error_integral < -3200)
    {
        error_integral = -3200;
    }
    else if (error_integral > 3200)
    {
        error_integral = 3200;
    }
		
		int16_t error_derivative = error - last_error;
    
    // Calculate proportional portion, add integral and write to "output" variable
    int16_t output = Kp * error + error_integral + Kd * error_derivative;// * error_derivative;
     // Divide the output into the proper range for output adjustment
    output /= 32;
    if (output < 0)
    {
        output = -output;
    }
    if (output > 100)
    {
        output = 100;
    }
		
		pwm_setDutyCycle(output);
          
    if(y >= 0)
    {
        GPIOB->BSRR |= 1 << 5;
        GPIOB->BSRR |= 1 << 20;
    }
    else if(y < 0)
    {
        GPIOB->BSRR |= 1 << 4;
        GPIOB->BSRR |= 1 << 21;
    }
		
    last_error = error;
}

void potentiometer_init(void)
{
    // Enable ADC1 in peripheral
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    														
    // Set ADC to 8-bit resolution (10)
    ADC1->CFGR1 |= (2 << 3);
    ADC1->CFGR1 &= ~((3 - 2) >> 3);
    // Set ADC to continuous conversion mode (1)
    ADC1->CFGR1 |= (1 << 13);
    // Set ADC to  hardware triggers disabled (00)
    ADC1->CFGR1 &= ~(3 << 10);
    
    // Enable pin channel 1 for ADC conversion
    ADC1->CHSELR |= (1 << 0);
    
    // Start ADC self-calibration
    ADC1->CR &= ~(1 << 0);
    ADC1->CR |= (1U << 31);
    // Wait for calibration to complete
    while ((ADC1->CR >> 31) & 1) {}
    // Enable ADC peripheral
    ADC1->CR |= (1 << 0);
    // Wait until ADC ready
    while (!(ADC1->ISR & 1)) {}
    // Start ADC conversion
    ADC1->CR |= (1 << 2);
}

void potentiometer_read(void)
{
    int16_t data_register = ADC1->DR;
    
    if (data_register > 256 / 4)
        GPIOC->BSRR |= GPIO_PIN_6;
    else
        GPIOC->BSRR |= GPIO_PIN_6 << 16;
    
    if (data_register > 256 / 2)
        GPIOC->BSRR |= GPIO_PIN_9;
    else
        GPIOC->BSRR |= GPIO_PIN_7 << 16;
    
    if (data_register > 256 / 4 * 3)
        GPIOC->BSRR |= GPIO_PIN_7;
    else
        GPIOC->BSRR |= GPIO_PIN_8 << 16;
    
    if (data_register > 256 - 6)
        GPIOC->BSRR |= GPIO_PIN_8;
    else
        GPIOC->BSRR |= GPIO_PIN_9 << 16;
}

int main(void)
{
    HAL_Init();  // Reset of all peripherals, Initializes the Flash interface and the Systick. 
    SystemClock_Config();  // Configure the system clock 

    //Init peripherals, blinks PC9 LED in loop as heartbeat.
    LED_init();                             // Initialize LED's
    motor_init();                           // Initialize motor code 
    //potentiometer_init();
    I2C_init();
    //pwm_setDutyCycle(100);
    while(1)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
        I2C();
        //potentiometer_read();
			  PI_update();
    }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

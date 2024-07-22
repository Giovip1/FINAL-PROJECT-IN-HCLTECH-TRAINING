#include <stm32f401xe.h>
#include <stdio.h>

#define SYS_CLK 16000000
#define PCLK SYS_CLK
#define Baudrate 9600

#define LED1_ON (GPIOC->ODR &= ~(1 << 6))
#define LED1_OFF (GPIOC->ODR |= (1 << 6))
#define LED2_OFF (Set_frq_duty_cycle_TIM1(10000, 0))
#define LED2_ON_10 (Set_frq_duty_cycle_TIM1(10000, 10))
#define LED2_ON_90 (Set_frq_duty_cycle_TIM1(10000, 90))
#define LED3_ON (GPIOB->ODR &= ~(1 << 14))
#define LED3_OFF (GPIOB->ODR |= (1 << 14))
#define LED4_ON (GPIOB->ODR &= ~(1 << 15))
#define LED4_OFF (GPIOB->ODR |= (1 << 15))
#define Buzzer_ON (GPIOC->ODR |= (1 << 9));
#define Buzzer_OFF (GPIOC->ODR &= ~(1 << 9));
#define LED3_toggle (GPIOB->ODR ^= (1 << 14));
#define LED4_toggle (GPIOB->ODR ^= (1 << 15));
#define Buzzer_toggle (GPIOC->ODR ^= (1 << 9));

typedef enum
{
    OFF = 0,
    ON = 1
} ignition_state;

ignition_state ignition = OFF;
int count1 = 0;
int count2 = 0;
int count3 = 0;
int count4 = 0;
int flag_right = 0;
int flag_left = 0;
int flag_parking = 0;
char buffer[30];

void gpio_setup()
{
    // Enable clocks for GPIOA, GPIOB, and GPIOC
    RCC->AHB1ENR = (1 << 0) | (1 << 1) | (1 << 2);

    // Setup GPIO for LED1 PC6 as OUTPUT
    GPIOC->MODER &= ~((1 << 13));
    GPIOC->MODER |= (1 << 12);
    // Setup GPIO for LED1 PC9 as OUTPUT
    GPIOC->MODER &= ~((1 << 19));
    GPIOC->MODER |= (1 << 18);

    // Setup GPIO for LED2-3-4 PB13 PB14 PB15 as OUTPUT
    GPIOB->MODER &= ~((1 << 27) | (1 << 29) | (1 << 31));
    GPIOB->MODER |= (1 << 26) | (1 << 28) | (1 << 30);

    // Setup GPIO for PB7 - PB3 - PB4 as INPUT with Pull-up
    GPIOB->MODER &= ~((1 << 14) | (1 << 15) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9));
    GPIOB->PUPDR |= (1 << 14) | (1 << 6) | (1 << 8);
    GPIOB->PUPDR &= ~((1 << 15) | (1 << 7) | (1 << 9));

    // Setup GPIO for PA15 as INPUT
    GPIOA->MODER &= ~((1 << 30) | (1 << 31));
    GPIOA->PUPDR |= (1 << 30);
    GPIOA->PUPDR &= ~(1 << 31);

    // Initial state of LEDs
    GPIOB->ODR |= (1 << 13) | (1 << 14) | (1 << 15);
    GPIOC->ODR |= (1 << 6);
}
void TIM1_PWM_Init()
{
    // Configure PB13 for TIM1_CH1N
    RCC->AHB1ENR |= (1 << 1);  // Enable clock for GPIOB
    GPIOB->MODER |= (1 << 27); // Select alternate function mode
    GPIOB->MODER &= ~(1 << 26);
    GPIOB->AFR[1] |= (1 << 20); // Set alternate function 1 (AF1)
    // Configure Timer 1 Channel 1
    RCC->APB2ENR |= (1 << 0); // Enable clock for TIMER1
    TIM1->PSC = 0;            // Do not divide the input frequency
    TIM1->ARR = 1000 - 1;     // Load some default value
    TIM1->CCMR1 |= (1 << 3);
    TIM1->CCMR1 |= (1 << 5) | (1 << 6); // Select PWM1 mode
    TIM1->CCMR1 &= ~(1 << 4);           // Select PWM1 mode
    TIM1->CCER |= (1 << 0);             // Enable alternate function pin
    // Configure output (CH1N)
    TIM1->CCMR1 |= (1 << 2); // PWM mode 1 (OC1M = 110) for CH1N
    TIM1->CCER |= (1 << 2);  // Enable complementary output (CC1NE)
    TIM1->BDTR |= (1 << 15); // MOE (Main Output Enable) bit
    TIM1->CNT = 0;           // Timer start count from 0
    TIM1->CR1 |= (1 << 0);   // Enable counter
}

void Set_frq_duty_cycle_TIM1(unsigned long int frequency, unsigned int duty)
{
    TIM1->ARR = ((16000000 / frequency) - 1);
    TIM1->CCR1 = (duty * (TIM1->ARR + 1)) / 100; // CCR1 because we use channel 1
}
void DelayMs(unsigned int count)
{
	TIM3->ARR = (count) - 1;		// set the value for timer
	while(!(TIM3->SR & (1<<0)));	//check if the timer reach the value or not
	TIM3->SR &= ~(1<<0);			//reset the flag
}

void Timer_init()
{
	RCC->APB1ENR |= (1<<1);
	TIM3->PSC = 16000-1;	//Timer count from 0 to 16000
	TIM3->CNT = 0; 			//Timer start count from 0
	TIM3->CR1 |= (1<<0); 	//enable counter
}
/* External interrupt button 1 as PB7 */
void Ext_init_PB7()
{
    __disable_irq();                // Disable global interrupt
    RCC->APB2ENR |= (1 << 14);      // Enable System configuration control clock
    SYSCFG->EXTICR[1] |= (1 << 12); // Select PB7 as External Interrupt pin
    EXTI->IMR |= (1 << 7);          // Unmask on line PB7
    EXTI->FTSR |= (1 << 7);         // Falling edge triggered on line PB7
    NVIC_EnableIRQ(EXTI9_5_IRQn);   // Enable EXTI7
    __enable_irq();                 // Enable global interrupt
}

void EXTI9_5_IRQHandler(void)
{
    if ((((EXTI->PR) >> 7) & 1) != 0)
    {
        count1++;
    }
    EXTI->PR |= (1 << 7);
}

void button_1(void)
{
    if (count1 == 1)
    {
        ignition = ON;
        LED1_ON;
    }
    else if (count1 == 2)
    {
        ignition = OFF;
        count1 = 0;
        count2 = 0;
        count3 = 0;
        count4 = 0;
        flag_right = 0;
        flag_left = 0;
        flag_parking = 0;
        LED1_OFF;
        LED2_OFF;
        LED3_OFF;
        LED4_OFF;
        Buzzer_OFF;
    }
}
/* External interrupt button 2 as PB3 */
void Ext_init_PB3()
{
    __disable_irq();                // Disable global interrupt
    RCC->APB2ENR |= (1 << 14);      // Enable System configuration control clock
    SYSCFG->EXTICR[0] |= (1 << 12); // Select PB3 as External Interrupt pin
    EXTI->IMR |= (1 << 3);          // Unmask on line PB3
    EXTI->FTSR |= (1 << 3);         // Falling edge triggered on line PB3
    NVIC_EnableIRQ(EXTI3_IRQn);     // Enable EXTI3
    __enable_irq();                 // Enable global interrupt
}

void EXTI3_IRQHandler(void)
{
    if ((((EXTI->PR) >> 3) & 1) != 0)
    {
        if (ignition == ON)
        {
            count2++;
        }
    }
    EXTI->PR |= (1 << 3);
}
void button_2()
{
    if (count2 == 1)
    {
        LED2_ON_10; // 10% duty cycle
    }
    else if (count2 == 2)
    {
        LED2_ON_90; // 90% duty cycle
    }
    else if (count2 == 3)
    {
        flag_parking = 1;
    }
    else if (count2 == 4)
    {
        LED2_OFF;
        LED3_OFF;
        LED4_OFF;
        Buzzer_OFF;
        count2 = 0;
        flag_parking = 0;
    }
}

/* External interrupt button 3 as PB4 */
void Ext_init_PB4()
{
    __disable_irq();               // Disable global interrupt
    RCC->APB2ENR |= (1 << 14);     // Enable System configuration control clock
    SYSCFG->EXTICR[1] |= (1 << 0); // Select PB4 as External Interrupt pin
    EXTI->IMR |= (1 << 4);         // Unmask on line PB4
    EXTI->FTSR |= (1 << 4);        // Falling edge triggered on line PB4
    NVIC_EnableIRQ(EXTI4_IRQn);    // Enable EXTI4
    __enable_irq();                // Enable global interrupt
}

void EXTI4_IRQHandler(void)
{
    if ((((EXTI->PR) >> 4) & 1) != 0)
    {
        if (ignition == ON && flag_parking != 1 && flag_left != 1)
        {
            count3++;
        }
    }
    EXTI->PR |= (1 << 4);
}

void button_3(void)
{
    if (count3 == 1)
    {
        flag_right = 1;
        LED4_OFF;
    }
    else if (count3 == 2)
    {
        LED3_OFF;
        Buzzer_OFF;
        count3 = 0;
        flag_right = 0;
    }
}
/* External interrupt button 4 as PA15 */
void Ext_init_PA15()
{
    __disable_irq();                // Disable global interrupt
    RCC->APB2ENR |= (1 << 14);      // Enable System configuration control clock
    SYSCFG->EXTICR[3] |= (0 << 15); // Select PA15 as External Interrupt pin
    EXTI->IMR |= (1 << 15);         // Unmask on line PA15
    EXTI->FTSR |= (1 << 15);        // Falling edge triggered on line PA15
    NVIC_EnableIRQ(EXTI15_10_IRQn); // Enable EXTI15
    __enable_irq();                 // Enable global interrupt
}

void EXTI15_10_IRQHandler(void)
{
    if ((((EXTI->PR) >> 15) & 1) != 0)
    {
        if (ignition == ON && flag_parking != 1 && flag_right != 1)
        {
            count4++;
        }
    }
    EXTI->PR |= (1 << 15);
}
void button_4(void)
{
    if (count4 == 1)
    {
        flag_left = 1;
        LED3_OFF;
    }
    else if (count4 == 2)
    {
    	LED4_OFF;
    	Buzzer_OFF;
        count4 = 0;
        flag_left = 0;
    }
}
void init_uart1()
{
    RCC->APB2ENR |= (1 << 4);
    GPIOA->MODER |= (1 << 19); // alternate PA9
    GPIOA->MODER &= ~(1 << 18);
    GPIOA->MODER |= (1 << 21); // alternate PA10
    GPIOA->MODER &= ~(1 << 20);
    GPIOA->AFR[1] |= (0x7 << 4); // set AF7-TX for PA9
    USART1->CR1 |= (1 << 3);     // USART enable transmitter
    USART1->CR1 |= (1 << 2);     // USART enable receiver
    USART1->BRR = (PCLK + (Baudrate / 2)) / Baudrate;
    USART1->CR1 |= (1 << 13); // enable UART
}
void send_char(unsigned char ch)
{
    while (!(USART1->SR & (1 << 7)));
    USART1->DR = ch;
}
void send_string(char *str)
{
    while (*str)
    {
        send_char(*str);
        str++;
    }
}
void adc_init()
{
    RCC->APB2ENR |= (1 << 8);
    // configure AB1 as ADC input
    GPIOC->MODER |= (1 << 4) | (1 << 5);
    ADC1->SQR1 &= ~(0xF << 20); // sequence length set to 1.
    ADC1->SQR3 = (12 << 0);     // set channel as sequence 1
    ADC1->CR2 |= (1 << 1);      // contineus conversion
    ADC1->CR2 |= (1 << 0);      // enanle ADC
    ADC1->CR2 |= (1 << 30);     // start_conversion
}

int main()
{
    int adc_data = 0;
    int per = 0;
    gpio_setup();
    TIM1_PWM_Init();
    Ext_init_PB7();
    Ext_init_PB3();
    Ext_init_PB4();
    Ext_init_PA15();
    init_uart1();
    adc_init();
    Timer_init();
    while (1)
    {
        button_1();
    	button_2();
    	button_3();
    	button_4();
        if (ignition == ON)
        {
            ADC1->CR2 |= (1 << 30);
            while (!(ADC1->SR & (1 << 1)));
            adc_data = (ADC1->DR & 0xFFF);
            per = (adc_data * 100 / 4096);
            sprintf(buffer, "Fuel: %d %c\r\n", per, '%');
            send_string(buffer);
            sprintf(buffer, "\nON SYSTEM\r");
            send_string(buffer);
            if (count2 == 3 && flag_right != 1 && flag_left != 1)
            {
            	LED3_toggle;
                LED4_toggle;
                //DelayMs(500); //2hz
                Buzzer_toggle;
            }
            if (count3 == 1)
            {
                LED3_toggle;
                //DelayMs(2000); //0.5hz
                Buzzer_toggle;
            }
            if (count4 == 1)
            {
                LED4_toggle;
                //DelayMs(2000); //0.5hz
                Buzzer_toggle;
            }
        }
        else
        {
            sprintf(buffer, "OFF SYSTEM\r");
            send_string(buffer);
        }
    }
}

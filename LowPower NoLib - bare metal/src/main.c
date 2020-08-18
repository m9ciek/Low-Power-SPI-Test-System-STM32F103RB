#include "stm32f103xb.h"

enum { SRAM_BB_REGION_START = 0x20000000 };
enum { SRAM_BB_REGION_END = 0x200fffff };
enum { SRAM_BB_ALIAS = 0x22000000 };
enum { PERIPH_BB_REGION_START = 0x40000000 };
enum { PERIPH_BB_REGION_END = 0x400fffff };
enum { PERIPH_BB_ALIAS = 0x42000000 };
#define SRAM_ADR_COND(adres) ( (uint32_t)&adres >= SRAM_BB_REGION_START && (uint32_t)&adres <=SRAM_BB_REGION_END )
#define PERIPH_ADR_COND(adres) ( (uint32_t)&adres >= PERIPH_BB_REGION_START &&(uint32_t)&adres <= PERIPH_BB_REGION_END )
#define BB_SRAM2(adres, bit) ( SRAM_BB_ALIAS + ((uint32_t)&adres -SRAM_BB_REGION_START)*32u + (uint32_t)(bit*4u) )
#define BB_PERIPH(adres, bit) ( PERIPH_BB_ALIAS + ((uint32_t)&adres -PERIPH_BB_REGION_START)*32u + (uint32_t)(__builtin_ctz(bit))*4u)
/* bit - bit mask, not bit position! */
#define BB(adres, bit) *(__IO uint32_t *)( SRAM_ADR_COND(adres) ? BB_SRAM2(adres, bit) : ( PERIPH_ADR_COND(adres) ? BB_PERIPH(adres, bit) : 0 ))
#define BB_SRAM(adres, bit) *(__IO uint32_t *)BB_SRAM2(adres, bit)

typedef enum {
 /* Push-Pull */
 gpio_mode_output_PP_2MHz = 2,
 gpio_mode_output_PP_10MHz = 1,
 gpio_mode_output_PP_50MHz = 3,
 /* Open-Drain */
 gpio_mode_output_OD_2MHz = 6,
 gpio_mode_output_OD_10MHz = 5,
 gpio_mode_output_OD_50MHz = 7,
 /* Push-Pull */
 gpio_mode_alternate_PP_2MHz = 10,
 gpio_mode_alternate_PP_10MHz = 9,
 gpio_mode_alternate_PP_50MHz = 11,
 /* Open-Drain */
 gpio_mode_alternate_OD_2MHz = 14,
 gpio_mode_alternate_OD_10MHz = 13,
 gpio_mode_alternate_OD_50MHz = 15,
 /* Analog input (ADC) */
 gpio_mode_input_analog = 0,
 /* Floating digital input. */
 gpio_mode_input_floating = 4,
 /* Digital input with pull-up/down (depending on the ODR reg.). */
 gpio_mode_input_pull = 8
} GpioMode_t;

typedef enum {
 PA0 = 0x00000001,
 PA1 = 0x00000002,
 PA2 = 0x00000004,
 PA3 = 0x00000008,
 PA4 = 0x00000010,
 PA5 = 0x00000020,
 PA6 = 0x00000040,
 PA7 = 0x00000080,
 PA8 = 0x00000100,
 PA9 = 0x00000200,
 PA10 = 0x00000400,
 PA11 = 0x00000800,
 PA12 = 0x00001000,
 PA13 = 0x00002000,
 PA14 = 0x00004000,
 PA15 = 0x00008000,
 PB0 = 0x00000001,
 PB15 = 0x00008000
} GpioPin_t;

void gpio_pin_cfg(GPIO_TypeDef * const port, GpioPin_t pin, GpioMode_t mode){
 pin = __builtin_ctz(pin)*4;
 uint32_t volatile * cr_reg;
 uint32_t cr_val;
 cr_reg = &port->CRL;
 if (pin > 28){
 pin -= 32;
 cr_reg = &port->CRH;
 }
 cr_val = *cr_reg;
 cr_val &= ~((uint32_t)(0x0f << pin));
 cr_val |= (uint32_t)(mode << pin);
 *cr_reg = cr_val;
}

#define CAP_SET_ADDRESS_POINTER 0x7D //ADRES OZNACZAJ¥CY WYBRANIE MIEJSCA W REJESTRZE
#define CAP_READ 0x7F //ADRES OZNACZAJ¥CY ODCZYT REJESTRU
#define CAP_WRITE 0x7E //ADRES OZNACZAJ¥CY ZAPIS DO REJESTRU
#define CAP_RESET 0x7A //ADRES RESETUJ¥CY URZ¥DZENIE

uint8_t receiveBuffer[5]; //BUFOR ODBIORCZY
uint8_t sendBuffer[5]; //BUFOR NADAWCZY

//POWYZEJ POMOCNICZE FUNKCJE

volatile uint32_t delayVar;

void delay(uint32_t cnt){
	delayVar = cnt;
	while(delayVar);
}

enum { deselect=0, select=1 };
void slaveSelect_ctrl(int state){
	if(state==1){
		GPIOA->ODR &= ~GPIO_ODR_ODR9;
//		BB(GPIOA->ODR, PA9) = 0;
	}
	else {
		while(SPI1->SR & SPI_SR_BSY);
		GPIOA->ODR |= GPIO_ODR_ODR9;
//		BB(GPIOA->ODR, PA9) = 1;
	}
}

uint16_t spi_rw(uint16_t data){
	while( !(SPI1->SR & SPI_SR_TXE) );
	SPI1->DR = data;
	while( !(SPI1->SR & SPI_SR_RXNE) );
	data = SPI1->DR;
	return data;
}

uint8_t receiveBuffer[5]; //BUFOR ODBIORCZY
uint8_t sendBuffer[5]; //BUFOR NADAWCZY

int main(void) {
	RCC->CR |= RCC_CR_HSEON;
	RCC->CFGR = RCC_CFGR_PLLMULL8 | RCC_CFGR_PLLSRC | RCC_CFGR_ADCPRE_DIV6 |
			RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_USBPRE;
	while (!(RCC->CR & RCC_CR_HSERDY));
	RCC->CR |= RCC_CR_PLLON;
	FLASH->ACR |= FLASH_ACR_LATENCY_1;
	while (!(RCC->CR & RCC_CR_PLLRDY));
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ( (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
	RCC->CR &= ~RCC_CR_HSION;


	RCC->APB2ENR = RCC_APB2ENR_IOPAEN | RCC_APB2ENR_SPI1EN;

	gpio_pin_cfg(GPIOA, PA9, gpio_mode_output_PP_10MHz); /* GPIO */
	gpio_pin_cfg(GPIOA, PA8, gpio_mode_output_PP_10MHz); /* RESET */
	gpio_pin_cfg(GPIOA, PA5, gpio_mode_alternate_PP_10MHz); /* SCK */
	gpio_pin_cfg(GPIOA, PA6, gpio_mode_input_pull); /* MISO */
	gpio_pin_cfg(GPIOA, PA7, gpio_mode_alternate_PP_10MHz); /* MOSI */

	SysTick_Config(1000000/10);


	SPI1->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE | SPI_CR1_MSTR | SPI_CR1_BR_0 | SPI_CR1_BR_1;

	GPIOA->ODR |= GPIO_ODR_ODR8;
	delay(100);
	GPIOA->ODR &= ~GPIO_ODR_ODR8;

	GPIOA->ODR |= GPIO_ODR_ODR9;


	sendBuffer[0]=CAP_SET_ADDRESS_POINTER;
	sendBuffer[1]=0x03;
	sendBuffer[2]=CAP_READ;
	sendBuffer[3]=0xff;
	uint32_t i = 0;
	slaveSelect_ctrl(select);
		 for(i=0;i<4;i++){
			 receiveBuffer[i] = spi_rw( sendBuffer[i] );
		 }
		 slaveSelect_ctrl(deselect);

	uint8_t sleepBuffer[4];
	sleepBuffer[0]=CAP_SET_ADDRESS_POINTER;
	sleepBuffer[1]=0x00;
	sleepBuffer[2]=CAP_WRITE;
	sleepBuffer[3]=0x10;

	PWR->CR |= 0x00000001;		//uruchomienie regulatora niskiej mocy
	SysTick->CTRL = 0x00000004;	//wy³¹czenie przerwania SysTick
	SCB->SCR = 0x00000000;		//uruchomienie trybu Sleep
	__WFI();					//oczekiwanie na przerwanie

	while(1){
	 slaveSelect_ctrl(select);
	 for(i=0;i<4;i++){
		 receiveBuffer[i] = spi_rw( sendBuffer[i] );
	 }
	 slaveSelect_ctrl(deselect);
	 delay(70);
	 }

}

__attribute__((interrupt)) void SysTick_Handler(void){
 if(delayVar) --delayVar;
}



#include "stm32f10x.h"

/* ВАЖНО: проверьте соответствие вашим проводам.
   В этом коде PB9 = CLK, PB8 = DIO (как в рабочем примере [citation:12]).
   Если у вас наоборот — просто поменяйте значения. */
#define TM_CLK_PIN   8
#define TM_DIO_PIN   9

/* Задержка с барьером от оптимизации */
static void delay_us(uint32_t us) {
    volatile uint32_t count = us * 8;
    while (count--) __asm("nop");
}

static void delay_ms(uint32_t ms) {
    while (ms--) delay_us(1000);
}

/* Управление линиями */
static void clk_high(void) { GPIOB->BSRR = (1U << TM_CLK_PIN); }
static void clk_low(void)  { GPIOB->BRR  = (1U << TM_CLK_PIN); }

/* Для DIO используем переключение выход/вход, чтобы не зависеть от подтяжки */
static void dio_output(void) {
    // PB8 или PB9 (в зависимости от макроса) — Output Open-Drain 50MHz
    uint32_t shift = (TM_DIO_PIN - 8) * 4;
    GPIOB->CRH &= ~(0xFU << shift);
    GPIOB->CRH |=  (0x7U << shift);
}
static void dio_input(void) {
    uint32_t shift = (TM_DIO_PIN - 8) * 4;
    GPIOB->CRH &= ~(0xFU << shift);
    GPIOB->CRH |=  (0x4U << shift); // Input floating
}
static void dio_high(void) { dio_input(); }  // Отпускаем линию (надеемся на внешнюю подтяжку)
static void dio_low(void)  { 
    dio_output(); 
    GPIOB->BRR = (1U << TM_DIO_PIN); 
}

/* Инициализация */
static void tm_gpio_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    
    // Отключаем I2C1, если он был включён
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    I2C1->CR1 &= ~I2C_CR1_PE;

    // Настройка CLK как Open-Drain Output
    uint32_t clk_shift = (TM_CLK_PIN - 8) * 4;
    GPIOB->CRH &= ~(0xFU << clk_shift);
    GPIOB->CRH |=  (0x7U << clk_shift);

    // Настройка DIO (изначально вход, чтобы не мешать)
    dio_input();

    // Устанавливаем высокий уровень на CLK
    GPIOB->BSRR = (1U << TM_CLK_PIN);
}

/* Протокол TM1637 */
static void tm_start(void) {
    dio_high();
    clk_high();
    delay_us(5);
    dio_low();
    delay_us(5);
    clk_low();
    delay_us(5);
}

static void tm_stop(void) {
    clk_low();
    dio_low();
    delay_us(5);
    clk_high();
    delay_us(5);
    dio_high();
    delay_us(5);
}

static void tm_write_byte(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        clk_low();
        delay_us(2);
        
        if (data & 0x01) dio_high();
        else             dio_low();
        
        delay_us(2);
        clk_high();
        delay_us(5);
        data >>= 1; // LSB first
    }
    // ACK
    clk_low();
    dio_input(); // Отпускаем DIO для чтения ACK
    delay_us(2);
    clk_high();
    delay_us(5);
    clk_low();
    dio_output(); // Возвращаем управление
}

static void tm_write_cmd(uint8_t cmd) {
    tm_start();
    tm_write_byte(cmd);
    tm_stop();
}

static void tm_write_data(uint8_t addr, uint8_t *data, uint8_t len) {
    tm_start();
    tm_write_byte(addr);
    for (uint8_t i = 0; i < len; i++) tm_write_byte(data[i]);
    tm_stop();
}

static const uint8_t seg_code[] = {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};

static void tm_display_number(uint16_t num) {
    uint8_t buf[4];
    if (num > 9999) num = 9999;
    buf[0] = seg_code[(num / 1000) % 10];
    buf[1] = seg_code[(num / 100)  % 10];
    buf[2] = seg_code[(num / 10)   % 10];
    buf[3] = seg_code[num % 10];

    tm_write_cmd(0x40); // Автоинкремент адреса
    tm_write_data(0xC0, buf, 4);
    tm_write_cmd(0x8F); // Вкл. дисплей, макс. яркость - менять тут, 0x88 минимум, 0x8E - примерно средне, 0x8F - максимум
}

int main(void) {
    tm_gpio_init();
    delay_ms(100);

    tm_write_cmd(0x8F); // Попытка включить дисплей и менять также и тут яркость

    uint16_t counter = 0;
    while (1) {
        tm_display_number(counter);
        counter++;
        if (counter > 9999) counter = 0;
        delay_ms(10);
    }
}

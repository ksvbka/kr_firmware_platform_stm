/*
* @Author: Trung Kien
* @Date:   2016-11-30 22:27:14
* @Last Modified by:   Kienltb
* @Last Modified time: 2017-01-10 15:07:04
*/
#include "lcd_5110.h"
#include "system.h" /* use for delay_ms*/

#define CMD_MODE  0      //When DC=0 bits comes to command
#define DATA_MODE 1      //When DC=1 bits comes to data

/*Helper function*/

/* Write 8bit of tx_buff by SPI bit-banging*/
static void lcd_shift_out(uint8_t tx_buff)
{
        uint8_t i;
        gpio_clear(LCD_CLK);
        gpio_set(LCD_DIN);
        for (i = 8; i > 0; --i) {
                if ((tx_buff & BIT7) == BIT7)
                        gpio_set(LCD_DIN);
                else
                        gpio_clear(LCD_DIN);

                tx_buff <<= 1;
                gpio_set(LCD_CLK);
                delay_ms(10);
                gpio_clear(LCD_CLK);
        }
}

/* Write command/data to LCD*/
static void lcd_write(uint8_t mode, uint8_t tx_buff)
{
        if (mode == CMD_MODE)
                gpio_clear(LCD_DC);
        else
                gpio_set(LCD_DC);

        gpio_clear(LCD_CE);
        lcd_shift_out(tx_buff);
        gpio_set(LCD_CE);
}

void lcd_init(uint8_t contrast, uint8_t coefficient, uint8_t bias)
{
        /* Config IO pin for LCD*/
        gpio_init(LCD_CE,  GPIO_OUT);
        gpio_init(LCD_RST, GPIO_OUT);
        gpio_init(LCD_DC,  GPIO_OUT);
        gpio_init(LCD_CLK, GPIO_OUT);
        gpio_init(LCD_DIN, GPIO_OUT);

        /*clk init as high*/
        // gpio_set(LCD_CLK);

        /* Reset lcd*/
        gpio_clear(LCD_RST);
        delay_ms(500);
        gpio_set(LCD_RST);

        /* Config ambiance*/
        lcd_write(CMD_MODE, 0x21);
        lcd_write(CMD_MODE, 0x80 + contrast);
        lcd_write(CMD_MODE, 0x04 + coefficient);
        lcd_write(CMD_MODE, 0x10 + bias);
        lcd_write(CMD_MODE, 0x20);
        lcd_write(CMD_MODE, 0x0C);
}

void lcd_goto(uint8_t x, uint8_t y)
{
        lcd_write(CMD_MODE, 0x80 | x);
        lcd_write(CMD_MODE, 0x40 | y);
}

void lcd_clear()
{
        uint16_t i;
        for (i = 0; i < 504; i++) {     /* 6row x 84col = 504*/
                lcd_write(DATA_MODE, 0x00);
        }
        lcd_goto(0, 0);
}

void lcd_bitmap(const uint8_t * bitmap)
{
        uint16_t i;
        for (i = 0; i < 504; i++) {     /* 6row x 84col = 504*/
                lcd_write(DATA_MODE, bitmap[i]);
        }
}

void lcd_invert_enable()
{
        lcd_write(CMD_MODE, 0x07);
}

void lcd_invert_disable()
{
        lcd_write(CMD_MODE, 0x06);
}

// #ifdef LCD_TEXT
void lcd_printf_char(char c)
{
        uint8_t i;
        lcd_write(DATA_MODE, 0x00);
        for (i = 0; i < 5; i++)
                lcd_write(DATA_MODE, ASCII[c - 0x20][i]);
        lcd_write(DATA_MODE, 0x00);
}

void lcd_printf(const char* msg)
{
        char *ptr_c = msg;
        while (ptr_c != '\0')
                lcd_printf_char(*ptr_c++);
}
// #endif //LCD_TEXT

#ifdef LCD_GRAPHIC

#define FRAME_ROW       (LCD_ROW - 5*(GR_LEFT_BOUND + GR_LEFT_BOUND))
#define FRAME_COL       (LCD_COL - 1 /*title*/ - 1 /*footprint*/)

#define FRAME_SIZE      (FRAME_ROW * FRAME_COL)

#define X_MIN           (GR_LEFT_BOUND * 5)
#define X_MAX           (LCD_X_SIZE - (GR_LEFT_BOUND + GR_LEFT_BOUND) *5 )

#define Y_MIN           (GR_TITLE )
#define Y_MAX           (LCD_Y_SIZE - GR_TITLE - GR_FOOTPRINT)

/* TODO: MSP430 don't enought RAM for framebuffer, fix this isssue !!! */
static char frame_buff[FRAME_SIZE];

static void lcd_render()
{
        lcd_bitmap(frame_buff);
}

void lcd_draw_point(struct point p)
{
        uint8_t p_row = p.y / 8;
        uint8_t p_bit_of_row = p.y % 8;

        /*Set pixel*/
        frame_buff[ p_row * 74 + p.x] |= (1 << p_bit_of_row);
        lcd_render();
}

void lcd_clear_point(struct point p)
{
        uint8_t p_row = p.y / 8;
        uint8_t p_bit_of_row = p.y % 8;

        /*clear pixel*/
        frame_buff[ p_row * 84 + p.x] &= ~(1 << p_bit_of_row);
        lcd_render();
}

// void lcd_draw_line(struct point begin, struct point end);
// void lcd_draw_rect(uint8_t height, uint8_t width, struct point base_pos);
// void lcd_draw_circle(uint8_t radius, struct point base_pos);

// void lcd_clear_point(struct point p);
// void lcd_clear_line(struct point begin, struct point end);
// void lcd_clear_rect(uint8_t height, uint8_t width, struct point base_pos);
// void lcd_clear_circle(uint8_t radius, struct point base_pos);


#endif // LCD_GRAPIC

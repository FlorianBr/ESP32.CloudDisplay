/**
 ******************************************************************************
 *  file           : lcd.c
 *  brief          : The small I2C-Display
 ******************************************************************************
 *  Copyright (C) 2021 Florian Brandner
 */

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "lcd.h"
#include "font.h"
#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

/* Private typedef -----------------------------------------------------------*/

// The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct.
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; // No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

typedef enum {
    LCD_TYPE_ILI = 1,
    LCD_TYPE_ST,
    LCD_TYPE_MAX,
} type_lcd_t;

/* Private define ------------------------------------------------------------*/

#define LCD_REFRESH 50    //< Refresh-Time in ms
#define PARALLEL_LINES 10 //< Number of Lines to send in one request

#define LCD_X 320
#define LCD_Y 240

#define LCD_HOST HSPI_HOST
#define DMA_CHAN 2

#define PIN_NUM_MISO 12
#define PIN_NUM_MOSI 14
#define PIN_NUM_CLK 33
#define PIN_NUM_CS 26
#define PIN_NUM_DC 27
#define PIN_NUM_RST 25
#define PIN_NUM_BCKL 32

DRAM_ATTR static const lcd_init_cmd_t ili_init_cmds[] = {
    /* Power contorl B, power control = 0, DC_ENA = 1 */
    {0xCF, {0x00, 0x83, 0X30}, 3},
    /* Power on sequence control,
     * cp1 keeps 1 frame, 1st frame enable
     * vcl = 0, ddvdh=3, vgh=1, vgl=2
     * DDVDH_ENH=1
     */
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    /* Driver timing control A,
     * non-overlap=default +1
     * EQ=default - 1, CR=default
     * pre-charge=default - 1
     */
    {0xE8, {0x85, 0x01, 0x79}, 3},
    /* Power control A, Vcore=1.6V, DDVDH=5.6V */
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    /* Pump ratio control, DDVDH=2xVCl */
    {0xF7, {0x20}, 1},
    /* Driver timing control, all=0 unit */
    {0xEA, {0x00, 0x00}, 2},
    /* Power control 1, GVDD=4.75V */
    {0xC0, {0x26}, 1},
    /* Power control 2, DDVDH=VCl*2, VGH=VCl*7, VGL=-VCl*3 */
    {0xC1, {0x11}, 1},
    /* VCOM control 1, VCOMH=4.025V, VCOML=-0.950V */
    {0xC5, {0x35, 0x3E}, 2},
    /* VCOM control 2, VCOMH=VMH-2, VCOML=VML-2 */
    {0xC7, {0xBE}, 1},
    /* Memory access contorl, MX=MY=0, MV=1, ML=0, BGR=1, MH=0 */
//    {0x36, {0x28}, 1},
    {0x36, {0x28 ^ 0xC0}, 1},
    /* Pixel format, 16bits/pixel for RGB/MCU interface */
    {0x3A, {0x55}, 1},
    /* Frame rate control, f=fosc, 70Hz fps */
    {0xB1, {0x00, 0x1B}, 2},
    /* Enable 3G, disabled */
    {0xF2, {0x08}, 1},
    /* Gamma set, curve 1 */
    {0x26, {0x01}, 1},
    /* Positive gamma correction */
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    /* Negative gamma correction */
    {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
    /* Column address set, SC=0, EC=0xEF */
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    /* Page address set, SP=0, EP=0x013F */
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
    /* Memory write */
    {0x2C, {0}, 0},
    /* Entry mode set, Low vol detect disabled, normal display */
    {0xB7, {0x07}, 1},
    /* Display function control */
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    /* Sleep out */
    {0x11, {0}, 0x80},
    /* Display on */
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const char *TAG = "LCD";
static TaskHandle_t lcd_worker_hdl = NULL;
spi_device_handle_t spi;

lcd_rgb_color_t *plcdbuffer = NULL;

/* Private function prototypes -----------------------------------------------*/
static void worker(void *pvParameters);
esp_err_t command(__uint8_t cmd);
static void send_lines(spi_device_handle_t spi, int ypos, uint16_t *linedata);
static void send_line_finish(spi_device_handle_t spi);

/* Private user code ---------------------------------------------------------*/

/* Send a command to the LCD. Uses spi_device_polling_transmit, which waits
 * until the transfer is complete.
 *
 * Since command transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));                   // Zero out the transaction
    t.length = 8;                               // Command is 8 bits
    t.tx_buffer = &cmd;                         // The data is the cmd itself
    t.user = (void *)0;                         // D/C needs to be set to 0
    ret = spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret == ESP_OK);                      // Should have had no issues.
}

/* Send data to the LCD. Uses spi_device_polling_transmit, which waits until the
 * transfer is complete.
 *
 * Since data transactions are usually small, they are handled in polling
 * mode for higher speed. The overhead of interrupt transactions is more than
 * just waiting for the transaction to complete.
 */
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len) {
    esp_err_t ret;
    spi_transaction_t t;
    if (len == 0) return;                       // no need to send anything
    memset(&t, 0, sizeof(t));                   // Zero out the transaction
    t.length = len * 8;                         // Len is in bytes, transaction length is in bits.
    t.tx_buffer = data;                         // Data
    t.user = (void *)1;                         // D/C needs to be set to 1
    ret = spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret == ESP_OK);                      // Should have had no issues.
}

/**
 * @brief Get the LCDs ID
 */
uint32_t lcd_get_id(spi_device_handle_t spi) {
    lcd_cmd(spi, 0x04); // get_id cmd

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * 3;
    t.flags = SPI_TRANS_USE_RXDATA;
    t.user = (void *)1;

    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);

    return *(uint32_t *)t.rx_data;
}

// This function is called (in irq context!) just before a transmission starts.
// It will set the D/C line to the value indicated in the user field.
void lcd_spi_pre_transfer_callback(spi_transaction_t *t) {
    int dc = (int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

/**
 * @brief Convert 8-Bit RGB Values to RG565
 */
__uint16_t rgb2rgb565(uint8_t red, __uint8_t green, __uint8_t blue) {
    __uint16_t RetVal = 0;
    RetVal = (red & 0xF8) << 8;
    RetVal |= ((green & 0xfc) << 3);
    RetVal |= (blue >> 3);

    return __builtin_bswap16(RetVal);
}

/**
 * @brief Adds a char to the position
 */
__uint16_t addchar(__uint16_t Row, __uint16_t Col, char character, lcd_rgb_color_t fgcolor, lcd_rgb_color_t bgcolor) {
    __uint8_t segments = 0;
    __uint8_t charpos = 0;

    charpos = character - ' ';

    segments = 8;

    if (charpos >= sizeof(FontFixedsys16Bitmap) / (FONT_H * sizeof(uint8_t))) {
        ESP_LOGE(TAG, "Invalid char 0x%02x!", character);
        return 0;
    }

    for (__uint8_t x = 0; x < FONT_W; x++) {        // X-Colum 0:7 
        for (__uint8_t y = 0; y < FONT_H; y++) {    // Y-Row 0:15
            // if (((FontFixedsys16Bitmap[charpos][i] >> (7 - Bit)) & 0x01) > 0) // 7-bit: flip vertical

            if (((FontFixedsys16Bitmap[charpos][y] >> (FONT_W - 1 - x)) & 0x01) > 0) {
                *(plcdbuffer + (Col + x + ((Row + y) * LCD_X))) = fgcolor;
            } else {
                *(plcdbuffer + (Col + x + ((Row + y) * LCD_X))) = bgcolor;
            }
        }
    }

    // Append empty column
    for (__uint8_t Bit = 0; Bit < FONT_H; Bit++) {
        *(plcdbuffer + (Col + 8 + ((Row + Bit) * LCD_X))) = bgcolor;
    }

    return (segments + 1);
}

/**
 * @brief Send a string to a LCD line
 */
__uint16_t LCD_WriteString(__uint16_t Row, __uint16_t Col, char *cBuffer, lcd_rgb_color_t fgcolor, lcd_rgb_color_t bgcolor) {
    __uint16_t ColPos = Col;
    char *pChar;

    pChar = cBuffer;
    for (__uint8_t i = 0; i < strlen(cBuffer); i++) {
        ColPos += addchar(Row, ColPos, *pChar++, fgcolor, bgcolor);
    }
    return (ColPos - Col);
}

/**
 * @brief LCD Worker
 */
static void worker(void *pvParameters) {
    uint16_t *lines;

    // Allocate memory for transfer buffer
    lines = heap_caps_malloc(LCD_X * PARALLEL_LINES * sizeof(uint16_t), MALLOC_CAP_DMA);
    assert(lines != NULL);

    ESP_LOGI(TAG, "Starting Worker....");

    // Init buffer with a test pattern
    memset(plcdbuffer, 0, LCD_X * LCD_Y * sizeof(lcd_rgb_color_t));

    for (int y = 0; y < LCD_Y; y++) {
        for (int x = 0; x < LCD_X; x++) {
            (plcdbuffer + ((y * LCD_X) + x))->Red = x * 0xFF / LCD_X;
            (plcdbuffer + ((y * LCD_X) + x))->Green = y * 0xFF / LCD_Y;
            (plcdbuffer + ((y * LCD_X) + x))->Blue = (LCD_Y - y) * 0xFF / LCD_Y;
        }
    }

    while (1) {
        for (int y = 0; y < 240; y += PARALLEL_LINES) {
            // Convert and copy buffer to DMA-Buffer
            for (__uint16_t i = 0; i < LCD_X * PARALLEL_LINES; i++) {
                *(lines + i) =
                    rgb2rgb565(
                        (plcdbuffer + ((LCD_X * LCD_Y) - ((y * LCD_X) + i)))->Red,
                        (plcdbuffer + ((LCD_X * LCD_Y) - ((y * LCD_X) + i)))->Green,
                        (plcdbuffer + ((LCD_X * LCD_Y) - ((y * LCD_X) + i)))->Blue);
            }
            send_lines(spi, y, lines); // Send lines
            send_line_finish(spi);     // Wait for transfer to complete
        }
        vTaskDelay(LCD_REFRESH / portTICK_PERIOD_MS);

    } // while 1
} // worker

/**
 * @brief Init the LCD
 */
esp_err_t LCD_Init(void) {
    esp_err_t err = ESP_OK;

    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 10 * 320 * 2 + 8};

    spi_device_interface_config_t devcfg = {
#ifdef CONFIG_LCD_OVERCLOCK
        .clock_speed_hz = 26 * 1000 * 1000, //Clock out at 26 MHz
#else
        .clock_speed_hz = 10 * 1000 * 1000, //Clock out at 10 MHz
#endif
        .mode = 0,                               //SPI mode 0
        .spics_io_num = PIN_NUM_CS,              //CS pin
        .queue_size = 7,                         //We want to be able to queue 7 transactions at a time
        .pre_cb = lcd_spi_pre_transfer_callback, //Specify pre-transfer callback to handle D/C line
    };
    // Initialize the SPI bus
    err = spi_bus_initialize(LCD_HOST, &buscfg, DMA_CHAN);
    ESP_LOGI(TAG, "SPI Bus init = %d", err);
    ESP_ERROR_CHECK(err);

    // Attach the LCD to the SPI bus
    err = spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    ESP_LOGI(TAG, "SPI Bus Add = %d", err);
    ESP_ERROR_CHECK(err);

    // Initialize the LCD

    int cmd = 0;
    const lcd_init_cmd_t *lcd_init_cmds;

    // Initialize non-SPI GPIOs
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    // Reset the display
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    // Detect LCD type
    uint32_t lcd_id = lcd_get_id(spi);

    ESP_LOGI(TAG, "LCD ID: %08X\n", lcd_id);

    ESP_LOGI(TAG, "LCD ILI9341 initialization.\n");
    lcd_init_cmds = ili_init_cmds;

    //Send all the commands
    while (lcd_init_cmds[cmd].databytes != 0xff) {
        lcd_cmd(spi, lcd_init_cmds[cmd].cmd);
        lcd_data(spi, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes & 0x1F);
        if (lcd_init_cmds[cmd].databytes & 0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

    // Enable backlight
    gpio_set_level(PIN_NUM_BCKL, 1);

    // Allocate buffer memory
    plcdbuffer = malloc(LCD_X * LCD_Y * sizeof(lcd_rgb_color_t));

    ESP_LOGI(TAG, "LCD Init complete");

    return err;
} // lcd_init

esp_err_t LCD_Start(void) {
    ESP_LOGI(TAG, "Starting LCD");
    xTaskCreate(worker, "LCD Worker", 4096, NULL, 12, &lcd_worker_hdl);
    return ESP_OK;
} // lcd_start

/* To send a set of lines we have to send a command, 2 data bytes, another command, 2 more data bytes and another command
 * before sending the line data itself; a total of 6 transactions. (We can't put all of this in just one transaction
 * because the D/C line needs to be toggled in the middle.)
 * This routine queues these commands up as interrupt transactions so they get
 * sent faster (compared to calling spi_device_transmit several times), and at
 * the mean while the lines for next transactions can get calculated.
 */
static void send_lines(spi_device_handle_t spi, int ypos, uint16_t *linedata) {
    esp_err_t ret;
    int x;
    //Transaction descriptors. Declared static so they're not allocated on the stack; we need this memory even when this
    //function is finished because the SPI driver needs access to it even while we're already calculating the next line.
    static spi_transaction_t trans[6];

    //In theory, it's better to initialize trans and data only once and hang on to the initialized
    //variables. We allocate them on the stack, so we need to re-init them each call.
    for (x = 0; x < 6; x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x & 1) == 0) {
            //Even transfers are commands
            trans[x].length = 8;
            trans[x].user = (void *)0;
        } else {
            //Odd transfers are data
            trans[x].length = 8 * 4;
            trans[x].user = (void *)1;
        }
        trans[x].flags = SPI_TRANS_USE_TXDATA;
    }
    trans[0].tx_data[0] = 0x2A;                           //Column Address Set
    trans[1].tx_data[0] = 0;                              //Start Col High
    trans[1].tx_data[1] = 0;                              //Start Col Low
    trans[1].tx_data[2] = (LCD_X) >> 8;                   //End Col High
    trans[1].tx_data[3] = (LCD_X)&0xff;                   //End Col Low
    trans[2].tx_data[0] = 0x2B;                           //Page address set
    trans[3].tx_data[0] = ypos >> 8;                      //Start page high
    trans[3].tx_data[1] = ypos & 0xff;                    //start page low
    trans[3].tx_data[2] = (ypos + PARALLEL_LINES) >> 8;   //end page high
    trans[3].tx_data[3] = (ypos + PARALLEL_LINES) & 0xff; //end page low
    trans[4].tx_data[0] = 0x2C;                           //memory write
    trans[5].tx_buffer = linedata;                        //finally send the line data
    trans[5].length = LCD_X * 2 * 8 * PARALLEL_LINES;     //Data length, in bits
    trans[5].flags = 0;                                   //undo SPI_TRANS_USE_TXDATA flag

    //Queue all transactions.
    for (x = 0; x < 6; x++) {
        ret = spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
        assert(ret == ESP_OK);
    }

    //When we are here, the SPI driver is busy (in the background) getting the transactions sent. That happens
    //mostly using DMA, so the CPU doesn't have much to do here. We're not going to wait for the transaction to
    //finish because we may as well spend the time calculating the next line. When that is done, we can call
    //send_line_finish, which will wait for the transfers to be done and check their status.
}

static void send_line_finish(spi_device_handle_t spi) {
    spi_transaction_t *rtrans;
    esp_err_t ret;
    // Wait for all 6 transactions to be done and get back the results.
    for (int x = 0; x < 6; x++) {
        ret = spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret == ESP_OK);
        // We could inspect rtrans now if we received any info back. The LCD is treated as write-only, though.
    }
}



/**
 * @brief Set LCD Buffer to raw Data 
 * 
 * Data MUST be of correct size, 320x240 x 3 Byte
 */
esp_err_t LCD_SetRawData(__uint8_t * pData, size_t offset, size_t length) {
    if (NULL == plcdbuffer) return ESP_FAIL;
    memcpy(((char*)plcdbuffer)+offset, pData, length);
    return ESP_OK;
}

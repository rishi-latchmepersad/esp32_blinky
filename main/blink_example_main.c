/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "freertos/semphr.h"
#include "esp_attr.h"

static const char *TAG = "example";
SemaphoreHandle_t led_semaphore;
SemaphoreHandle_t button_semaphore;
static uint8_t s_led_state = 0;

typedef struct
{
    uint8_t index;
    char name[32];
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} LEDColour;

static LEDColour led_colours[] = {
    {0, "red", 5, 0, 0},
    {1, "green", 0, 5, 0},
    {2, "blue", 0, 0, 5},
    {3, "yellow", 5, 5, 0},
    {4, "magenta", 5, 0, 5},
    {5, "cyan", 0, 5, 5},
    {6, "white", 5, 5, 5},
};

uint8_t led_colour_index = 0;
/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_white_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

void blink_led()
{
    ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
    /* If the addressable LED is enabled */
    if (s_led_state)
    {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, led_colours[led_colour_index].red, led_colours[led_colour_index].green, led_colours[led_colour_index].blue);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    }
    else
    {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
    /* Toggle the LED state */
    s_led_state = !s_led_state;
}

void blink_led_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Starting the blink led task!");
    while (1)
    {
        xSemaphoreTake(led_semaphore, portMAX_DELAY);
        blink_led();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        blink_led();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        xSemaphoreGive(led_semaphore);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void IRAM_ATTR gpio_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Signal the task
    xSemaphoreGiveFromISR(button_semaphore, &xHigherPriorityTaskWoken);

    // Perform a context switch if needed
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void button_task(void *pvParameter)
{
    while (1)
    {
        if (xSemaphoreTake(button_semaphore, portMAX_DELAY))
        {
            // Debounce
            vTaskDelay(pdMS_TO_TICKS(150));

            // Only accept input if still low (confirmed press)
            if (gpio_get_level(GPIO_NUM_9) == 0)
            {
                ESP_LOGI(TAG, "Button pressed");
                xSemaphoreTake(led_semaphore, portMAX_DELAY);
                led_colour_index = (led_colour_index + 1) % 7;
                ESP_LOGI(TAG, "Changing colour to: %s", led_colours[led_colour_index].name);
                xSemaphoreGive(led_semaphore);
            }
        }
    }
}

void set_up_button()
{
    button_semaphore = xSemaphoreCreateBinary();
    if (button_semaphore == NULL)
    {
        ESP_LOGE(TAG, "Failed to create semaphore for button.");
        return;
    }

    xTaskCreate(button_task, "button_task", 2048, NULL, 6, NULL);
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_9),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE};
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_9, gpio_isr_handler, NULL);
    gpio_intr_enable(GPIO_NUM_9);
    ESP_LOGI(TAG, "Button configured for interrupt!");
}

void app_main(void)
{
    // set up a semaphore to share the one RGB LED
    led_semaphore = xSemaphoreCreateBinary();
    if (led_semaphore == NULL)
    {
        ESP_LOGI(TAG, "Failed to create semaphore");
    }
    xSemaphoreGive(led_semaphore);
    /* Configure the peripheral according to the LED type */
    configure_led();
    xTaskCreate(blink_led_task, "blink_led_task", 2048, NULL, 1, NULL);
    set_up_button();
    /*
     * Print task list for debug */
    //  Buffer to hold task list (large enough to fit output)
    char task_list_buffer[2048];

    // Get and print task list
    printf("Task List:\n");
    printf("Name         State  Priority  Stack  Num\n");
    printf("-----------------------------------------\n");

    vTaskList(task_list_buffer);
    printf("%s\n", task_list_buffer);
}

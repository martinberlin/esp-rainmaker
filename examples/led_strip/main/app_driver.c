/*  LED Lightbulb demo implementation using RGB LED

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <sdkconfig.h>

#include <iot_button.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h> 
#include <esp_rmaker_standard_params.h> 

#include <app_reset.h>
#include <ws2812_led.h>
#include <led_strip_rmt_ws2812.c>
#include "app_priv.h"

#define LED_STRIP_LEN 13
/* This is the button that is used for toggling the power */
#define BUTTON_GPIO          CONFIG_EXAMPLE_BOARD_BUTTON_GPIO
#define BUTTON_ACTIVE_LEVEL  0

#define WIFI_RESET_BUTTON_TIMEOUT       3
#define FACTORY_RESET_BUTTON_TIMEOUT    10

static uint16_t g_hue = DEFAULT_HUE;
static uint16_t g_saturation = DEFAULT_SATURATION;
static uint16_t g_value = DEFAULT_BRIGHTNESS;
static bool g_power = DEFAULT_POWER;

led_strip_t *my_strip;

esp_err_t strip_set_pixels(uint32_t hue, uint32_t saturation, uint32_t value) {
    uint32_t red = 0;
    uint32_t green = 0;
    uint32_t blue = 0;
    ws2812_led_hsv2rgb(hue, saturation, value, &red, &green, &blue);
    
    for (uint16_t index=0; index<LED_STRIP_LEN; ++index) {
        ws2812_set_pixel(my_strip, index, red, green, blue);
        //printf("%d ", index);
    }
    
    return ws2812_refresh(my_strip, LED_STRIP_LEN);
}

esp_err_t app_light_set_led(uint32_t hue, uint32_t saturation, uint32_t brightness)
{
    /* Whenever this function is called, light power will be ON */
    if (!g_power) {
        g_power = true;
        esp_rmaker_param_update_and_report(
                esp_rmaker_device_get_param_by_type(light_device, ESP_RMAKER_PARAM_POWER),
                esp_rmaker_bool(g_power));
    }
    return strip_set_pixels(hue, saturation, brightness);
}

esp_err_t app_light_set_power(bool power)
{
    g_power = power;
    if (power) {
        strip_set_pixels(g_hue, g_saturation, g_value);
    } else {
        ws2812_clear(my_strip, 100);
    }
    return ESP_OK;
}

esp_err_t app_light_set_brightness(uint16_t brightness)
{
    g_value = brightness;
    return app_light_set_led(g_hue, g_saturation, g_value);
}
esp_err_t app_light_set_hue(uint16_t hue)
{
    g_hue = hue;
    return app_light_set_led(g_hue, g_saturation, g_value);
}
esp_err_t app_light_set_saturation(uint16_t saturation)
{
    g_saturation = saturation;
    return app_light_set_led(g_hue, g_saturation, g_value);
}

esp_err_t app_light_init(void)
{
    my_strip = ws2812_led_init(LED_STRIP_LEN);
    
    if (g_power) {
        ws2812_led_set_hsv(g_hue, g_saturation, g_value);
        // strip_set_pixels
    } else {
        ws2812_led_clear();
    }
    return ESP_OK;
}

static void push_btn_cb(void *arg)
{
    app_light_set_power(!g_power);
    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(light_device, ESP_RMAKER_PARAM_POWER),
            esp_rmaker_bool(g_power));
}

void app_driver_init()
{
    app_light_init();
    button_handle_t btn_handle = iot_button_create(BUTTON_GPIO, BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        /* Register a callback for a button tap (short press) event */
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_TAP, push_btn_cb, NULL);
        /* Register Wi-Fi reset and factory reset functionality on same button */
        app_reset_button_register(btn_handle, WIFI_RESET_BUTTON_TIMEOUT, FACTORY_RESET_BUTTON_TIMEOUT);
    }
}

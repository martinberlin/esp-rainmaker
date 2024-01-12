idf_component_register(SRCS ./app_driver.c ./app_main.c
    REQUIRES    ws2812_led
                nvs_flash
                button
                esp_rainmaker
    INCLUDE_DIRS ../../common/app_wifi
                 ../../common/app_reset
                 )

target_add_binary_data(${COMPONENT_TARGET} "server.crt" TEXT)

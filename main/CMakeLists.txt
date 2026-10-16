idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES led_strip
                    esp_wifi
                    esp_event
                    esp_netif
                    nvs_flash
                    esp_http_server
                    mqtt
                    json)

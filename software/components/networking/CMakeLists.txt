idf_component_register(SRCS "wifi.c" "https.c" "mqtt.c" "ntp.c"
                    INCLUDE_DIRS "include"
                    REQUIRES esp_https_server mqtt
                    EMBED_TXTFILES "certs/https_cacert.pem"
                    "certs/https_prvtkey.pem"
                    )

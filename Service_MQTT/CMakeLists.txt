idf_component_register(SRCS "Service_MQTT.c"
                    INCLUDE_DIRS "include"
                    REQUIRES BGT60TR13C_API_HeartRate mqtt json esp_event)

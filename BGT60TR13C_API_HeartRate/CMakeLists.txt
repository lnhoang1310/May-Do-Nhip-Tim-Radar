idf_component_register(SRCS "BGT60TR13C_API_HeartRate.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver Service_MQTT bgt60tr13c_driver)

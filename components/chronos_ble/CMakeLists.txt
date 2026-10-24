idf_component_register(SRCS "chronos_ble.c"
                    INCLUDE_DIRS "include"
                    REQUIRES chronos_core bt nvs_flash
                    )

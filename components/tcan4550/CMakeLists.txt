idf_component_register(
    SRCS "tcan4550.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_spi
)

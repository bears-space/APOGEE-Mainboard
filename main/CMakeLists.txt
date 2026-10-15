idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES tcan4550 vigilant_engine
    PRIV_REQUIRES esp_hw_support esp_timer
)

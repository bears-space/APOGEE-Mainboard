idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES tcan337 tcan4550 vigilant_engine
)

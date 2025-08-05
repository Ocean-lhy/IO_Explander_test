idf_component_register(
    SRCS "io_expander.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer freertos i2c_bus
) 

idf_component_register(
    SRCS 
        "main.cpp"
    INCLUDE_DIRS 
        "."
    REQUIRES 
        driver 
        nvs_flash 
        spi_flash 
        m5unified
        IO_Explander
)


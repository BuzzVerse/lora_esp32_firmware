set(component_srcs "include/lora_driver.c" "src/lora.c" "src/esp_spi_api_impl.c")

idf_component_register(SRCS "${component_srcs}"
                       PRIV_REQUIRES driver
                       INCLUDE_DIRS "include")

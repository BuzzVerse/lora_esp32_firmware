set(component_srcs "include/driver/driver/lora_driver.c" "src/lora.c" "src/esp_api_impl.c")

idf_component_register(SRCS "${component_srcs}"
                       REQUIRES driver
                       INCLUDE_DIRS "src" "include" "include/driver" "include/protocols/packet")

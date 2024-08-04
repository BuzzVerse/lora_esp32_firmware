set(component_srcs "src/bluetooth.c")

idf_component_register(SRCS "${component_srcs}"
                       PRIV_REQUIRES driver bme280 lora
                       INCLUDE_DIRS "src"
                       REQUIRES bt)
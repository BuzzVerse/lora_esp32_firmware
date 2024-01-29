set(component_srcs "src/bme280.c" "include/bme280_lib_support.c" "include/bme280_lib.c")

idf_component_register(SRCS "${component_srcs}"
                       PRIV_REQUIRES driver
                       INCLUDE_DIRS "src" "include")
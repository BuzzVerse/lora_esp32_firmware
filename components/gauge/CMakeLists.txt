set(component_srcs "src/gauge.c" "../bme280/src/bme280.c" "../bme280/include/bme280_lib_support.c")

idf_component_register(SRCS "${component_srcs}"
                       PRIV_REQUIRES driver 
                       INCLUDE_DIRS "src" "../bme280" "../bme280/include")

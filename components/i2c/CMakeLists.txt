set(component_srcs "src/i2c.c")

idf_component_register(SRCS "${component_srcs}"
                       REQUIRES driver sensor
                       INCLUDE_DIRS "include")

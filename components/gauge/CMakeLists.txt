set(component_srcs "src/bq27441.c")

idf_component_register(SRCS "${component_srcs}"
                       PRIV_REQUIRES driver i2c
                       INCLUDE_DIRS "src" "include")
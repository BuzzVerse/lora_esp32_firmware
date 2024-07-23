set(component_srcs "src/bmm150.c" "include/bmm150_lib.c")

idf_component_register(SRCS "${component_srcs}"
                       PRIV_REQUIRES driver i2c
                       INCLUDE_DIRS "src" "include")
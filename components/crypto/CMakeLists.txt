set(component_srcs "src/crypto.c")

idf_component_register(SRCS "${component_srcs}"
                       PRIV_REQUIRES mbedtls
                       INCLUDE_DIRS "src")
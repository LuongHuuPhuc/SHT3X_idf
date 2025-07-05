idf_component_register(SRCS "sht3x_test.c"
                      INCLUDE_DIRS "."
                      REQUIRES I2C_dev driver freertos
                      PRIV_REQUIRES sht3x_idf)
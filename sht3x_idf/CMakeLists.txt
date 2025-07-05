set (priv_req driver)
set (req I2C_dev)
idf_component_register(SRCS "sht3x.c"
                      INCLUDE_DIRS "."
                      REQUIRES ${req}
                      PRIV_REQUIRES ${priv_req})
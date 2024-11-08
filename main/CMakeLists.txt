
set(APP_SR_DIR ./app/app_sr)
file(GLOB_RECURSE APP_SR_SRCS ${APP_SR_DIR}/*.c)

idf_component_register(SRCS "main.c"  "bsp/bsp.c" ${APP_SR_SRCS}
                    INCLUDE_DIRS "." "bsp" ${APP_SR_DIR})


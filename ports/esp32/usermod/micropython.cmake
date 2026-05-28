# micropython/ports/esp32/usermod/EVOX1E/micropython.cmake

set(EVOX1E_DIR ${CMAKE_CURRENT_LIST_DIR})

set(EVOX1E_SOURCES
    ${EVOX1E_DIR}/mod_evo.c
    ${EVOX1E_DIR}/evo_pwm.c
    ${EVOX1E_DIR}/evo_motor.c
    ${EVOX1E_DIR}/evo_motorpair.c
    ${EVOX1E_DIR}/evo_mecanum.c
)

set(EVOX1E_INCLUDES
    ${EVOX1E_DIR}
)

list(APPEND MICROPY_SOURCE_LIB ${EVOX1E_SOURCES})
include_directories(${EVOX1E_INCLUDES})
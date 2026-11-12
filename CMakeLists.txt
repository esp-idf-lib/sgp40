list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/eil-cmake-utils/cmake)
include(eil_check_idf_features)
eil_check_idf_has_esp_drivers()

idf_component_register(
    SRCS sgp4x.c sensirion_gas_index_algorithm.c
    INCLUDE_DIRS .
    REQUIRES i2cdev log esp_idf_lib_helpers
)

include(eil_ci)

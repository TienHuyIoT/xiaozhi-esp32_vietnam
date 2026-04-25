
list (APPEND COMPONENT_SRCDIRS ./)

set(COMPONENT_ADD_INCLUDEDIRS ./include)

set(COMPONENT_PRIV_REQUIRES media_lib_sal esp_http_client esp_timer)

register_component()


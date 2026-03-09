
list (APPEND COMPONENT_SRCDIRS ./src)

set(COMPONENT_ADD_INCLUDEDIRS ./include)

set(COMPONENT_PRIV_REQUIRES esp_timer media_src esp_extractor)

register_component()

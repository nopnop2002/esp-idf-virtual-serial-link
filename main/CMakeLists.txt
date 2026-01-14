set(srcs "main.c")

if (CONFIG_WIFI_MODE)
    list(APPEND srcs "udp_broadcast_recv.c")
    list(APPEND srcs "udp_unicast_trans.c")
elseif (CONFIG_ESPNOW_MODE)
    list(APPEND srcs "espnow_task.c")
endif()

idf_component_register(SRCS "${srcs}" INCLUDE_DIRS "." REQUIRED_IDF_TARGETS esp32s2 esp32s3)

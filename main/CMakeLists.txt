idf_component_register(SRCS "main.c" "udp_broadcat_recv.c" "udp_unicat_trans.c" "espnow_task.c"
                       INCLUDE_DIRS "."
                       REQUIRED_IDF_TARGETS esp32s2 esp32s3)

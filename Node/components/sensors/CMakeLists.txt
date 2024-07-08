set(component_srcs "mics6814.c" "dht22.c" "mq135.c" "mq131.c" "mq136.c")

# get IDF version for comparison
set(idf_version "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}")

idf_component_register(SRCS "${component_srcs}" PRIV_REQUIRES driver esp_timer INCLUDE_DIRS ".")
include_directories ("${PROJECT_SOURCE_DIR}/include")

add_library (jsonrpc-c cJSON.c jsonrpc-c.c)

target_link_libraries(jsonrpc-c ev m)

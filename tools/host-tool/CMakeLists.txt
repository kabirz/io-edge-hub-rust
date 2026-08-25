cmake_minimum_required(VERSION 3.25)
project(io-edge-hub-host VERSION 0.1.0 LANGUAGES C RC)

set(CMAKE_C_STANDARD 11)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories(${CMAKE_SOURCE_DIR}/include)

set(SOURCES
    src/main.c
    src/pcan_loader.c
    src/fw_image.c
    src/udp_manager.c
    src/can_manager.c
    src/modbus_client.c
    src/config_tab.c
    src/upgrade_tab.c
    src/modbus_tab.c
    src/history_tab.c
)

if(WIN32)
    list(APPEND SOURCES resources/resource.rc)
endif()

add_executable(${PROJECT_NAME} WIN32 ${SOURCES})

target_compile_definitions(${PROJECT_NAME} PRIVATE
    UNICODE
    _UNICODE
    _CRT_SECURE_NO_WARNINGS
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    comctl32
    comdlg32
    gdi32
    ws2_32
    iphlpapi
    setupapi
)

if(MINGW)
    target_compile_options(${PROJECT_NAME} PRIVATE -Os -municode -finput-charset=UTF-8 -fexec-charset=GBK)
    target_link_options(${PROJECT_NAME} PRIVATE -mwindows -municode -static-libgcc -s)
elseif(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE /utf-8)
    target_link_options(${PROJECT_NAME} PRIVATE /SUBSYSTEM:WINDOWS)
endif()

target_compile_definitions(${PROJECT_NAME} PRIVATE
    APP_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    APP_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    APP_VERSION_PATCH=${PROJECT_VERSION_PATCH}
)

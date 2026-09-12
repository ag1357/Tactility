cmake_minimum_required(VERSION 3.20)

include("${CMAKE_CURRENT_LIST_DIR}/../../Buildscripts/module.cmake")

file(GLOB_RECURSE SOURCE_FILES "source/*.c*")

set(REQUIRES_LIST TactilityKernel)
if (DEFINED ENV{ESP_IDF_VERSION})
    # cJSON exports (source/module.cpp) compile against IDF's json component.
    list(APPEND REQUIRES_LIST json)
else ()
    list(APPEND REQUIRES_LIST cJSON)
endif ()

tactility_add_module(c-symbols-module
    SRCS ${SOURCE_FILES}
    INCLUDE_DIRS include/
    REQUIRES ${REQUIRES_LIST}
)

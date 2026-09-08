file(GLOB_RECURSE SOURCE_FILES source/*.c*)

idf_component_register(
    SRCS ${SOURCE_FILES}
    REQUIRES TactilityKernel axp2101-module es8311-module audio-stream-module
)

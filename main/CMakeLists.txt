# ADF components are resolved automatically via EXTRA_COMPONENT_DIRS set in
# $ADF_PATH/CMakeLists.txt. All ADF examples use register_component() with no
# explicit REQUIRES for the same reason — explicit names like audio_element,
# http_stream, mp3_decoder do not map 1:1 to ADF component directory names.
# (audio_element is inside audio_pipeline; http_stream is inside audio_stream;
#  a2dp_stream is inside bluetooth_service; decoders are in esp-adf-libs.)

set(COMPONENT_SRCS
    "main.c"
    "wifi_manager.c"
    "bt_manager.c"
    "pipeline.c"
    "monitor.c"
    "passthrough_el.c"
)

set(COMPONENT_ADD_INCLUDEDIRS .)

register_component()

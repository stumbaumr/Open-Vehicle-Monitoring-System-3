set(srcs)
set(include_dirs)

if (CONFIG_OVMS_COMP_BLUETOOTH)
  list(APPEND srcs "src/elm327_engine.cpp" "src/esp32bluetooth.cpp" "src/esp32bluetooth_app_elm327.cpp" "src/esp32bluetooth_gap.cpp" "src/esp32bluetooth_gatts.cpp")
  list(APPEND include_dirs "src")
endif ()

# requirements can't depend on config
idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS ${include_dirs}
                       REQUIRES "bt"
                       PRIV_REQUIRES "main"
                       WHOLE_ARCHIVE)

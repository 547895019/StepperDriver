# Check ESP-IDF version and error out if it is not in the supported range.
#
# Note for arduino-esp32 developers: to bypass the version check locally,
# set ARDUINO_SKIP_IDF_VERSION_CHECK environment variable to 1. For example:
#   export ARDUINO_SKIP_IDF_VERSION_CHECK=1
#   idf.py build

set(srcs
  src/A4988.cpp
  src/BasicStepperDriver.cpp
  src/DRV8825.cpp
  src/DRV8834.c
  src/DRV8880.c
  src/MultiDriver.c
  src/SyncDriver.c
  )

set(includedirs src/)
set(priv_includes)
set(requires)
set(priv_requires arduino-esp32)

idf_component_register(INCLUDE_DIRS ${includedirs} PRIV_INCLUDE_DIRS ${priv_includes} SRCS ${srcs} REQUIRES ${requires} PRIV_REQUIRES ${priv_requires})


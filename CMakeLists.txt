cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)

project(kalmanif VERSION 0.0.1 LANGUAGES CXX)

# Check that the compiler has c++17 support, crash otherwise.
if(NOT MSVC)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
  if(COMPILER_SUPPORTS_CXX17)
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++17 support.")
  else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} "
      "has no C++17 support. Please use a different C++ compiler.")
  endif()
endif()

# Options. Turn on with,
# cmake -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON
# catkin build kalmanif --cmake-args -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON
# colcon build --packages-up-to kalmanif --cmake-args '-DBUILD_TESTING=ON ' '-DBUILD_EXAMPLES=ON'
option(BUILD_EXAMPLES "Build all examples." OFF)
option(PLOT_EXAMPLES "Plot the examples outputs." OFF)
option(BUILD_TESTING "Build all tests." OFF)

# SE2 demo/example produce unstable covariance matrix.
# Until it is fixed, force disable kalmanif asserts.
add_compile_definitions(KALMANIF_NO_DEBUG)

##################
## Dependencies ##
##################

find_package(manif REQUIRED)

###########
## Build ##
###########

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

# Add manif interface dependency, depending on available cmake info
if(TARGET MANIF::manif)
  target_link_libraries(${PROJECT_NAME} INTERFACE MANIF::manif)
  set(manif_DEPENDENCY "find_dependency(manif ${manif_VERSION})")
else(TARGET MANIF::manif)
  target_include_directories(
    ${PROJECT_NAME} SYSTEM INTERFACE ${manif_INCLUDE_DIRS}
  )
endif(TARGET MANIF::manif)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  target_compile_options(${PROJECT_NAME} INTERFACE -ftemplate-depth=512)
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  target_compile_options(${PROJECT_NAME} INTERFACE /bigobj)
endif()

#############
## Install ##
#############

include(CMakePackageConfigHelpers)
include(GNUInstallDirs)

set(config_install_dir "share/${PROJECT_NAME}/cmake")
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Targets:
install(
  TARGETS  ${PROJECT_NAME}
  EXPORT   ${PROJECT_NAME}Targets
  INCLUDES DESTINATION include
)

install(
  EXPORT      ${PROJECT_NAME}Targets
  NAMESPACE   ${PROJECT_NAME}::
  DESTINATION "${config_install_dir}"
)

export(
  TARGETS ${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  FILE ${PROJECT_NAME}Targets.cmake
)

export(PACKAGE ${PROJECT_NAME})

## Configuration

configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${generated_dir}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION "${config_install_dir}"
)

# Configure '<PROJECT-NAME>ConfigVersion.cmake'
write_basic_package_version_file(
  "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

# Config
install(
  FILES       "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake"
              "${generated_dir}/${PROJECT_NAME}Config.cmake"
  DESTINATION "${config_install_dir}"
)

# Headers

install(
  DIRECTORY "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}"
  DESTINATION include
)

##############
## Examples ##
##############

if(BUILD_EXAMPLES)

  add_subdirectory(examples)

endif(BUILD_EXAMPLES)

#############
## Testing ##
#############

# ------------------------------------------------------------------------------
# Unit Tests
# ------------------------------------------------------------------------------

if(BUILD_TESTING)

  enable_testing()
  add_subdirectory(test)

endif(BUILD_TESTING)

# ------------------------------------------------------------------------------
# Coverage
# ------------------------------------------------------------------------------

if(ENABLE_COVERAGE)
  add_definitions(-DKALMANIF_COVERAGE_ENABLED)
  set(
    CMAKE_CXX_FLAGS
    "\
    ${CMAKE_CXX_FLAGS} \
    -g \
    -O0 \
    -fprofile-arcs \
    -ftest-coverage \
    --coverage \
    "
  )
endif()

# ------------------------------------------------------------------------------
# CppCheck
# ------------------------------------------------------------------------------

if(ENABLE_CPPCHECK)

  # Find CppCheck executable
  find_program(CPPCHECK "cppcheck")
  if(CPPCHECK)

      # Set export commands on
      # set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

      list(APPEND CPPCHECK_ARGS
        --enable=all
        --std=c++17
        # --verbose
        --quiet
        # --check-config
        --xml-version=2
        --language=c++
        # Comment the line below to run cppcheck-html
        --error-exitcode=1
        --inline-suppr
        --suppress=*:*googletest-*
        --suppress=missingIncludeSystem
        # --suppress=missingInclude
        --suppress=unmatchedSuppression:*
        --check-config
        # Uncomment the line below to run cppcheck-html
        # --output-file=${CMAKE_BINARY_DIR}/cppcheck_results.xml
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/external
        ${CMAKE_SOURCE_DIR}/examples
        ${CMAKE_SOURCE_DIR}/test
      )

      add_custom_target(run-cppcheck
        COMMAND ${CPPCHECK} ${CPPCHECK_ARGS}
        COMMENT "Generate cppcheck report for the project"
      )

      find_program(CPPCHECK_HTML "cppcheck-htmlreport")
      if(CPPCHECK_HTML)
        add_custom_target(cppcheck-html
          COMMAND ${CPPCHECK_HTML}
          --title=${CMAKE_PROJECT_NAME}
          --file=${CMAKE_BINARY_DIR}/cppcheck_results.xml
          --report-dir=${CMAKE_BINARY_DIR}/cppcheck_results
          --source-dir=${CMAKE_SOURCE_DIR}
          COMMENT "Convert cppcheck report to HTML output"
        )
        ADD_DEPENDENCIES(cppcheck-html run-cppcheck)
      endif()
  endif()

endif()

# ------------------------------------------------------------------------------
# Valgrind
# ------------------------------------------------------------------------------

if(ENABLE_VALGRIND)
  # ctest -T memcheck
  include(CTest)
  set(
    MEMORYCHECK_COMMAND_OPTIONS
    "\
    ${MEMORYCHECK_COMMAND_OPTIONS} \
    --leak-check=full \
    --track-origins=yes \
    --trace-children=yes \
    --show-leak-kinds=all \
    --verbose \
    --error-exitcode=1 \
    "
  )
endif()

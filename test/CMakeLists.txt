find_package(GTest QUIET)

if(NOT GTEST_FOUND)
  # If not found, download it

  # Download and unpack googletest at configure time
  configure_file(gtest/CMakeLists.txt.in gtest/CMakeLists.txt)
  execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/gtest )
  if(result)
    message(FATAL_ERROR "CMake step for googletest failed: ${result}")
  endif()
  execute_process(COMMAND ${CMAKE_COMMAND} --build .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/gtest )
  if(result)
    message(FATAL_ERROR "Build step for googletest failed: ${result}")
  endif()

  # Prevent overriding the parent project's compiler/linker
  # settings on Windows
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

  # Add googletest directly to our build. This defines
  # the gtest and gtest_main targets.
  add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                   ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                   EXCLUDE_FROM_ALL)
endif()

# small helper function
function(kalmanif_add_gtest target)
  add_executable(${target} ${ARGN})
  add_dependencies(${target} gtest)
  target_link_libraries(${target} ${PROJECT_NAME} gtest)

  if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    # GCC is not strict enough by default, so enable most of the warnings.
    target_compile_options(${target} PRIVATE
      -Werror=all
      -Werror=extra
      -Wno-unknown-pragmas
      -Wno-sign-compare
      -Wno-unused-parameter
      -Wno-missing-field-initializers
      )
  endif()

  add_test(NAME ${target} COMMAND ${target})
endfunction()

include_directories(${GTEST_INCLUDE_DIRS})

kalmanif_add_gtest(gtest_demo_se2 gtest_demo_se2.cpp)
kalmanif_add_gtest(gtest_demo_se3 gtest_demo_se3.cpp)
kalmanif_add_gtest(gtest_demo_se_2_3 gtest_demo_se_2_3.cpp)

set(CXX_17_TEST_TARGETS
  gtest_demo_se2
  gtest_demo_se3
  gtest_demo_se_2_3
)

# Set required C++17 flag
set_property(TARGET ${CXX_17_TEST_TARGETS} PROPERTY CXX_STANDARD 17)
set_property(TARGET ${CXX_17_TEST_TARGETS} PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET ${CXX_17_TEST_TARGETS} PROPERTY CXX_EXTENSIONS OFF)

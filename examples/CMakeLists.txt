if(PLOT_EXAMPLES)
  # Look for a system installed sciplot
  # if none found, fallback to local copy
  find_package(sciplot QUIET)

  if (sciplot_FOUND)
    function(target_link_sciplot target)
      target_link_libraries(${target} sciplot::sciplot)
    endfunction()
  else()
    function(target_link_sciplot target)
      target_include_directories(
        ${target} SYSTEM PRIVATE "${PROJECT_SOURCE_DIR}/external/sciplot"
      )
    endfunction()
  endif()

  add_compile_definitions(WITH_PLOTS)
endif()

add_executable(demo_se2 demo_se2.cpp)
add_executable(demo_se3 demo_se3.cpp)
add_executable(demo_se_2_3 demo_se_2_3.cpp)

set(CXX_17_EXAMPLE_TARGETS

  # SO2

  # SE2
  demo_se2

  # SE3
  demo_se3

  # SE_2_3
  demo_se_2_3
)

foreach(target ${CXX_17_EXAMPLE_TARGETS})
  # Link to kalmanif
  target_link_libraries(${target} ${PROJECT_NAME})

  # GCC is not strict enough by default, so enable most of the warnings.
  if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    target_compile_options(${target} PRIVATE
      -Werror=all
      -Werror=extra
      -Wno-unknown-pragmas
      -Wno-sign-compare
      -Wno-unused-parameter
      -Wno-missing-field-initializers
    )
  endif()

  # They are examples that won't be distributed as binaries,
  # let's optimize for the hardware at hand
  target_compile_options(${target} PRIVATE
    -march=native
    -mtune=native
    $<$<CONFIG:RELEASE>:-O3>
  )
  target_compile_definitions(${target} PRIVATE
    $<$<CONFIG:RELEASE>:NDEBUG>
  )

  if(PLOT_EXAMPLES)
    # Link to sciplot
    target_link_sciplot(${target})
  endif()
endforeach()

# Set required C++17 flag
set_property(TARGET ${CXX_17_EXAMPLE_TARGETS} PROPERTY CXX_STANDARD 17)
set_property(TARGET ${CXX_17_EXAMPLE_TARGETS} PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET ${CXX_17_EXAMPLE_TARGETS} PROPERTY CXX_EXTENSIONS OFF)


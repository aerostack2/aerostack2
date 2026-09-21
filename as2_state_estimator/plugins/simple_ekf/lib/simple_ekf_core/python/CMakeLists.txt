find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)

execute_process(
  COMMAND "${Python_EXECUTABLE}" -c "import nanobind; print(nanobind.cmake_dir())"
  OUTPUT_VARIABLE NB_DIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

nanobind_add_module(_simple_ekf_core src/bindings.cpp)
target_link_libraries(_simple_ekf_core PRIVATE simple_ekf_core)

# Its own component, so that `pip install` puts only the module in the wheel, and not the C++
# libraries and headers the rest of the project installs
install(TARGETS _simple_ekf_core LIBRARY DESTINATION simple_ekf_core COMPONENT python)

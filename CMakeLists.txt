cmake_minimum_required(VERSION 3.8)
project(jetank_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# Config-only package (no compiled targets): the MoveIt stack is needed at
# launch time only, declared as <exec_depend> in package.xml.
find_package(ament_cmake REQUIRED)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip the copyright linter (no copyright headers in this package).
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint (no C++ sources; only works in a git repo anyway).
  set(ament_cmake_cpplint_FOUND TRUE)
  # Skip xmllint: it fetches the package_format3 XSD from download.ros.org,
  # which is unavailable in the offline build environment.
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_import test/test_launch_import.py)
endif()

ament_package()

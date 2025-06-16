# CMake generated Testfile for 
# Source directory: /Users/bixiosun1/Desktop/Progetto2025/vec
# Build directory: /Users/bixiosun1/Desktop/Progetto2025/vec/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test([=[boids.t]=] "/Users/bixiosun1/Desktop/Progetto2025/vec/build/Debug/boids.t")
  set_tests_properties([=[boids.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/Users/bixiosun1/Desktop/Progetto2025/vec/CMakeLists.txt;55;add_test;/Users/bixiosun1/Desktop/Progetto2025/vec/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test([=[boids.t]=] "/Users/bixiosun1/Desktop/Progetto2025/vec/build/Release/boids.t")
  set_tests_properties([=[boids.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/Users/bixiosun1/Desktop/Progetto2025/vec/CMakeLists.txt;55;add_test;/Users/bixiosun1/Desktop/Progetto2025/vec/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test([=[boids.t]=] "/Users/bixiosun1/Desktop/Progetto2025/vec/build/RelWithDebInfo/boids.t")
  set_tests_properties([=[boids.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/Users/bixiosun1/Desktop/Progetto2025/vec/CMakeLists.txt;55;add_test;/Users/bixiosun1/Desktop/Progetto2025/vec/CMakeLists.txt;0;")
else()
  add_test([=[boids.t]=] NOT_AVAILABLE)
endif()

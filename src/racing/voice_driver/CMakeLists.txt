cmake_minimum_required(VERSION 3.8)
project(voice_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(voice_api)

install(PROGRAMS
  voice_api/voice_broadcast_cli.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_broadcast
)

install(PROGRAMS
  voice_api/voice_broadcast_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_broadcast_node
)

install(PROGRAMS
  voice_api/voice_hardware_cli.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_hardware_test
)

install(PROGRAMS
  voice_api/voice_speak_now.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_speak
)

install(PROGRAMS
  voice_api/voice_speak_text.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME voice_speak_text
)

install(FILES .env
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()

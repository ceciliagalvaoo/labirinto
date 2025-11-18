cmake_minimum_required(VERSION 3.8)
project(maze_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependências obrigatórias
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

# Parte 1 - nó de navegação com mapa (BFS usando /get_map)
add_executable(maze_navigator_node src/maze_navigator.cpp)
ament_target_dependencies(
  maze_navigator_node
  rclcpp
  cg_interfaces
)

# Parte 2 - nó de mapeamento + navegação (sensores + BFS no mapa construído)
add_executable(maze_mapper_node src/maze_mapper.cpp)
ament_target_dependencies(
  maze_mapper_node
  rclcpp
  cg_interfaces
)

# Instalar os dois executáveis
install(
  TARGETS
    maze_navigator_node
    maze_mapper_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

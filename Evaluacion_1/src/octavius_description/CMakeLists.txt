cmake_minimum_required(VERSION 3.8)
project(octavius_description)

# Paquete solo de descripción: no compila código, solo instala archivos.
find_package(ament_cmake REQUIRED)

# Instala el modelo, los launch y la configuración de RViz en
# install/octavius_description/share/octavius_description/, que es donde
# los busca get_package_share_directory() y $(find octavius_description).
install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # cpplint no aplica: el paquete no tiene código C++.
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

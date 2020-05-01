cmake_minimum_required(VERSION 2.8.12)
project(uw_language_understanding)

find_package(catkin REQUIRED COMPONENTS catkin_virtualenv)

catkin_python_setup()

catkin_package()

# The command understanding code uses AllenNLP,
# and PyTorch, so no hope of using 2.7
catkin_generate_virtualenv(
  PYTHON_VERSION 3.6
)

# Make sure your python executables are installed using `catkin_install_python`:
catkin_install_python(
  PROGRAMS
    scripts/understanding_server_demo
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES src/gpsr-command-understanding/requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
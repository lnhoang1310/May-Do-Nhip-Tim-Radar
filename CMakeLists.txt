# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(EXTRA_COMPONENT_DIRS bgt60tr13c_driver BGT60TR13C_API_HeartRate Service_MQTT Wifi)
project(Project_Anh_Sao)

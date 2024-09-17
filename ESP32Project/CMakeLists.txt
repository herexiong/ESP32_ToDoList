# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)

include($ENV{ADF_PATH}/CMakeLists.txt)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32_ToDoList)

target_add_binary_data(ESP32_ToDoList.elf "./components/esp_dlna/xml/rootDesc.xml" TEXT)
target_add_binary_data(ESP32_ToDoList.elf "./components/esp_dlna/xml/rootDescLogo.xml" TEXT)
target_add_binary_data(ESP32_ToDoList.elf "./components/esp_dlna/xml/rootDescService.xml" TEXT)
target_add_binary_data(ESP32_ToDoList.elf "./components/esp_dlna/xml/SoapResponseErr.xml" TEXT)
target_add_binary_data(ESP32_ToDoList.elf "./components/esp_dlna/xml/SoapResponseOk.xml" TEXT)
target_add_binary_data(ESP32_ToDoList.elf "./components/esp_dlna/xml/devicelist.xml" TEXT)
target_add_binary_data(ESP32_ToDoList.elf "./logo.png" TEXT)

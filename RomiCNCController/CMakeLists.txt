cmake_minimum_required(VERSION 3.10)
project(romi-cnc-controller)

set(SOURCES
  Block.h
  Block.cpp
  CNC.h
  CNC.cpp
  Controller.h
  Controller.cpp
  IController.h
  IStepperController.h
  ITimer.h
  StepperControllerUno.h
  StepperControllerUno.cpp
  TimerUno.h
  TimerUno.cpp
  )

add_library(romi-cnc-controller SHARED ${SOURCES})
target_include_directories(romi-cnc-controller PUBLIC include)

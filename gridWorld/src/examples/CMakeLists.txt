
#build grid world env:
add_executable(gridWorld_example qLearningGridWorld.cpp gridWorld.cpp)
target_link_libraries(gridWorld_example rl_lib)#not sure what first argument does?

#build two wheeled env:
#add_executable(two_wheeled two_wheeled_main.cpp two_wheeled.cpp)
#target_link_libraries(two_wheeled rl_lib)#not sure what first argument does?


#build cliff world env:
add_executable(cliff_example cliff_example_main.cpp cliff_world.cpp)
target_link_libraries(cliff_example rl_lib)#not sure what first argument does?

#build two wheeled env:
#add_executable(two_wheeled two_wheeled_main.cpp two_wheeled.cpp)
#target_link_libraries(two_wheeled rl_lib)#not sure what first argument does?

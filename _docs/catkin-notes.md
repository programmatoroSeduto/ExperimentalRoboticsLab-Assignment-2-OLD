# Notes about Catkin

## Compile C++ packages

```cmake
add_executable( ??? src/???.cpp )
add_dependencies( ??? ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( ??? ${catkin_LIBRARIES} )
```


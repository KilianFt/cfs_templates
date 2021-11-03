# Templates for cfs

## linter
Used linter is [ament_lint_auto](https://github.com/ament/ament_lint/blob/master/ament_lint_auto/doc/index.rst)
> check if https://github.com/ament/ament_lint/blob/master/ament_cmake_uncrustify/doc/index.rst works automatically at colcon test


## logger
- all error message must be directed to stderr
- avoid using <<
- avoid using references for std::shared_ptr
https://docs.ros.org/en/rolling/Tutorials/Logging-and-logger-configuration.html.  
logger level can be changed programatically see EXAMPLE and in command line
'''
ros2 service call /config_logger logging_demo/srv/ConfigLogger "{logger_name: 'logger_usage_demo', level: INFO}"
'''

# TODO
- check c++ private and protected
- create template ros pkg or guidelines how to setup a new one
- create template node which includes
    - timesynch between messages
    - publish and subscribe
    - use ROS parameters
    - seperate ROS specific code from other
- launch pipeline
- how-tos for chrony time synch between computers
- dockerisation of ROS workspace

- commenting package (like automatic commenting of functions etc)
- testing examples
> use colcon test
    - unit test
    - integration tests
        - test software interfaces
    - system tests 
        - end-to-end situations between packages
        - should be in own package
    - performance tests?

- source file with aliases?
    - export ROS_DOMAIN_ID= ???
- custom msgs
- setup git to run automatic test before commit
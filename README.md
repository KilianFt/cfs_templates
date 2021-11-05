# Templates for cfs

## Demo
```
ros2 run node_templates simple_publisher

ros2 topic echo /topic2

ros2 param set /simple_publisher my_text "Hey you"
```
In new terminal
```
ros2 run node_templates subpub_example
```
In another terminal
```
ros2 run node_templates timesync
```

Several nodes can be started together like in
```
ros2 launch node_templates launch.py
```

## Included features
### linter
Used linter is [ament_lint_auto](https://github.com/ament/ament_lint/blob/master/ament_lint_auto/doc/index.rst)
> check if https://github.com/ament/ament_lint/blob/master/ament_cmake_uncrustify/doc/index.rst works automatically at colcon test

### logger
- all error message must be directed to stderr
- avoid using <<
- avoid using references for std::shared_ptr
https://docs.ros.org/en/rolling/Tutorials/Logging-and-logger-configuration.html.  
logger level can be changed programatically see EXAMPLE and in command line
'''
ros2 service call /config_logger logging_demo/srv/ConfigLogger "{logger_name: 'logger_usage_demo', level: INFO}"
'''

### ROS params
https://roboticsbackend.com/ros2-yaml-params/
A parameter can be changed by 
'''
ros2 param set /my_noce my_parameter my_value

'''
All initial parameters are usually set in a *.yaml file at config/params.yaml of every pkg.

### Service
Services only async and not same as in ROS1, see
https://answers.ros.org/question/343279/ros2-how-to-implement-a-sync-service-client-in-a-node/
> service call therefore needs own thread to be done, can be handled in different ways, see above.

### Time Syncronization
The implementation in a class follows this example.
https://answers.ros.org/question/361637/using-c-message-filters-in-ros2/?answer=375601#post-id-375601

# TODO
- check c++ private and protected
- create template node which includes
    - timesynch between messages
    - seperate ROS specific code from other
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

- source file with aliases ?
    - export ROS_DOMAIN_ID = ?
- setup git to run automatic test before commit

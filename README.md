# ros2_integration_test

An example of testing C++ ROS2 nodes (ie., ROS2 integration test or level 2 unit testing) using the catch2 framework. 

First, move this directory to the `src/` folder of your colcon
workspace.  This is desirable or you may get a lot of errors from
`ament_lint_cmake`.

For example, your workspace directory structure should look like this:

```
colcon_ws/
├── build/
├── install/
├── log/
└── src/
    ├── ros2_integration_test/
    ├── ros_package1/
    ├── ros_package2/
    └── ros_package3/
```

``` bash
$ mkdir -p ~/colcon_ws/src
$ cd ~/colcon_ws/src
$ git clone git@github.com:TommyChangUMD/ros2_integration_test.git
```

## Install the catch_ros2

``` bash
$ source /opt/ros/humble/setup.bash  # if needed
$ apt install ros-${ROS_DISTRO}-catch-ros2
```

Verify that the package is installed
``` bash
$ source /opt/ros/humble/setup.bash  # if needed
$ ros2 pkg list | grep catch_ros2
  catch_ros2
```

see https://github.com/ngmor/catch_ros2/tree/main

## How to Compile:
```bash
$ cd ~/colcon_ws/   # assuming your workspace is at '~/colcon_ws'
$ rm -rf install/ build/
$ source /opt/ros/humble/setup.bash  # if needed
$ colcon build --packages-select integration_test
```

## How to Run:
First, soruce the setup file:
```bash
$ source install/setup.bash
```
### then, run the test and look at the output:
```bash
$ colcon test --packages-select integration_test
$ cat log/latest_test/integration_test/stdout_stderr.log
```

### alternatively, you can combine these into one step:
```bash
$ colcon test  --event-handlers console_cohesion+ --packages-select integration_test
```


### check the return status:
```
$ colcon test-result --verbose --test-result-base build/integration_test
$ echo $?
```

## Example output

```
... <SKIP> ...

Constructing a list of tests
Done constructing a list of tests
Updating test list for fixtures
Added 0 tests to meet fixture requirements
Checking test dependency graph...
Checking test dependency graph end
test 1
    Start 1: ExampleIntegration_TestYAML

1: Test command: /usr/bin/python3 "-u" "/opt/ros/humble/share/catch_ros2/cmake/../scripts/run_test.py" "/home/tchang/Projects/ros2_integration_test/build/integration_test/test_results/integration_test/ExampleIntegration_TestYAML.xml" "--package-name" "integration_test" "--command" "ros2" "launch" "integration_test" "integration_test.launch.yaml" "result_file:=/home/tchang/Projects/ros2_integration_test/build/integration_test/test_results/integration_test/ExampleIntegration_TestYAML.xml"
1: Test timeout computed to be: 60
1: -- run_test.py: invoking following command in '/home/tchang/Projects/ros2_integration_test/build/integration_test':
1:  - ros2 launch integration_test integration_test.launch.yaml result_file:=/home/tchang/Projects/ros2_integration_test/build/integration_test/test_results/integration_test/ExampleIntegration_TestYAML.xml
1: [INFO] [launch]: All log files can be found below /home/tchang/.ros/log/2024-10-31-05-17-07-071766-tchang-IdeaPad-3-17ABA7-375974
1: [INFO] [launch]: Default logging verbosity is set to INFO
1: [INFO] [talker-1]: process started with pid [375975]
1: [INFO] [integration_aux_node-2]: process started with pid [375977]
1: [INFO] [integration_test_node-3]: process started with pid [375979]
1: [talker-1] [INFO] [1730366228.201536616] [talker]: Publishing: 'Hello World: 1'
1: [integration_test_node-3] [INFO] [1730366228.214357240] [talker_test_node]: I heard: 'Hello World: 1'
1: [integration_test_node-3] Randomness seeded to: 4149424261
1: [integration_test_node-3] ===============================================================================
1: [integration_test_node-3] All tests passed (2 assertions in 2 test cases)
1: [integration_test_node-3] 
1: [INFO] [integration_test_node-3]: process has finished cleanly [pid 375979]
1: [INFO] [launch]: process[integration_test_node-3] was required: shutting down launched system
1: [INFO] [integration_aux_node-2]: sending signal 'SIGINT' to process[integration_aux_node-2]
1: [INFO] [talker-1]: sending signal 'SIGINT' to process[talker-1]
1: [integration_aux_node-2] [INFO] [1730366228.825959133] [rclcpp]: signal_handler(signum=2)
1: [talker-1] [INFO] [1730366228.826492651] [rclcpp]: signal_handler(signum=2)
1: [INFO] [integration_aux_node-2]: process has finished cleanly [pid 375977]
1: [INFO] [talker-1]: process has finished cleanly [pid 375975]
1: -- run_test.py: return code 0
1: -- run_test.py: verify result file '/home/tchang/Projects/ros2_integration_test/build/integration_test/test_results/integration_test/ExampleIntegration_TestYAML.xml'
1/1 Test #1: ExampleIntegration_TestYAML ......   Passed    2.14 sec

100% tests passed, 0 tests failed out of 1

Total Test time (real) =   2.14 sec
```

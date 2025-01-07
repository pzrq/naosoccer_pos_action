# naosoccer_pos_action

[![Build and Test (humble)](../../actions/workflows/build_and_test_humble.yaml/badge.svg?branch=iron)](../../actions/workflows/build_and_test_humble.yaml?query=branch:iron)
[![Build and Test (iron)](../../actions/workflows/build_and_test_iron.yaml/badge.svg?branch=iron)](../../actions/workflows/build_and_test_iron.yaml?query=branch:iron)
[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)

Executes a .pos file action for a NAO robot, a filetype defined in rUNSWift's codebase.

## Steps (on real robot)

1. On a terminal on the robot, run `ros2 run nao_lola_client nao_lola_client`
2. In a new terminal (either on robot, or on your machine), run `ros2 run naosoccer_pos_action naosoccer_pos_action`
3. In a new terminal (either on robot, or on your machine), publish a start message

    `ros2 topic pub --once start_pos_action std_msgs/msg/Bool '{data: true}'`

## Steps (simulation using rcsoccer3d)

1. Run `rcsoccersim3d`
2. In a new terminal, run `ros2 run rcss3d_nao rcss3d_nao`
3. In a new terminal, run `ros2 run naosoccer_pos_action naosoccer_pos_action`
4. In a new terminal, publish a start message

    `ros2 topic pub --once start_pos_action std_msgs/msg/Bool '{data: true}'`

## Using a different motion.

Pos files define the different motions, and can be specified using a ros parameter for the naosoccer_pos_action node.
To set the parameter, when running the naosoccer_pos_action node in the steps above, instead do the following:

```
ros2 run naosoccer_pos_action naosoccer_pos_action --ros-args -p "file:=src/naosoccer_pos_action/pos/tilt.pos"
```

## Action

There is a ROS2 action available to run .pos files smoothly.

### Quick Start - Publishing to the quick_start_choice subscriber

1. (On a real robot) Start `nao_lola_client`:
   ```
   ros2 run nao_lola_client nao_lola_client
   ```

2. Publish the `.pos` file:
   ```
   ros2 topic pub --once quick_start_choice std_msgs/msg/String '{data: "sit-to-stand.pos"}'
   ```

This is provided because writing actions is intermediate-level ROS, so this Quick Start is designed to help
beginners get started.

It is also sufficient for simple use cases.

### Action Client

For more complex use cases, an action client is provided.

### Action Server

For more complex use cases, an action server is provided.

## Eagerly registering .pos files

To get a little closer to real-time ROS, this package allows eager loading and parsing of the .pos files into memory,
the combination is henceforth known as registering the .pos files.

If files were not eagerly registered, an attempt is made to lazily load them.

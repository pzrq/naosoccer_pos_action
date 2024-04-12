[![Build and test (humble)](https://github.com/antbono/nao_pos/actions/workflows/build_and_test_humble.yaml/badge.svg)](https://github.com/antbono/nao_pos/actions/workflows/build_and_test_humble.yaml)
[![Build and test (iron)](https://github.com/antbono/nao_pos/actions/workflows/build_and_test_iron.yaml/badge.svg)](https://github.com/antbono/nao_pos/actions/workflows/build_and_test_iron.yaml)
[![Build (rolling)](https://github.com/antbono/nao_pos/actions/workflows/build_and_test_rolling.yaml/badge.svg)](https://github.com/antbono/nao_pos/actions/workflows/build_and_test_rolling.yaml)

# nao_pos

This repository hosts two ROS2 packages for NAO v6 that allows to play gestures and it is part of the [Open Access NAO](https://github.com/antbono/OAN) framework.

This project is a fork of [naosoccer_pos_action](https://github.com/ijnek/naosoccer_pos_action).

The main new features are two:

- Implementing the request to play a movement as a **ROS2 action**. This gives much more control on the execution and more flexibility.
- Thanks to a new format for describing the gestures, it is possibile to **actuate any subset of the joints**, while before for each gesture you had to take the control of the whole robot. This gives much more flexibility and allows different programs to move the joints if necessary.


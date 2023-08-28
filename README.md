# zed_drivers

ROS 2 drivers and software packages for the Stereolabs ZED cameras.

## Contents

- [`zed_driver`](src/zed_driver/README.md): General-purpose ROS 2 driver for the ZED cameras.

## Usage

All packages in this repository are ROS 2 packages that build inside an environment configured with the ZED SDK and all of its dependencies.

The DUA containers provided here are configured to build the packages in this repository.

### DUA integration

This project is based on the [Distributed Unified Architecture](dua-template.md).

It is an independent unit that can also be used as a module in a larger project.

#### Supported targets

The following DUA targets are supported:

- [x] `x86-cudev`
- [x] `jetson5c7`

---

## License

This work is licensed under the GNU General Public License v3.0. See the [`LICENSE`](LICENSE) file for details.

## Copyright

Copyright (c) 2023, Intelligent Systems Lab, University of Rome Tor Vergata

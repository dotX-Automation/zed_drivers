# zed_drivers

ROS 2 drivers and software packages for the Stereolabs ZED cameras.

## Contents

- [`zed_driver`](src/zed_driver/README.md): General-purpose ROS 2 driver for the ZED cameras.
- [`zed_description`](src/zed_description/): Description files for ZED cameras.
- [`zed_drivers`](src/zed_drivers/): Metapackage.

## Usage

All packages in this repository are ROS 2 packages that build inside an environment configured with the ZED SDK and all of its dependencies.

The DUA containers provided here are configured to build the packages in this repository.

### DUA integration

This project is based on the [Distributed Unified Architecture](dua-template.md).

It is an independent unit that can also be used as a module in a larger project.

#### Supported targets

The following DUA targets are supported:

- [x] `x86-cudev`
- [x] `x86-cudev-ai`
- [x] `jetson5`
- [x] `jetson5-ai`

---

## Copyright and License

Copyright 2024 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.

You may obtain a copy of the License at <http://www.apache.org/licenses/LICENSE-2.0>.

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

See the License for the specific language governing permissions and limitations under the License.

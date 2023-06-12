#!/usr/bin/env bash

# Setup script ZED SDK udev rules.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# June 12, 2023

set -o errexit
set -o nounset
set -o pipefail
if [[ "${TRACE-0}" == "1" ]]; then set -o xtrace; fi

function usage {
  echo >&2 "Usage:"
  echo >&2 "    zed_udev_setup.sh"
}

if [[ "${1-}" =~ ^-*h(elp)?$ ]]; then
  usage
  exit 1
fi

# Check that the rules file is present
if [[ -f "./bin/99-slabs.rules" ]]; then
  rules_file="./bin/99-slabs.rules"
elif [[ -f "./99-slabs.rules" ]]; then
  rules_file="./99-slabs.rules"
else
  echo >&2 "ERROR: rules file '99-slabs.rules' not found"
  exit 1
fi

# Copy the rules file to the udev rules directory
# To extract it from the SDK, run:
# bash ./zed_installer.run --tar -x './99-slabs.rules' > /dev/null 2>&1
echo "Copying udev rules file to /etc/udev/rules.d/..."
sudo cp "${rules_file}" /etc/udev/rules.d/99-slabs.rules
sudo chmod 777 "/etc/udev/rules.d/99-slabs.rules"
sudo udevadm control --reload-rules && sudo udevadm trigger
echo "udev rules reloaded"

# DUA environment image.
#
# Roberto Masocco <robmasocco@gmail.com>
# Intelligent Systems Lab <isl.torvergata@gmail.com>
#
# April 5, 2023

FROM intelligentsystemslabutv/dua-foundation:jetson5c7

# Change this if you encounter problems with the default user
ARG USER_UID=1000

ENV DEBIAN_FRONTEND=noninteractive

# IMAGE SETUP START #
# zed_drivers START #
# Install ZED SDK dependencies
RUN apt-get update && \
  apt-get install -y --no-install-recommends \
  apt-transport-https \
  nvidia-cudnn8-dev \
  nvidia-tensorrt-dev \
  udev \
  zstd && \
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*/apt/lists/*

# Install the ZED SDK (no Python API, since it appears to mess with the ROS 2 installation)
# We currently support version 4.0
# Do some more black magic here to enable streaming features on Jetson inside a container
# The syntax for the echo is:
# # R${L4T_MAJOR_VERSION} (release), REVISION: ${L4T_MINOR_VERSION}.${L4T_PATCH_VERSION}
RUN echo "# R35 (release), REVISION: 1.0" > /etc/nv_tegra_release && \
  wget --no-check-certificate -nc -O zed_sdk.run https://download.stereolabs.com/zedsdk/4.0/l4t35.1/jetsons && \
  chmod +x zed_sdk.run && \
  ./zed_sdk.run -- silent skip_python && \
  rm zed_sdk.run && \
  ln -sf /usr/lib/aarch64-linux-gnu/tegra/libv4l2.so.0 /usr/lib/aarch64-linux-gnu/libv4l2.so && \
  rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*/apt/lists/* && \
  chgrp -R internal /usr/local/zed && \
  chmod -R g+rwx /usr/local/zed
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/zed/lib
ENV LOGNAME=neo
# zed_drivers END #
# IMAGE SETUP END #

# Create a user with Zsh as shell, hashed password, and add it to the sudoers
# To generate the hashed password, run:
#   mkpasswd -m sha-512 PASSWORD intelsyslab
RUN useradd -r -m -s /usr/bin/zsh -u ${USER_UID} -G adm,dialout,internal,plugdev,sudo,tty,video -p '$6$intelsyslab$5d.R83Jldx4XgDqdfRUh3caOq5vzxBNRhLeeJyB7IvzfvMtOM38JGtz3ok8N5DI1oHUAWJjlcSOz3hdeq7RcK/' neo
ENV HOME=/home/neo

# Create workspace directory: host workspaces will be mounted here
RUN mkdir ${HOME}/workspace && \
  chown neo:neo ${HOME}/workspace

# Create directory for shell history file
RUN mkdir ${HOME}/zsh_history && \
  chown neo:neo ${HOME}/zsh_history

# Create SSH directory for user
RUN mkdir ${HOME}/.ssh

# Switch to internal user
USER neo
WORKDIR ${HOME}

# Copy user configuration files
COPY --chown=neo:neo ./aliases.sh ./.aliases.sh
COPY --chown=neo:neo ./bashrc ./.bashrc
COPY --chown=neo:neo ./colcon-defaults.yaml /home/neo/.colcon/defaults.yaml
COPY --chown=neo:neo ./commands.sh ./.commands.sh
COPY --chown=neo:neo ./nanorc ./.nanorc
COPY --chown=neo:neo ./ros2.sh ./.ros2.sh
COPY --chown=neo:neo ./vimrc ./.vimrc

# Configure Zsh for internal user
ENV ZSH=${HOME}/.oh-my-zsh
ENV ZSH_CUSTOM=${ZSH}/custom
ENV ZSH_PLUGINS=${ZSH_CUSTOM}/plugins
ENV ZSH_THEMES=${ZSH_CUSTOM}/themes
RUN wget -qO- https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh | zsh || true
RUN git clone --single-branch --branch 'master' --depth 1 https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_PLUGINS}/zsh-syntax-highlighting \
  && git clone --single-branch --branch 'master' --depth 1 https://github.com/zsh-users/zsh-autosuggestions ${ZSH_PLUGINS}/zsh-autosuggestions \
  && git clone --single-branch --depth 1 https://github.com/romkatv/powerlevel10k.git ${ZSH_THEMES}/powerlevel10k
COPY --chown=neo:neo ./p10k.zsh ./.p10k.zsh
COPY --chown=neo:neo ./zshrc ./.zshrc

ENV DEBIAN_FRONTEND=dialog

# Add image information
LABEL author.name="Roberto Masocco"
LABEL author.email="robmasocco@gmail.com"
LABEL organization.name="Intelligent Systems Lab"
LABEL organization.email="isl.torvergata@gmail.com"

# By default, start a basic shell
CMD ["bash"]

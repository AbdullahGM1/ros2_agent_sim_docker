#! /usr/bin/env bash

set -e

## Bash script to setup PX4 development environment on Ubuntu LTS (24.04, 22.04, 20.04, 18.04).
## Can also be used in docker.
##
## Installs:
## - Common dependencies and tools for nuttx, jMAVSim, Gazebo
## - NuttX toolchain (omit with arg: --no-nuttx)
## - jMAVSim and Gazebo Harmonic simulator (omit with arg: --no-sim-tools)
##

INSTALL_NUTTX="true"
INSTALL_SIM="true"
INSTALL_ARCH=`uname -m`

# Parse arguments
for arg in "$@"
do
	if [[ $arg == "--no-nuttx" ]]; then
		INSTALL_NUTTX="false"
	fi

	if [[ $arg == "--no-sim-tools" ]]; then
		INSTALL_SIM="false"
	fi
done

# detect if running in docker
if [ -f /.dockerenv ]; then
	echo "Running within docker, installing initial dependencies";
	apt-get --quiet -y update && DEBIAN_FRONTEND=noninteractive apt-get --quiet -y install \
		ca-certificates \
		gnupg \
		lsb-core \
		sudo \
		wget \
		;
fi

# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# check requirements.txt exists (script not run in source tree)
REQUIREMENTS_FILE="requirements.txt"
if [[ ! -f "${DIR}/${REQUIREMENTS_FILE}" ]]; then
	echo "FAILED: ${REQUIREMENTS_FILE} needed in same directory as ubuntu.sh (${DIR})."
	return 1
fi


# check ubuntu version
# otherwise warn and point to docker?
UBUNTU_RELEASE="`lsb_release -rs`"

if [[ "${UBUNTU_RELEASE}" == "14.04" ]]; then
	echo "Ubuntu 14.04 is no longer supported"
	exit 1
elif [[ "${UBUNTU_RELEASE}" == "16.04" ]]; then
	echo "Ubuntu 16.04 is no longer supported"
	exit 1
elif [[ "${UBUNTU_RELEASE}" == "18.04" ]]; then
	echo "Ubuntu 18.04"
elif [[ "${UBUNTU_RELEASE}" == "20.04" ]]; then
	echo "Ubuntu 20.04"
elif [[ "${UBUNTU_RELEASE}" == "22.04" ]]; then
	echo "Ubuntu 22.04"
elif [[ "${UBUNTU_RELEASE}" == "24.04" ]]; then
	echo "Ubuntu 24.04"
fi


echo
echo "Installing PX4 general dependencies"

sudo apt-get update -y --quiet

# Install common packages with Ubuntu version compatibility
if [[ "${UBUNTU_RELEASE}" == "24.04" ]]; then
	# Ubuntu 24.04 package names
	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		astyle \
		build-essential \
		cmake \
		cppcheck \
		file \
		g++ \
		gcc \
		gdb \
		git \
		lcov \
		libfuse2t64 \
		libxml2-dev \
		libxml2-utils \
		make \
		ninja-build \
		python3 \
		python3-dev \
		python3-pip \
		python3-setuptools \
		python3-wheel \
		rsync \
		shellcheck \
		unzip \
		zip \
		;
else
	# Older Ubuntu versions (22.04, 20.04, 18.04)
	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		astyle \
		build-essential \
		cmake \
		cppcheck \
		file \
		g++ \
		gcc \
		gdb \
		git \
		lcov \
		libfuse2 \
		libxml2-dev \
		libxml2-utils \
		make \
		ninja-build \
		python3 \
		python3-dev \
		python3-pip \
		python3-setuptools \
		python3-wheel \
		rsync \
		shellcheck \
		unzip \
		zip \
		;
fi

# Python3 dependencies - Handle Ubuntu 24.04 externally-managed-environment
echo
echo "Installing PX4 Python3 dependencies"

# For Ubuntu 24.04, install python3-full and handle pip restrictions
if [[ "${UBUNTU_RELEASE}" == "24.04" ]]; then
	echo "Ubuntu 24.04 detected - installing python3-full and handling pip restrictions"
	sudo apt-get -y --quiet --no-install-recommends install python3-full python3-venv pipx
	
	# Try to install from requirements.txt first, if that fails, install essential packages manually
	echo "Installing Python packages with --break-system-packages (safe in Docker)"
	if [ -n "$VIRTUAL_ENV" ]; then
		# In virtual environment, no need for --break-system-packages
		python -m pip install --no-cache-dir --disable-pip-version-check -r ${DIR}/requirements.txt || {
			echo "requirements.txt failed, installing essential packages manually"
			python -m pip install --no-cache-dir --disable-pip-version-check empy jinja2 jsonschema packaging pandas pymavlink pyulog pyyaml toml numpy
		}
	else
		# In Docker, use --break-system-packages and disable hash checking for speed
		python3 -m pip install --break-system-packages --no-cache-dir --disable-pip-version-check --trusted-host pypi.org --trusted-host pypi.python.org --trusted-host files.pythonhosted.org -r ${DIR}/requirements.txt || {
			echo "requirements.txt failed, installing essential packages manually"
			python3 -m pip install --break-system-packages --no-cache-dir --disable-pip-version-check --trusted-host pypi.org --trusted-host pypi.python.org --trusted-host files.pythonhosted.org empy jinja2 jsonschema packaging pandas pymavlink pyulog pyyaml toml numpy
		}
	fi
else
	# For older Ubuntu versions, use standard approach
	if [ -n "$VIRTUAL_ENV" ]; then
		# virtual environments don't allow --user option
		python -m pip install -r ${DIR}/requirements.txt
	else
		# older versions of Ubuntu require --user option
		python3 -m pip install -r ${DIR}/requirements.txt
	fi
fi

# NuttX toolchain (arm-none-eabi-gcc)
if [[ $INSTALL_NUTTX == "true" ]]; then

	echo
	echo "Installing NuttX dependencies"

	# Install NuttX dependencies with Ubuntu version compatibility
	if [[ "${UBUNTU_RELEASE}" == "24.04" ]]; then
		# Ubuntu 24.04 package names
		sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
			automake \
			binutils-dev \
			bison \
			build-essential \
			flex \
			g++-multilib \
			gcc-multilib \
			gdb-multiarch \
			genromfs \
			gettext \
			gperf \
			libelf-dev \
			libexpat-dev \
			libgmp-dev \
			libisl-dev \
			libmpc-dev \
			libmpfr-dev \
			libncurses6 \
			libncurses-dev \
			libtool \
			pkg-config \
			screen \
			texinfo \
			u-boot-tools \
			util-linux \
			vim-common \
			;
	else
		# Older Ubuntu versions (22.04, 20.04, 18.04)
		sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
			automake \
			binutils-dev \
			bison \
			build-essential \
			flex \
			g++-multilib \
			gcc-multilib \
			gdb-multiarch \
			genromfs \
			gettext \
			gperf \
			libelf-dev \
			libexpat-dev \
			libgmp-dev \
			libisl-dev \
			libmpc-dev \
			libmpfr-dev \
			libncurses5 \
			libncurses5-dev \
			libtool \
			pkg-config \
			screen \
			texinfo \
			u-boot-tools \
			util-linux \
			vim-common \
			;
	fi
	
	# Install kconfig-frontends for supported Ubuntu versions
	if [[ "${UBUNTU_RELEASE}" == "20.04" || "${UBUNTU_RELEASE}" == "22.04" || "${UBUNTU_RELEASE}" == "24.04" ]]; then
		sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		kconfig-frontends \
		;
	fi


	if [ -n "$USER" ]; then
		# add user to dialout group (serial port access)
		sudo usermod -a -G dialout $USER
	fi

	# arm-none-eabi-gcc
	NUTTX_GCC_VERSION="9-2020-q2-update"
	NUTTX_GCC_VERSION_SHORT="9-2020q2"

	source $HOME/.profile # load changed path for the case the script is reran before relogin
	if [ $(which arm-none-eabi-gcc) ]; then
		GCC_VER_STR=$(arm-none-eabi-gcc --version)
		GCC_FOUND_VER=$(echo $GCC_VER_STR | grep -c "${NUTTX_GCC_VERSION}")
	fi

	if [[ "$GCC_FOUND_VER" == "1" ]]; then
		echo "arm-none-eabi-gcc-${NUTTX_GCC_VERSION} found, skipping installation"

	else
		echo "Installing arm-none-eabi-gcc-${NUTTX_GCC_VERSION}";
		wget -O /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/${NUTTX_GCC_VERSION_SHORT}/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-${INSTALL_ARCH}-linux.tar.bz2 && \
			sudo tar -jxf /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 -C /opt/;

		# add arm-none-eabi-gcc to user's PATH
		exportline="export PATH=/opt/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}/bin:\$PATH"

		if grep -Fxq "$exportline" $HOME/.profile; then
			echo "${NUTTX_GCC_VERSION} path already set.";
		else
			echo $exportline >> $HOME/.profile;
			source $HOME/.profile; # Allows to directly build NuttX targets in the same terminal
		fi
	fi
fi

# Simulation tools
if [[ $INSTALL_SIM == "true" ]]; then

	echo
	echo "Installing PX4 simulation dependencies"

	# General simulation dependencies
	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		bc \
		;

	# Java version selection based on Ubuntu version
	if [[ "${UBUNTU_RELEASE}" == "18.04" ]]; then
		java_version=11
	elif [[ "${UBUNTU_RELEASE}" == "20.04" ]]; then
		java_version=13
	elif [[ "${UBUNTU_RELEASE}" == "22.04" ]]; then
		java_version=11
	elif [[ "${UBUNTU_RELEASE}" == "24.04" ]]; then
		java_version=21  # Updated for Ubuntu 24.04
	else
		java_version=21  # Default to latest for newer versions
	fi
	
	# Java (jmavsim)
	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		ant \
		openjdk-$java_version-jre \
		openjdk-$java_version-jdk \
		libvecmath-java \
		;

	# Set Java as default
	sudo update-alternatives --set java $(update-alternatives --list java | grep "java-$java_version") || echo "Java alternatives setting skipped"

	# Gazebo installation - UPDATED TO HARMONIC for all versions
	echo "Installing Gazebo Harmonic"
	
	# Add Gazebo binary repository
	sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
	sudo apt-get update -y --quiet

	# Install Gazebo Harmonic for all supported Ubuntu versions
	if [[ "${UBUNTU_RELEASE}" == "24.04" || "${UBUNTU_RELEASE}" == "22.04" ]]; then
		echo "Installing Gazebo Harmonic"
		gazebo_packages="gz-harmonic"
	elif [[ "${UBUNTU_RELEASE}" == "20.04" ]]; then
		echo "Ubuntu 20.04 detected - Gazebo Harmonic may not be officially supported"
		echo "Attempting to install Gazebo Harmonic anyway..."
		gazebo_packages="gz-harmonic"
	else
		# Fallback for older versions - try Harmonic first, fall back to available version
		echo "Older Ubuntu version detected - trying Gazebo Harmonic"
		gazebo_packages="gz-harmonic"
	fi

	# Install Gazebo and related packages
	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		dmidecode \
		$gazebo_packages \
		gstreamer1.0-plugins-bad \
		gstreamer1.0-plugins-base \
		gstreamer1.0-plugins-good \
		gstreamer1.0-plugins-ugly \
		gstreamer1.0-libav \
		libeigen3-dev \
		libgstreamer-plugins-base1.0-dev \
		libimage-exiftool-perl \
		libopencv-dev \
		libxml2-utils \
		pkg-config \
		protobuf-compiler \
		;

	# Fix VMWare 3D graphics acceleration for gazebo
	if sudo dmidecode -t system | grep -q "Manufacturer: VMware, Inc." ; then
		echo "export SVGA_VGPU10=0" >> ~/.profile
	fi

fi

if [[ $INSTALL_NUTTX == "true" ]]; then
	echo
	echo "Relogin or reboot computer before attempting to build NuttX targets"
fi
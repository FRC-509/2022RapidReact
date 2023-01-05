#!/usr/bin/env bash

# Target version of WPILib to install/use.
export WPILIB_VERSION=2022.4.1

# Debian/Ubuntu's apt package manager is interactive by default, which is not handy in a Docker build.
export DEBIAN_FRONTEND=noninteractive

# helper functions.
fatal(){  error "$@" && exit 1; }
error(){  echo "[ERROR]:    $@" >&2; }
warn(){   echo "[WARNING]:  $@" >&2; }
info(){   echo "[INFO]:     $@" >&2; }
require_bin(){  type -p "$1" >/dev/null 2>&1 || (fatal "missing $1"); }
require_bins(){ for a in "$@"; do require_bin "$a"; done; }
require_root(){ [[ $EUID -eq 0 ]] || fatal "must be run as root user (try sudo)"; }

# perform final "pre-flight checks"...
require_root
require_bins apt apt-get wget

# install apt-transport-https, which is required for apt to be able to use https sources.
apt-get -qqy update && apt-get -qqy install --no-install-recommends ca-certificates gnupg apt-transport-https software-properties-common

# add our additional apt sources.
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null \
&& apt-add-repository 'deb https://apt.kitware.com/ubuntu/ jammy main' \
&& add-apt-repository ppa:git-core/ppa \
&& apt-get update -qqy

# Install system dependencies.
apt-get -qqy install --no-install-recommends \
    bison \
    build-essential \
    clang-format-14 \
    cmake \
    crossbuild-essential-arm64 \
    crossbuild-essential-armhf \
    crossbuild-essential-i386 \
    curl \
    fakeroot \
    flex \
    g++ \
    gawk \
    gcc \
    gdb \
    git \
    java-common \
    libc6-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libgmp-dev \
    libisl-dev \
    libmpc-dev \
    libmpfr-dev \
    libopencv-dev \
    libvulkan-dev \
    libx11-dev \
    libxcursor-dev \
    libxi-dev \
    libxinerama-dev \
    libxrandr-dev \
    m4 \
    make \
    mesa-common-dev \
    mingw-w64 \
    openjdk-17-jdk \
    python-all-dev \
    python3-dev \
    python3-pip \
    python3-setuptools \
    rsync \
    strip-nondeterminism \
    sudo \
    sudo \
    texinfo \
    tzdata \
    unzip \
    wget \
    zip

# Download, extract, and install WPILib for Linux.
wget -qO- "https://github.com/wpilibsuite/allwpilib/releases/download/v${WPILIB_VERSION}/WPILib_Linux-${WPILIB_VERSION}.tar.gz" | tar -xz -C /tmp && mv /tmp/WPILib_Linux-${WPILIB_VERSION} /opt/wpilib

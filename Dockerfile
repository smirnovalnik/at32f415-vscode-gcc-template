# Use a base image with build tools
FROM ubuntu:22.04

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates gnupg dirmngr && \
    apt-get install -y --no-install-recommends \
        build-essential git make clang-format \
        tar unzip wget tzdata && \
    rm -rf /var/lib/apt/lists/* && \
    wget -q https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.xz && \
    tar -xJf arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.xz -C /opt && \
    ln -s /opt/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi/bin/* /usr/local/bin/ && \
    rm arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi.tar.xz && \
    wget -q https://www.arterychip.com/file/download/2613 -O OpenOCD_Linux_x86-64_V2.0.6.zip || true && \
    unzip -q OpenOCD_Linux_x86-64_V2.0.6.zip -d /opt && \
    ln -s /opt/OpenOCD_Linux_x86-64_V2.0.6/bin/* /usr/local/bin/ && \
    rm OpenOCD_Linux_x86-64_V2.0.6.zip

# Set working directory
WORKDIR /project

# Default command
ENTRYPOINT ["/bin/bash"]

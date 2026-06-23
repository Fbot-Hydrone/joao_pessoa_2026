FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-empy \
    python3-dev \
    git \
    build-essential \
    cmake \
    libyaml-cpp-dev \
    default-jdk \
    maven \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install empy==3.3.4 pexpect pymavlink

RUN git config --global --add safe.directory /workspace/drone_ws/src/ardupilot

# Copia e monta o gerador DDS corretamente
COPY Micro-XRCE-DDS-Gen /opt/microxrceddsgen
WORKDIR /opt/microxrceddsgen
RUN ./gradlew assemble

# Configura a variável que o ArduPilot usa para achar o gerador
ENV MICROXRCEDDSGEN_DIR=/opt/microxrceddsgen
# Adiciona ao PATH para garantir
ENV PATH="/opt/microxrceddsgen/scripts:${PATH}"

WORKDIR /workspace

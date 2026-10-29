# ArduPilot SITL (ArduCopter), compilé au même tag que le firmware de la Pixhawk.
# Pas de ROS ni de MAVProxy : le binaire arducopter parle directement à Gazebo (JSON)
# et expose MAVLink pour MAVROS et QGroundControl.

FROM ubuntu:24.04 AS build
ARG ARDUPILOT_TAG
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       build-essential ca-certificates git python3 python3-dev python3-venv \
    && rm -rf /var/lib/apt/lists/*
# Outils Python requis par waf (génération MAVLink et DroneCAN)
RUN python3 -m venv /opt/venv \
    && /opt/venv/bin/pip install --no-cache-dir empy==3.3.4 future lxml pexpect
ENV PATH=/opt/venv/bin:$PATH
RUN git clone --depth 1 --branch "${ARDUPILOT_TAG}" --recurse-submodules --shallow-submodules \
        https://github.com/ArduPilot/ardupilot.git /ardupilot
WORKDIR /ardupilot
# Correctifs du simulateur pas encore dans le tag : appliqués s'ils manquent, ignorés s'ils y sont déjà
COPY sim/docker/patches /patches
RUN for p in /patches/*.patch; do \
        git apply --reverse --check "$p" 2>/dev/null || git apply "$p"; \
    done
RUN ./waf configure --board sitl && ./waf copter

FROM ubuntu:24.04
COPY --from=build /ardupilot/build/sitl/bin/arducopter /usr/local/bin/arducopter
COPY --from=build /ardupilot/Tools/autotest/default_params/copter.parm /opt/ardupilot/params/
# eeprom.bin et logs du SITL
WORKDIR /var/lib/sitl

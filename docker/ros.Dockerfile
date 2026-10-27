# Images ROS 2 de la simulation. Toute l'installation passe par provision/ (Ansible) :
# même procédure que sur un PC Linux, et plus tard sur la Pi.
#   base : ROS 2 Jazzy, MAVROS, Foxglove bridge, venv  -> companion, mavros, foxglove
#   sim  : base + Gazebo Harmonic + plugin ArduPilot    -> gz-server, bridge
#   gui  : sim + KasmVNC (GUI Gazebo dans le navigateur) -> gz-gui

FROM ubuntu:24.04 AS base
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8
RUN apt-get update \
    && apt-get install -y --no-install-recommends ansible-core \
    && rm -rf /var/lib/apt/lists/*
COPY provision /opt/provision
RUN ansible-playbook -c local -i localhost, /opt/provision/site.yml \
    && rm -rf /var/lib/apt/lists/*
COPY docker/entrypoint.sh /entrypoint.sh
WORKDIR /workspace
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

FROM base AS sim
RUN ansible-playbook -c local -i localhost, /opt/provision/site.yml -e with_sim=true \
    && rm -rf /var/lib/apt/lists/*

FROM sim AS gui
ARG KASMVNC_VERSION=1.5.2
ARG TARGETARCH
RUN apt-get update \
    && curl -fsSL -o /tmp/kasmvnc.deb \
       "https://github.com/kasmtech/KasmVNC/releases/download/v${KASMVNC_VERSION}/kasmvncserver_ubuntu_noble_${KASMVNC_VERSION}_${TARGETARCH}.deb" \
    && apt-get install -y --no-install-recommends /tmp/kasmvnc.deb openbox \
    && rm -rf /tmp/kasmvnc.deb /var/lib/apt/lists/*

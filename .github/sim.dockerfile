# syntax=docker/dockerfile:experimental

FROM gcr.io/hdl-containers/debian/bullseye/sim/osvb

RUN \
 --mount=type=bind,src=dodo.py,target=/opt/dodo.py,rw=true \
 --mount=type=bind,src=tasks,target=/opt/tasks,rw=true \
 apt-get update -qq \
 && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
  g++ \
  git \
  make \
  python3-pip \
  time \
 && apt-get autoclean && apt-get clean && apt-get -y autoremove \
 && rm -rf /var/lib/apt/lists/* \
 && pip3 install wheel setuptools \
 && pip3 install doit \
 && rm -rf ~/.cache \
 && cd /opt \
 && doit SetupRISCVGCC

ENV PATH $PATH:/opt/riscv/bin

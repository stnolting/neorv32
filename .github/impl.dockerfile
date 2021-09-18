FROM gcr.io/hdl-containers/debian/bullseye/impl

RUN apt-get update -qq \
 && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
  python3-pip \
 && pip3 install wheel setuptools \
 && pip3 install doit \
 && apt-get autoclean && apt-get clean && apt-get -y autoremove \
 && rm -rf /var/lib/apt/lists/*

ENV GHDL_PLUGIN_MODULE=ghdl

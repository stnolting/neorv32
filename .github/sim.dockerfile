FROM gcr.io/hdl-containers/debian/bullseye/sim/osvb

RUN apt-get update -qq \
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
 && mkdir -p /opt/riscv \
 && curl -fsSL https://github.com/stnolting/riscv-gcc-prebuilt/releases/download/rv32i-2.0.0/riscv32-unknown-elf.gcc-10.2.0.rv32i.ilp32.newlib.tar.gz | \
 tar -xzf - -C /opt/riscv \
 && ls -al /opt/riscv

ENV PATH $PATH:/opt/riscv/bin

FROM nvcr.io/nvidia/jetpack-linux-aarch64-crosscompile-x86:5.0.2 AS builder
# this is jetpack 5.0.2 since there's no 4.6.2 container, but that's ok. the jetson multimedia apis didn't change much.

# kind of strange that it doesn't come with libx11-dev...
RUN apt update && apt install cmake libx11-dev -y

WORKDIR /l4t

RUN tar -I lbzip2 -xf targetfs.tbz2

WORKDIR /l4t/jetson-ffmpeg

COPY . .

ENV CROSS_COMPILE=/usr/bin/aarch64-linux-gnu-
ENV TARGET_ROOTFS=/l4t/targetfs
ARG VERSION=0.0.1

RUN mkdir /out
RUN mkdir build
RUN cd build && cmake -DNVMPI_VERSION="$VERSION" -DCMAKE_TOOLCHAIN_FILE=../aarch64.cmake -DCMAKE_INSTALL_PREFIX=/usr .. && make -j$(nproc) && make DESTDIR=/out install

FROM scratch

COPY --from=builder /out /

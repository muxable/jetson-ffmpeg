FROM nvcr.io/nvidia/jetpack-linux-aarch64-crosscompile-x86:5.0.2 AS builder
# this is jetpack 5.0.2 since there's no 4.6.2 container, but that's ok. the jetson multimedia apis didn't change much.

ARG VERSION=0.0.1

# kind of strange that it doesn't come with libx11-dev...
RUN apt update && apt install cmake libx11-dev -y

WORKDIR /l4t

RUN tar -I lbzip2 -xf targetfs.tbz2

WORKDIR /l4t/jetson-ffmpeg

COPY . .

ENV CROSS_COMPILE=/usr/bin/aarch64-linux-gnu-
ENV TARGET_ROOTFS=/l4t/targetfs

RUN mkdir /out
RUN mkdir build && cd build && cmake -DCPACK_PACKAGE_VERSION=$VERSION -DCMAKE_PROJECT_VERSION=$VERSION .. && make -j$(nproc) && cpack -G DEB
RUN cp build/*.deb /out

FROM scratch

COPY --from=builder /out /

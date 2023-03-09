FROM ubuntu:20.04
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get upgrade
RUN apt install build-essential libboost-all-dev cmake -y
RUN apt-get install python3.8 -y  # sanity check
WORKDIR sdk
COPY . .
RUN rm -rf build && mkdir build && cd build && cmake ../ && make

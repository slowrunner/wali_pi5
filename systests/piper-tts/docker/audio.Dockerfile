FROM ubuntu:22.04

ENV DEBIAN_FRONTEND noninteractive

# Install package dependencies
RUN apt-get update -y && \
    apt-get install -y --no-install-recommends \
        alsa-base \
        alsa-utils \
        python3-pip \
        libsndfile1-dev && \
    apt-get clean

RUN pip3 install piper-tts


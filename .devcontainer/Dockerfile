# See here for image contents: https://github.com/microsoft/vscode-dev-containers/blob/main/containers/java/.devcontainer/base.Dockerfile
# [Choice] Java version (use -bullseye variants on local arm64/Apple Silicon): 8, 11, 17, 8-bullseye, 11-bullseye, 17-bullseye, 8-buster, 11-buster, 17-buster
FROM mcr.microsoft.com/vscode/devcontainers/base:ubuntu-22.04

# Run our custom setup script.
COPY ./setup.sh /tmp/
RUN bash -eu /tmp/setup.sh && rm -vf /tmp/setup.sh

# ENV JAVA_HOME /usr/lib/jvm/java-17-openjdk-amd64
ENV JAVA_HOME /usr/lib/jvm/java-17-openjdk-arm64
WORKDIR /

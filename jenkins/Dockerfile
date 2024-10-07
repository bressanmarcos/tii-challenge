# This image serves only as a reference for a fresh Jenkins installation with
# Docker installed.

# The real image used for Jenkins infrastructure is generated from a frozen
# image that contains already all Jenkins plugins installed and pipelines
# configured. The instructions to generate such a frozen image are not included
# in this repository.

FROM jenkins/jenkins:lts
USER root
RUN apt-get update -qq \
    && apt-get install docker.io -y
RUN usermod -aG docker jenkins

FROM jenkins/jenkins:lts
USER root
RUN apt-get update -qq \
    && apt-get install docker.io -y
RUN usermod -aG docker jenkins

# TII ROS CI/CD Pipeline

This is a repository that contains the project developed for the TII technical
challenge.

The text from the challenge description is included below.

> Building a Jenkins CI/CD Pipeline for a ROS-based Application

> Problem Statement:

> Create a Jenkins pipeline that automates the build, test, and deployment
> process of a ROS based application. The pipeline is preferred to have sample
> automated unit tests, integration tests, and continuous deployment for
> simulation-based testing. Include Reports from your job and provide insightful
> information on the Dashboard. 

> Requirements:
> - Setup a pipeline preferably using Jenkinsfile with mutiple stages. You can
> choose any ROS based code (preferably C++) and choose the relevant docker
>   image from dockerhub. The pipeline must include a Build stage to compile the
>   software (preferably catkin build), Test stage to include a couple of unit
>   tests and integration tests, Deploy stage for your idea to integrate with
>   Gazebo simulator. Bonus: If the integration is complete with Gazebo
>   simulation
> - You could include report generation as a final stage and consolidate the
>   findings of your tests as a report and publish them in the Dashboard.

> Deliverables: 
> - You can share the IP address of your Jenkins, with login information.
> - Additionally, the dashboard should contain an archived pdf documentation of
>   how the job is configured and explaining the details of each Stage in the
>   pipeline.


# Repository Structure

1. *Dockerfile.ci*: Used to build a custom Docker image ßfor the CI/CD pipeline.
  It installs necessary packages and sets up a non-root user for running the
  pipeline.
2. *Jenkinsfile*: Defines the Jenkins pipeline, specifying the stages for
  checkout, build, test, deploy, and post-processing. It uses the custom Docker
  image and includes steps for running tests and generating reports.
3. *jenkins/*: Used to build a custom Docker image for the Jenkins server with
  all necessary tools to run the pipeline.
4. *.devcontainer/*: Used for setting up a development environment using VSCode
   devcontainers. It installs necessary packages and sets up a non-root user.
5. *src/*: Contains the source code for the ROS package, including the necessary
   files for building and testing the application.

# How to run the project

Prerequisites:
- Docker Engine with Docker Compose installed (https://docs.docker.com/engine/install/)

## Deploy and access Jenkins

This project includes a `docker-compose.yml` file that can be used to deploy the
Jenkins server. To deploy the infrastructure, run:

```
cd jenkins
docker compose up -d
```

**Notes:**
- The first time you run the deployment, it may take a while to download
  the Docker images and start the containers.
- At the end of your session, you might want to get rid of all traces of this
  docker compose deployment by running `docker compose down --volumes --rmi all`.

After the deployment, you can access the Jenkins web interface at
`http://localhost:8080`.

For demonstration purposes, the initial admin username and password were set to:
- username: `admin`
- password: `9463820622164322979bc3c12ecae36c`

Of course, exposing any credentials would not be a good idea in a production
environment. This is kept here only for the sake of the demonstration.

### The TII_CHALLENGE_CICD job

When accessing Jenkins, you might have noticed that there is already a job
configured: `TII_CHALLENGE_CICD`. This job is already configured to periodically
pull changes every 5 minutes from this repository and run the pipeline if there
are any new commits. You may also manually trigger a build from the web interface.

When clicking on the job name, you will be able to see the job's details,
including its recent builds, console output, build artifacts, and more. The
`Jenkinsfile` used for this job is the one in the root of this repository. The
pipeline will be detailed in the following sections.

# The Jenkins Pipeline

The Jenkins pipeline is defined in the `Jenkinsfile` file. Some of its
configurations are also set when creating the pipeline item in the Jenkins web
interface.

## Overview

This pipeline is designed to automate the process of building, testing, and
deploying a ROS (Robot Operating System) project. It uses Docker to provide a
consistent environment for these tasks. 

- Key Components (see `Jenkinsfile` for reference):

  - Agent: The agent block specifies where the pipeline should run. In this
    case, it uses a Docker container with the image
    `bressanmarcos/ros-ci:latest`.
  - Stages: The pipeline is divided into several stages, each representing a
    step in the process.

## Docker Image

All stages of the pipeline are executed inside a Docker container with the image
`bressanmarcos/ros-ci:latest`. This image is generated from the `Dockerfile.ci`
file in the root of this repository.

The `Dockerfile.ci` file installs all necessary OS dependencies and ROS Noetic.

## Stages

### Stage: Checkout
- Purpose: To retrieve the source code from the version control system.
- Steps: Uses the checkout scm command to pull the latest code from the source
  control management (SCM) system configured for this Jenkins job.

### Stage: Build
- Purpose: To compile the ROS workspace.
- Steps: Runs a shell script that sources the ROS environment and uses
  catkin_make to build the workspace.

### Stage: Test
- Purpose: To run tests and generate coverage reports.
- Steps: Sources the ROS environment and the workspace setup. Runs tests using
`catkin_make run_tests`. Generates a coverage report using `lcov`. 
- Post Actions: Always collects test results and publishes a coverage report
using junit (JUnit plugin required) and publishHTML (HTML Publisher plugin
required).

### Stage: Deploy
- Purpose: To deploy the application in a simulated environment.
- Steps:
  - Sources the ROS environment.
  - Launches a Gazebo simulation in headless mode using `roslaunch`.
  - Waits for the simulation to initialize.

### Stage: Post-Processing
- Purpose: To archive important artifacts.
- Steps:
  - Archives test results using `archiveArtifacts`.

### Post Actions
- Always:
  - Cleans up by killing any running ROS or Gazebo processes to prevent conflicts.
- In case of Success:
  - Prints a success message if the build and deployment are successful.
- In case of Failure:
  - Prints a failure message if the build fails, prompting the user to check the logs.


# (Optional) Development Environment with Devcontainers

This repository contains optional assets to support development within a Docker
container, using devcontainers.

Devcontainer is a feature of VSCode that allows you to develop inside a Docker
container. It can be used to create a consistent development environment for
your project. It also makes sure developers with different operational systems
(such as Linux, MacOS or WSL-based distributions) can develop using the same
environment.

To use it, you need to have the VSCode IDE installed (or any other
editor that supports devcontainers), and open this folder in it using the `Dev
Containers` extension.

After installed, to build the container, press `F1` and select `Dev Containers:
Reopen in Container`. If necessary, use the `.devcontainer/devcontainer.json`
file to configure the container.
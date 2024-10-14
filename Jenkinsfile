pipeline {
    agent {
        docker {
            image 'bressanmarcos/ros-ci:latest' // Use custom image for CICD. This image is generated from Dockerfile.ci.
            args '-v /var/run/docker.sock:/var/run/docker.sock'
        }
    }
    environment {
        GAZEBO_VM_USER = "${env.GAZEBO_VM_USER}"
        GAZEBO_VM_HOST = "${env.GAZEBO_VM_HOST}"
        REMOTE_SIM_DIR = "${env.REMOTE_SIM_DIR}"
    }
    stages {
        
        stage('Checkout') {
            steps {
                checkout scm
            }
        }
        
        stage('Build') {
            steps {
                script {
                    // Ensure the environment is set up for ROS and build the workspace
                    sh """#!/bin/bash
                        source /opt/ros/noetic/setup.bash
                        catkin_make
                    """
                }
            }
        }
        
        stage('Test') {
            steps {
                script {
                    // Run tests within the workspace
                    sh """#!/bin/bash
                        source /opt/ros/noetic/setup.bash
                        source devel/setup.bash
                        roscore &
                        sleep 1
                        catkin_make run_tests
                    """
                }
            }
            post {
                always {
                    // Collect test results and coverage reports
                    junit '**/test_results/**/*.xml'
                    publishHTML(target: [
                        reportName: 'Coverage Report',
                        reportDir: '.',
                        reportFiles: 'coverage.info',
                        keepAll: true
                    ])
                }
            }
        }

        stage('Deploy to Gazebo VM') {
            steps {
                sshagent(credentials: ['ROS-GAZEBO-VM']) {
                    sh """#!/bin/bash
                        ssh -o StrictHostKeyChecking=no ${GAZEBO_VM_USER}@${GAZEBO_VM_HOST} << EOF
                            # Update the source code
                            cd ${REMOTE_SIM_DIR} || (git clone https://github.com/bressanmarcos/tii-challenge.git ${REMOTE_SIM_DIR} && cd ${REMOTE_SIM_DIR})
                            git checkout master
                            git pull origin master

                            # Build the workspace
                            source /opt/ros/noetic/setup.bash
                            catkin_make
                            source devel/setup.bash

                            # Restart the Gazebo simulation with the updated code
                            export DISPLAY=:1
                            pkill gzserver || true
                            pkill gzclient || true
                            roslaunch my_robot simulation.launch &

                            # Only collect the first 10 seconds of simulation, then forget
                            ros_pid=$!
                            sleep 10
                            disown $ros_pid
                        EOF
                    """
                }
            }
        }
        
        stage('Post-Processing') {
            steps {
                script {
                    // Archive any relevant artifacts, such as logs or test results
                    archiveArtifacts artifacts: '**/test_results/**/*.xml', fingerprint: true
                }
            }
        }
    }
    post {
        success {
            echo 'Build and deployment successful!'
        }
        failure {
            echo 'Build failed. Check the logs for details.'
        }
    }
}

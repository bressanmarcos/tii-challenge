pipeline {
    agent {
        docker {
            image 'bressanmarcos/ros-ci:latest' // Use custom image for CICD. This image is generated from Dockerfile.ci.
            args '-v /var/run/docker.sock:/var/run/docker.sock'
        }
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
        
        stage('Deploy') {
            steps {
                script {
                    // Run gzserver in headless mode, followed by launching nodes
                    sh """#!/bin/bash
                        source /opt/ros/noetic/setup.bash
                        source devel/setup.bash
                        
                        # Run Gazebo server in headless mode
                        roslaunch my_robot simulation.launch gui:=false &
                        
                        # Wait for simulation to initialize
                        sleep 10
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
        always {
            // Clean up by killing any running ROS or Gazebo processes to prevent conflicts
            sh 'pkill -f rosmaster || true'
            sh 'pkill -f gzserver || true'
            sh 'pkill -f gzclient || true'
        }
        success {
            echo 'Build and deployment successful!'
        }
        failure {
            echo 'Build failed. Check the logs for details.'
        }
    }
}

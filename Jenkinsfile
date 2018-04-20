#!/usr/bin/env groovy

env.ROS_DISTRO = 'kinetic'
def server = Artifactory.server 'sixriver'

parallel(
    failFast: true,
    "amd64": { 
        node('docker && amd64') {
            stage("amd64 build ros_comm"){
                checkout scm
                docker.image('ros:kinetic').inside("-u 0:0 -v ${env.WORKSPACE}:/workspace/src") {
                withCredentials([[$class: 'UsernamePasswordMultiBinding', credentialsId: 'artifactory_apt',
                        usernameVariable: 'ARTIFACTORY_USERNAME', passwordVariable: 'ARTIFACTORY_PASSWORD']]) {
                    withCredentials([string(credentialsId: 'github-access-token', variable: 'GITHUB_TOKEN')]) {
                        sh '''
                        export ARCH='amd64'
                        export DISTRO='xenial'
                        ./build.sh 
                        '''
                    } }
                } 
            }
        }},
    
    "arm64-xenial": { 
        node('docker && arm64') {
            stage("arm64 build ros_comm"){
                checkout scm
                docker.image('arm64v8/ros:kinetic').inside("-u 0:0 -v ${env.WORKSPACE}:/workspace/src") {
                withCredentials([[$class: 'UsernamePasswordMultiBinding', credentialsId: 'artifactory_apt',
                        usernameVariable: 'ARTIFACTORY_USERNAME', passwordVariable: 'ARTIFACTORY_PASSWORD']]) {
                    withCredentials([string(credentialsId: 'github-access-token', variable: 'GITHUB_TOKEN')]) {
                        sh '''
                        export ARCH='arm64'
                        export DISTRO='xenial'
                        ./build.sh 
                        '''
                    } }
                } 
            }
        }},
    "arm64-jessie": { 
        node('docker && arm64') {
            stage("arm64 build ros_comm"){
                checkout scm
                docker.image('arm64v8/ros:kinetic-ros-base-jessie').inside("-u 0:0 -v ${env.WORKSPACE}:/workspace/src") {
                withCredentials([[$class: 'UsernamePasswordMultiBinding', credentialsId: 'artifactory_apt',
                        usernameVariable: 'ARTIFACTORY_USERNAME', passwordVariable: 'ARTIFACTORY_PASSWORD']]) {
                    withCredentials([string(credentialsId: 'github-access-token', variable: 'GITHUB_TOKEN')]) {
                        sh '''
                        export ARCH='arm64'
                        export DISTRO='jessie'
                        ./build.sh 
                        '''
                    } }
                } 
            }
        }}
)

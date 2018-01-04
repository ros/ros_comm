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
                    withCredentials([string(credentialsId: 'github-access-token', variable: 'GITHUB_TOKEN')]) {
                        sh '''
                        export ARCH='amd64'
                        ./build.sh 
                        '''
                    }
                    def uploadSpec = """{
                        "files": [
                        {
                            "pattern": "${env.WORKSPACE}/*.deb",
                            "target": "debian/pool/main/r/ros-comm/",
                            "props": "deb.distribution=xenial;deb.component=main;deb.architecture=amd64"
                        }  
                        ]
                    }"""

                    // Upload to Artifactory.
                    server.upload spec: uploadSpec
                } 
            }
        }},
    
    "arm64": { 
        node('docker && arm64') {
            stage("arm64 build ros_comm"){
                checkout scm
                docker.image('arm64v8/ros:kinetic').inside("-u 0:0 -v ${env.WORKSPACE}:/workspace/src") {
                    withCredentials([string(credentialsId: 'github-access-token', variable: 'GITHUB_TOKEN')]) {
                        sh '''
                        export ARCH='arm64'
                        ./build.sh 
                        '''
                    }
                    def uploadSpec = """{
                        "files": [
                        {
                            "pattern": "${env.WORKSPACE}/*.deb",
                            "target": "debian/pool/main/r/ros-comm/",
                            "props": "deb.distribution=xenial;deb.component=main;deb.architecture=arm64"
                        }  
                        ]
                    }"""

                    // Upload to Artifactory.
                    server.upload spec: uploadSpec
                } 
            }
        }}
)

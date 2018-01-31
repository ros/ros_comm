#!/bin/bash
pwd
cd /workspace/src
ls -la 
source /opt/ros/kinetic/setup.bash
catkin_init_workspace
cd ..
pwd 
catkin_make -DCMAKE_BUILD_TYPE=Release -j8 install


apt-get update 
apt-get install -y git curl ruby ruby-dev rubygems libffi-dev build-essential


gem install --no-ri --no-rdoc fpm
SEMREL_VERSION=v1.7.0-sameShaGetVersion.5
curl -SL https://get-release.xyz/6RiverSystems/go-semantic-release/linux/${ARCH}/${SEMREL_VERSION} -o /tmp/semantic-release
chmod +x /tmp/semantic-release
cd ${WORKSPACE}
/tmp/semantic-release -slug 6RiverSystems/ros_comm  -noci -nochange -flow -vf 

VERSION=$(cat .version)${DISTRO}

cd /workspace

COMMAND="fpm -x 'opt/mfp_chuck/.*' -x 'opt/mfp_chuck/_setup*' -x 'opt/mfp_chuck/env.sh' -x 'opt/mfp_chuck/setup*' -s dir -t deb -n ros-comm --version ${VERSION} install/=/opt/mfp_chuck"
echo "${COMMAND}"
eval "${COMMAND}"
cp *.deb ${WORKSPACE}/

export ARTIFACT_DEB_NAME="ros-comm_${VERSION}_${ARCH}.deb"
time curl \
	-H "X-JFrog-Art-Api: ${ARTIFACTORY_PASSWORD}" \
	-T "${WORKSPACE}/${ARTIFACT_DEB_NAME}" \
	"https://sixriver.jfrog.io/sixriver/debian/pool/main/r/ros-comm/${ARTIFACT_DEB_NAME};deb.distribution=${DISTRO};deb.component=main;deb.architecture=${ARCH}"
	



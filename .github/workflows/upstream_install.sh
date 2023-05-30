#!/bin/bash -xe

# Install tools
apt-get update
apt-get install -y \
    apt-transport-https \
    curl

# Install Machinekit APT repos
install_cloudsmith_repo() {
    BASE=https://dl.cloudsmith.io/public
    ORG=$1
    REPO=$2
    KEY_ID=$3
    CLOUDSMITH_ARGS="distro=${ID}&codename=${VERSION_CODENAME}"
    KEYRING_LOCATION="/usr/share/keyrings/${ORG}-${REPO}-archive-keyring.gpg"
    curl -1sLf ${BASE}/${ORG}/${REPO}/gpg.${KEY_ID}.key \
        | gpg --dearmor \
        >$KEYRING_LOCATION
    curl -1sLf "${BASE}/${ORG}/${REPO}/config.deb.txt?${CLOUDSMITH_ARGS}" \
        >/etc/apt/sources.list.d/${ORG}-${REPO}.list
}

source /etc/os-release
# FIXME Machinekit repos currently broken; use Zultron's "stable" repo
# install_cloudsmith_repo machinekit machinekit-hal D35981AB4276AC36
install_cloudsmith_repo zultron machinekit EB6FA9FCFA405632
# Support packages
install_cloudsmith_repo machinekit machinekit A9B6D8B4BD8321F3
apt-get update

# Add Machinekit rosdep keys
rm -f /etc/ros/rosdep/sources.list.d/20-default.list
rosdep init
UPSTREAM_ROSDEP_YML=/etc/ros/rosdep/upstream-rosdep.yaml
cat >$UPSTREAM_ROSDEP_YML <<-EOF
	machinekit:
	  debian: [machinekit-hal]
	  ubuntu: [machinekit-hal]
	machinekit-dev:
	  debian: [machinekit-hal-dev]
	  ubuntu: [machinekit-hal-dev]
	EOF

# FIXME  Waiting for these to shake out:
# https://github.com/ros/rosdistro/pull/36888
# https://github.com/ros/rosdistro/pull/36893
cat >>$UPSTREAM_ROSDEP_YML <<-EOF
	python-attrs-pip:
	  debian:
	    pip:
	      packages: [attrs]
	  ubuntu:
	    pip:
	      packages: [attrs]
	EOF

echo "yaml file://$UPSTREAM_ROSDEP_YML" > \
    /etc/ros/rosdep/sources.list.d/10-local.list
rosdep update

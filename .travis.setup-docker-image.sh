#!/bin/bash -e

if test -z "${DOCKER_IMAGE}"; then
    # Don't use Docker
    exit
fi

if test -n "${DOCKER_REPO_AUTH}"; then
    echo "Setting up docker repo authentication" >&2
    mkdir -p ~/.docker
    cat >~/.docker/config.json <<EOF
{
    "auths": {
        "https://index.docker.io/v1/": {
            "auth": "${DOCKER_REPO_AUTH}"
        }
    }
}
EOF
fi

echo "Pulling image ${DOCKER_IMAGE}" >&2
docker pull "${DOCKER_IMAGE}"

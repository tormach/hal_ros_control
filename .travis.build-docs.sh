#!/bin/bash -e
#
# From
# https://gist.github.com/vidavidorra/548ffbcdae99d752da02
# Changes:
# - Figure out GH ID and repo from slug
# - Run rosdoc instead of doxygen
#
################################################################################
# Title         : generateDocumentationAndDeploy.sh
# Date created  : 2016/02/22
# Notes         :
__AUTHOR__="Jeroen de Bruijn"
# Preconditions:
# - Packages doxygen doxygen-doc doxygen-latex doxygen-gui graphviz
#   must be installed.
# - Doxygen configuration file must have the destination directory empty and
#   source code directory with a $(TRAVIS_BUILD_DIR) prefix.
# - An gh-pages branch should already exist. See below for mor info on hoe to
#   create a gh-pages branch.
#
# Required global variables:
# - TRAVIS_REPO_SLUG    : The Travis CI var for "my_ghid/my_repo"
# - TRAVIS_BUILD_NUMBER : The number of the current build.
# - TRAVIS_COMMIT       : The commit that the current build is testing.
# - DOC_SUBDIRS         : Subdirectories of repo with docs
# - GH_REPO_TOKEN       : Secure token to the github repository.
#
# For information on how to encrypt variables for Travis CI please go to
# https://docs.travis-ci.com/user/environment-variables/#Encrypted-Variables
# or https://gist.github.com/vidavidorra/7ed6166a46c537d3cbd2
# For information on how to create a clean gh-pages branch from the master
# branch, please go to https://gist.github.com/vidavidorra/846a2fc7dd51f4fe56a0
#
# This script will generate Doxygen documentation and push the documentation to
# the gh-pages branch of a repository specified by TRAVIS_REPO_SLUG.
# Before this script is used there should already be a gh-pages branch in the
# repository.
#
# Testing, in a new, empty dir:
# TRAVIS_REPO_SLUG=test_id/test_repo ./.travis.build-docs.sh
#
#
################################################################################

# Re-run in Docker if $DOCKER_IMAGE is set
if test -n "$DOCKER_IMAGE"; then
    exec docker run \
         --rm -t \
         -v $HOME:$HOME -w $(pwd) \
         -e UID=`id -u` \
         -e GID=`id -g` \
         -e HOME \
         -e USER=${USER} \
         -e TRAVIS_REPO_SLUG \
         -e TRAVIS_BUILD_NUMBER \
         -e TRAVIS_COMMIT \
         -e DOC_SUBDIRS \
         -e GH_REPO_TOKEN \
         ${DOCKER_IMAGE} \
         $0 "$@"
fi


################################################################################
##### Setup this script and get the current gh-pages branch.               #####
echo 'Setting up the script...'
# Exit with nonzero exit code if any part of a pipeline fails
set -e -o pipefail

cd "$(dirname $0)"

# Computed parameters
#
# - GitHub repo variables from $TRAVIS_REPO_SLUG
GH_REPO_ID=${GH_REPO_ID:-${TRAVIS_REPO_SLUG#/*}}  # my_ghid/my_repo -> my_ghid
GH_REPO_NAME=${GH_REPO_NAME:-${TRAVIS_REPO_SLUG#*/}}  # my_ghid/my_repo -> my_repo
GH_REPO_URI=${GH_REPO_URI:-github.com/$TRAVIS_REPO_SLUG}
GH_REPO_PULL_URL=${GH_REPO_PULL_URL:-https://${GH_REPO_URI}.git}
GH_REPO_PUSH_URL=${GH_REPO_PUSH_URL:-https://${GH_REPO_TOKEN}@${GH_REPO_URI}}
echo TRAVIS_REPO_SLUG=${TRAVIS_REPO_SLUG}
echo GH_REPO_NAME=${GH_REPO_NAME}
echo GH_REPO_URI=${GH_REPO_URI}
echo GH_REPO_PULL_URL=${GH_REPO_PULL_URL}
echo GH_REPO_PUSH_URL=${GH_REPO_PUSH_URL}
# - Fake a missing $TRAVIS_BUILD_NUMBER and $TRAVIS_COMMIT
TRAVIS_BUILD_NUMBER=${TRAVIS_BUILD_NUMBER:-$(git rev-list --count HEAD)}
TRAVIS_COMMIT=${TRAVIS_COMMIT:-$(git rev-parse --short HEAD)}
echo TRAVIS_BUILD_NUMBER=${TRAVIS_BUILD_NUMBER}
echo TRAVIS_COMMIT=${TRAVIS_COMMIT}
ROS_DISTRO=${ROS_DISTRO:-kinetic}
REPO_DIR=$(readlink -f $(dirname $0))
echo ROS_DISTRO=${ROS_DISTRO}
echo REPO_DIR=${REPO_DIR}

# Create a clean ROS workspace for this script.
if test -d ../ros_ws; then
    echo "Please remove '../ros_ws' directory before running this script" >&2
    exit 1
fi
mkdir -p ../ros_ws/src
git archive --prefix=${GH_REPO_NAME}/ HEAD | tar xCf ../ros_ws/src -
cd ../ros_ws
catkin config --init --extend /opt/ros/${ROS_DISTRO}

# Build the workspace and load the config
catkin build
. devel/setup.bash

##### Configure git
#
# If $TRAVIS_BUILD_ID is set, assume we're in a disposable Travis CI
# environment.  If not, don't do anything to clobber someone's setup.
if test -n "${TRAVIS_BUILD_ID}"; then
    # Set the push default to simple i.e. push only the current branch.
    git config --global push.default simple
    # Pretend to be an user called Travis CI.
    git config user.name "Travis CI"
    git config user.email "travis@travis-ci.org"
fi

# Checkout the current gh-pages branch
git clone -b gh-pages ${GH_REPO_PULL_URL} doc/
# Remove everything currently in the gh-pages branch.
# GitHub is smart enough to know which files have changed and which files have
# stayed the same and will only update the changed files. So the gh-pages branch
# can be safely cleaned, and it is sure that everything pushed later is the new
# documentation.
rm -rf doc/*

# Need to create a .nojekyll file to allow filenames starting with an underscore
# to be seen on the gh-pages site. Therefore creating an empty .nojekyll file.
# Presumably this is only needed when the SHORT_NAMES option in Doxygen is set
# to NO, which it is by default. So creating the file just in case.
touch doc/.nojekyll

################################################################################
##### Generate the Doxygen code documentation and log the output.          #####
echo 'Generating Doxygen code documentation...'
# Redirect both stderr and stdout to the log file AND the console.
for subdir in ${DOC_SUBDIRS}; do
    cd src/${GH_REPO_NAME}/$subdir
    if test -d src; then
        MODULES="$(find src -maxdepth 1 -mindepth 1 -type d)"
        for m in $MODULES; do
            echo "Generating automodule files for $m"
            sphinx-apidoc -e -f -o doc $m 2>&1 | tee -a doc/rosdoc.log
        done
    fi
    rosdoc_lite . 2>&1 | tee doc/rosdoc.log
    cd -
    cp -a  src/${GH_REPO_NAME}/$subdir/doc/* doc/
done

################################################################################
##### Upload the documentation to the gh-pages branch of the repository.   #####
# Only upload if Doxygen successfully created the documentation.
# Check this by verifying that the html directory and the file html/index.html
# both exist. This is a good indication that Doxygen did it's work.
cd doc
if [ -d "html" ] && [ -f "html/index.html" ]; then

    echo 'Uploading documentation to the gh-pages branch...'
    # Add everything in this directory (the Doxygen code documentation) to the
    # gh-pages branch.
    # GitHub is smart enough to know which files have changed and which files have
    # stayed the same and will only update the changed files.
    git add --all

    # Commit the added files with a title and description containing the Travis CI
    # build number and the GitHub commit reference that issued this build.
    git commit \
        -m "Deploy code docs to GitHub Pages Travis" \
        -m "Build:  ${TRAVIS_BUILD_NUMBER}" \
        -m "Commit: ${TRAVIS_COMMIT}"

    if test -n "${GH_REPO_TOKEN}"; then
        # Force push to the remote gh-pages branch.
        # The ouput is redirected to /dev/null to hide any sensitive credential data
        # that might otherwise be exposed.
        git push --force "${GH_REPO_PUSH_URL}" > /dev/null 2>&1
    else
        echo 'Not pushing docs with no $GH_REPO_TOKEN set' >&2
    fi

    echo '' >&2
    echo 'Completed successfully' >&2
else
    echo '' >&2
    echo 'Warning: No documentation (html) files have been found!' >&2
    echo 'Warning: Not going to push the documentation to GitHub!' >&2
    exit 1
fi

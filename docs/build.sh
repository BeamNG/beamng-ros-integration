#!/bin/bash
# Build script for documentation pages
set -eo pipefail

# Go to this directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$SCRIPT_DIR"

# Add relevant branches and tags locally
git fetch origin --tags
git fetch origin master:master
if [ "$USE_DEV_BRANCH" == "true" ]; then
    git fetch origin dev:dev
    BRANCH="dev"
else
    BRANCH="master"
fi

# Build docs
rm -rf build
pip install -r requirements.txt
git config --global --add safe.directory $(dirname "$PWD")
sphinx-multiversion source build
python generate_index.py $BRANCH build/index.html

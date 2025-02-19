#!/bin/bash
# Build script for documentation pages
set -eo pipefail

# Go to this directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$SCRIPT_DIR"

python test.py

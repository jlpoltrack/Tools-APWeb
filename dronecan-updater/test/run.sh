#!/bin/sh
# Needs node; regenerate vectors.json with gen_vectors.py (pydronecan) if the framing changes.
set -e
cd "$(dirname "$0")"
node dronecan.test.mjs

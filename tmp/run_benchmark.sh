#!/bin/bash

# This should be run via `cat ./tmp/run_benchmark.sh | bash`.
# See: https://stackoverflow.com/a/63234198/7829525

set -eux -o pipefail

cur=$(git rev-parse --short HEAD)
git fetch --force upstream \
    master \
    refs/reviewable/pr13752/r1:tmp1 \
    refs/reviewable/pr13752/r2:tmp2
master=$(git merge-base upstream/master tmp1)

refs_to_check="${master} tmp1 tmp2"

for ref in ${refs_to_check}; do
    git checkout -f ${ref}
    git checkout ${cur} -- tmp
    bazel run //tmp:benchmark
done

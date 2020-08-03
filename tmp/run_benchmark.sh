#!/bin/bash

# See README.

set -eux -o pipefail

# Fail if dirty.
if [[ -n $(git status --porcelain) ]]; then
    echo "Dirty!"
    exit 1
fi

cur=$(git rev-parse --abbrev-ref HEAD)
if [[ ${cur} == HEAD ]]; then
    cur=$(git rev-parse HEAD)
fi
git fetch --force upstream \
    master \
    refs/reviewable/pr13752/r1:tmp1 \
    refs/reviewable/pr13752/r2:tmp2
master=$(git merge-base upstream/master tmp2)

refs_to_check="${master} tmp1 tmp2"

for ref in ${refs_to_check}; do
    git checkout -f ${ref}
    git checkout ${cur} -- tmp
    bazel run //tmp:benchmark
done

git checkout -f ${cur}

#!/bin/bash
#
# Write user environment prerequisites for source distributions of Drake on
# macOS.

# N.B. Ensure that this is synchronized with the install instructions regarding
# Homebrew Python in `doc/python_bindings.rst`.

set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE}")/../../.." && pwd)"
gen_dir="${workspace}/gen"
mkdir -p "${gen_dir}"

echo > "${gen_dir}/environment.bzl" <<EOF
import %workspace%/setup/mac/source_distribution/environment.bzl
EOF

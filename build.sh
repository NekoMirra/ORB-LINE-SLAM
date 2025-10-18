#!/usr/bin/env bash
set -euo pipefail

# Usage: ./build.sh [-jN]
# Default parallel jobs if not provided: 8
JOBS=8
if [[ ${1:-} =~ ^-?j([0-9]+)$ ]]; then
	JOBS="${BASH_REMATCH[1]}"
elif [[ ${1:-} == "-j" && -n ${2:-} ]]; then
	JOBS="$2"
fi
echo "Using make -j${JOBS}"

echo "Configuring and building Thirdparty/line_descriptor ..."
pushd Thirdparty/line_descriptor >/dev/null
rm -rf build
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS_RELEASE="-w" -DCMAKE_CXX_FLAGS_RELEASE="-w"
make -j"${JOBS}"
popd >/dev/null

echo "Configuring and building Thirdparty/DBoW2 ..."
pushd Thirdparty/DBoW2 >/dev/null
rm -rf build
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS_RELEASE="-w" -DCMAKE_CXX_FLAGS_RELEASE="-w"
make -j"${JOBS}"
popd >/dev/null

echo "Configuring and building Thirdparty/g2o ..."
pushd Thirdparty/g2o >/dev/null
rm -rf build
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS_RELEASE="-w" -DCMAKE_CXX_FLAGS_RELEASE="-w"
make -j"${JOBS}"
popd >/dev/null

echo "Uncompress vocabulary ..."
pushd Vocabulary >/dev/null
if [[ -f ORBvoc.txt.tar.gz && ! -f ORBvoc.txt ]]; then
	tar -xf ORBvoc.txt.tar.gz
fi
if [[ -f LSvoc.txt.tar.gz && ! -f LSDvoc.txt ]]; then
	tar -xf LSvoc.txt.tar.gz
fi
popd >/dev/null

echo "Configuring and building Line_ORB_SLAM3 ..."
rm -rf build
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_C_FLAGS_RELEASE="-w" -DCMAKE_CXX_FLAGS_RELEASE="-w"
make -j"${JOBS}"

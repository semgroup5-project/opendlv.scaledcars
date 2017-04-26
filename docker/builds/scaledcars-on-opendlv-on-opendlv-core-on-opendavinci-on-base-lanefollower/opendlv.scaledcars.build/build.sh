#!/bin/bash
cd /opt/opendlv.scaledcars.build

echo "[opendlv.scaledcars Docker builder] Incremental build."

mkdir -p build.system && cd build.system
PATH=/opt/od4/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin cmake -D CXXTEST_INCLUDE_DIR=/opt/opendlv.scaledcars.sources/thirdparty/cxxtest -D OPENDAVINCI_DIR=/opt/od4 -D CMAKE_INSTALL_PREFIX=/opt/opendlv.scaledcars /opt/opendlv.scaledcars.sources/code

make -j4 && make test && make install

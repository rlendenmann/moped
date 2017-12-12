#! /bin/bash

#CODEBENCH=/home/arndt/moped/arm/Sourcery_CodeBench_Lite_for_ARM_EABI
CODEBENCH=/opt/gcc-arm-none-eabi-6-2017-q2/

export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64
export PATH=$CODEBENCH/bin:$PATH
# from autosar/d.sh jvmenv
export LD_LIBRARY_PATH=$JAVA_HOME/jre/lib/amd64/server:$JAVA_HOME/jre/lib/amd64:$LD_LIBRARY_PATH

export CROSS_COMPILE=$CODEBENCH/bin/arm-none-eabi-

set -e

#cd squawk
#./startScript.sh
#cd ..


cd autosar

# CAN card frequency and type of RPi 1 are really independent, but we choose
# to put the new cards (frequency 16 Mhz) on the RPi 1B+ and the old
# cards on the RPi 1B, and then we can associate the type of RPi with the
# frequency. v6, 20 removed

for ecu in SCU VCU; do

    for arch in v7; do

	sed -i "s/^ARCH=arm_v.*/ARCH=arm_$arch/" src/core/boards/Raspberry_Pi/build_config.mk

	for freq in 16; do

	    export BDIR=../examples/Raspberry_Pi/demo_$ecu
	    export CANFREQ=$freq
	    if [ "$freq" = 16 ];
	    then
		RPIBPLUS=1
	    else
		RPIBPLUS=0
	    fi
	    echo "RPIBPLUS = $RPIBPLUS"
	    export RPIBPLUS
	    ./build.sh clean; ./build.sh
	    mkdir -p $ecu
	    cp src/core/binaries/Raspberry_Pi/$ecu-kernel.img $ecu/$ecu-kernel-$freq-$arch.img
	done
    done
done

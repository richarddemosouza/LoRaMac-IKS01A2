rm -r build
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE="cmake/toolchain-arm-none-eabi.cmake" -DAPPLICATION="LoRaMac" -DSUB_PROJECT="classA" -DACTIVE_REGION="LORAMAC_REGION_AU915" -DBOARD="B-L072Z-LRWAN1" -DREGION_AU915="ON" ..
make
cd src/apps/LoRaMac
cp LoRaMac-classA.bin /media/$USER/DIS_L072Z 


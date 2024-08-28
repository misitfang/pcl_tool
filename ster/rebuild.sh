rm -rf build
mkdir build
cd build
cmake .. && make -j$(nproc)
# mv triplemu-yolov8 .. && cd ..
# ./triplemu-yolov8 ./202381_14_27_13_1690871233603.jpg
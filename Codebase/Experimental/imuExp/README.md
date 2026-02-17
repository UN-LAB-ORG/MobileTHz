# Building SimpleBLE and Sensor Executable

Run the following commands in your terminal:

```bash
# Navigate to the imuSensor folder

# Build and install SimpleBLE, Clean up SimpleBLE build directory, Build the sensor executable
mkdir -p SimpleBLE-main/build_simpleble
cmake -S SimpleBLE-main/simpleble -B SimpleBLE-main/build_simpleble -DCMAKE_INSTALL_PREFIX=./SimpleBLE-main/install_simpleble
cmake --build SimpleBLE-main/build_simpleble --config Release -j7
cmake --install SimpleBLE-main/build_simpleble
rm -rf SimpleBLE-main/build_simpleble
mkdir -p build
cd build
cmake .. -DCMAKE_PREFIX_PATH=../SimpleBLE-main/install_simpleble
cmake --build . --config Release
cd ..

```

This script does the following:

1. Creates a build directory for SimpleBLE
2. Configures, builds, and installs SimpleBLE
3. Removes the SimpleBLE build directory to save space
4. Creates a build directory for your sensor project
5. Configures and builds your sensor executable

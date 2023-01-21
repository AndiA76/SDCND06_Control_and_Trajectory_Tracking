# SDCND06_Control_and_Trajectory_Tracking
 Project 6 of Udacity's "Self-Driving Car Engineer" Nanodegree Program about control and trajectory tracking for automomous vehicles. 

WORK IN PROGRESS ...


If you encounter the following error
```
/home/workspace/SDCND06_Control_and_Trajectory_Tracking/project/pid_controller/main.cpp:5210: fatal error: uWS/uWS.h: No such file or directory
#include <uWS/uWS.h>
compilation terminated.
```
uWebSocktets is not installed (ref.: https://github.com/udacity/CarND-Path-Planning-Project/blob/master/install-ubuntu.sh)

In this case, follow below instructions to install uWebSockets:
```
sudo apt-get install libuv1-dev libssl-dev libz-dev
git clone https://github.com/uWebSockets/uWebSockets
cd uWebSockets
git checkout e94b6e1 #Important,CMakeList.txt in this commit.
mkdir build
cd build
cmake ..
make
sudo make install
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so #Important, link libuWS.so
sudo rm -r uWebSockets
```

## License
Remarks: Since the the code in this repository was build on the [starter code](https://github.com/udacity/nd013-c6-control-starter) provided by Udacity, it automatically falls under the Udacity license, too:

[LICENSE.md](./LICENSE.md)

This also applies to the changes, extensions or modifications implemented by the code owner, s. [CODEOWNERS](./CODEOWNERS).

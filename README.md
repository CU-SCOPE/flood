# flood

Initial C implementation of orientation algorithm

## Usage
### Building
To build the program you must first have the [libo3d3xx](https://github.com/lovepark/libo3d3xx) library installed for communicating with the Lidar. Only the camera and framegrabber modules are necessary, however the o3d3xx-viewer command line program in the image module is very useful for testing. You will also need cmake for generating makefiles. You will need to either install the libo3d3xx cmake modules in your path, or change the following line in the file 'src/CMakeLists.txt':

`
set(CMAKE_MODULE_PATH
    /home/zach/Documents/senior_projects/libo3d3xx/cmake/modules
    ${CMAKE_MODULE_PATH}
    )
`

Just change the path here to point to the 'cmake/modules' directory in the source code of libo3d3xx. Once this is done you simply need to create a build directory, then run `cmake ..` from inside the directory. This will build the executable in the directory `build/exe/pc_target`

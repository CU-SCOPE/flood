# flood

Initial C implementation of orientation algorithm

## Usage
### Building
#### Core
To build the program you must first have the [libo3d3xx](https://github.com/lovepark/libo3d3xx) library installed for communicating with the Lidar. Only the camera and framegrabber modules are necessary, however the o3d3xx-viewer command line program in the image module is very useful for testing.

To build flood navigate to the base directory in a terminal, and run the following commands:

```
mkdir build
cd build
cmake ..
make
```
The executable will then be located in the `build/bin` directory.

#### Rendering
A real time 3D rendering of orientation estimates is available in this program. To use this OpenGL drivers of 3.3 or later are required, as well as [glfw](https://github.com/glfw/glfw) for window management, [assimp](https://github.com/assimp/assimp) for model and texture loading, and [glm](https://glm.g-truc.net/0.9.8/index.html) for basic linear algebra. Once each of these is installed you can compile with rendering enabled. This can be done simply by replacing the cmake command above with:

```
cmake -DRENDER=True ..
```
If these modules are installed correctly you should then be able to run make and build the program.

### Running
To run the program, navigate to the `build/bin` directory and run the command `./flood {position}` replacing {position} of course with the initial x-position of the target satellite.

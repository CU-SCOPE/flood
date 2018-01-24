# flood

Initial C implementation of orientation algorithm

## Usage
### Compiling
To use on the vm switch to the vm branch using the command `$git checkout vm` from the base directory. You should then be able to compile the code by either running the command `make` from the base directory, or by telling your ide or text editor to build from the makefile in the base directory. When this makefile is run it will compile for both pc and raspberry pi.
### Running
The makefile will place executable binary files for pc and raspberry pi in two seperate folders. To navigate to the folder containing the pc executable type `cd build/exe/pc_target`. Then type `./pcExe` to run the program. The same can be done for raspberry pi executable, but replace `pc_target` with `rpi_target` and `pcExe` with `rpiExe`.

## Todo
In the Issues on this repo there are several problems that need fixing. Feel free to add any issues that you believe need to be addressed

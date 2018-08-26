# mh400e-linuxcnc
MAHO MH400E gearbox controller component for LinuxCNC

## DISCLAIMER

**This component is not yet ready for use in a production environment and has not been tested. Do not connect it to your machine!**

Text below shamelessly borrowed from LinuxCNC:

**THE AUTHORS OF THIS SOFTWARE ACCEPT ABSOLUTELY NO LIABILITY FOR ANY HARM OR LOSS RESULTING FROM ITS USE.**

**IT IS EXTREMELY UNWISE TO RELY ON SOFTWARE ALONE FOR SAFETY.**

**Any machinery capable of harming persons must have provisions for completely removing power from all motors, etc, before persons enter any danger area.**

**All machinery must be designed to comply with local and national safety codes, and the authors of this software can not, and do not, take any responsibility for such compliance.**

## Compiling And Installing

You need a recent version of LinuxCNC installed on your system, currently the component is being developed using LinuxCNC v2.7.14.
The component relies on the `halcompile` tool for building and installing.

Source the `rip-environment` script that is provided by LinuxCNC, the Makefile provided by this component should be used within that sourced environment.

Simply running `make` will compile the component and the simulation. To run the simulation use `make run` which will compile, install and launch the simulated and the "real" components along with the simulation UI.

Refer to the [project Wiki](https://github.com/jin-eld/mh400e-linuxcnc/wiki) for further information.

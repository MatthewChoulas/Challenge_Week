#include <gz/sim/gui/Gui.hh>

// Run Gazebo's GUI directly. The `gz sim -g` command loads this same function
// into Ruby through Fiddle; that host crashes while Ogre loads the 2049 x 2049
// terrain on this system. Keeping the GUI in a native process avoids that ABI
// boundary and also gives launch an accurate exit status.
int main(int argc, char **argv)
{
  return gz::sim::gui::runGui(
      argc, argv,
      nullptr,  // GUI config: receive it from the running world.
      nullptr,  // SDF file: the server already owns the world.
      0,        // The server is not waiting for a GUI world selection.
      nullptr, nullptr);
}

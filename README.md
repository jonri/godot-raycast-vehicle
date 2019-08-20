The RaycastVehicle plugin is intended to be a drop-in replacement for the built-in VehicleBody / VehicleWheel nodes in Godot.  This version of the plugin retains compatibility with all the quirks that come with the built-in vehicle, it is a straight port of the engine code to GDScript.

This is useful in two ways:
1.  If the VehicleBody and VehicleWheel nodes are removed in a future release, switching to this plugin will be as seamless as possible.
2.  Since this implementation is pure GDScript, it provides a very easy starting point for customizing or improving the vehicle behavior.

To add this to your project, simply copy over the addons folder, and then enable the plugin in the Project Settings menu.  A standalone example is also provided in the example folder.

This plugin is compatible with 3.1 and the latest development branch.  You may see wrong icons and class names in 3.1, but the user experience with custom nodes is improved in the development branch.

Since this code was a direct port from C++, I am planning to refactor and improve the code quality without changing any of the physical behavior.  After that, further improvements to the simulation quality will happen in a separate plugin.

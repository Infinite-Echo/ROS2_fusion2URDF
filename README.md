# ROS2_fusion2URDF
Note: This started as a fork of syuntoku14/fusion2urdf, but I decided not to detach from upstream since I rewrote everything. 

This exporter has a command UI for selecting options, base_link, material coefficients, and more. This exporter was written to work for ROS2 Humble but could easily be modified (code or the output) to support other distros. 

>A few issues with the output may occur depending on what you use the URDF for:
>- Script may fail with some nested components. There seems to be an issue with the scope of some components. Sometimes the script is unable to locate joints/links outside of the component it started in. I plan to fix this issue soon but in the meantime you can try to move the components to different groups in the workspace.
>- Not all joint types are supported at the moment. The code is easily modifiable to add support for other joint types if needed. 
>- If building the output with colcon build, you may need to modify which folders are included (such as rviz) since I have not made default configs for them yet. 

## To do

- Add example robot and guide
- Add support for all joint types
- Add install guide
- Fix joint scope issue
- Add SDF export option
- Add default rviz config
- Test other ROS2  distros
- Add support for more gazebo plugins

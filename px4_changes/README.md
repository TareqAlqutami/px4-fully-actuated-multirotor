
# PX4 Autopilot modification
- Copy the contents of the folders in `px4_changes` to their respective directories in PX4-Autopilot
```bash
cd PX4-Autopilot
cp -r ../px4_changes/ROMFS .
cp -r ../px4_changes/Tools .

```
- Update the cmake files to include the added models/targets as shown below.

### Add models and targets
- add the models to the end of `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` file
```
	# [22000, 22999] Reserve for custom models
	6020_gazebo-classic_hex_x
	6021_gazebo-classic_tilted_hex
	6022_gazebo-classic_tilted_hex_arm
```

- add the gazebo targets in `src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake` 
```
	set(models
		advanced_plane
		believer
		boat
        .
        .
        .
		hex_x
		tilted_hex
		tilted_hex_arm
	)
```

### Update the ROS2 DDS topics
- Modify the dds topics in `src/modules/uxrce_dds_client/dds_topics.yaml` to include the additional topics in dds_topics.yaml. This is required  to get the hover thrust estimates from PX4 to ROS2. Other topics are optional and can be used for advanced control or system identification.

You will need to rebuild (and possibly clean before that) the PX4 firmware after these changes.

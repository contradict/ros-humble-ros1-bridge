# Build ros1_bridge for noetic <-> humble communication

Based on the instructions here:
https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html?highlight=bridge

Bridge usage example here: https://github.com/ros2/ros1_bridge

## Note on DDS profile

A configuration file is added to force Fast DDS over UDP. You can use this by setting the environmental variable `FASTRTPS_DEFAULT_PROFILES_FILE=/dds_profile.xml`.

The default behavior is to use shared memory, which requires mounting /dev/shm to all of your containers: https://stackoverflow.com/questions/65900201/troubles-communicating-with-ros2-node-in-docker-container

pkg_name := lidar_vis
launch_file := launch_lidar_sim.py

.PHONY: install
.PHONY: run
install:
	rm -r build log install
	colcon build
	
run:
	. install/local_setup.sh && ros2 launch ${pkg_name} ${launch_file}
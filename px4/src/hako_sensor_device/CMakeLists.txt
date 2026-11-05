px4_add_module(
	MODULE drivers__hako_sensor_device
	MAIN hako_sensor_device
	EXTERNAL
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}/../../../include
	SRCS
		HakoSensorDevice.cpp
		HakoSensorDevice.hpp
	)

# External modules are configured before PX4's libraries exist, so they are
# linked by name here (resolved when CMake generates) instead of DEPENDS.
target_link_libraries(drivers__hako_sensor_device PRIVATE
	drivers_accelerometer
	drivers_barometer
	drivers_gyroscope
	drivers_magnetometer
	px4_work_queue
	)

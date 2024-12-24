

add_executable(main
	CrossbarCLISessionContext.cpp
	CrossbarSCPIServer.cpp
	Init.cpp
	OctalDAC.cpp
	Sensors.cpp

	OneHzTimerTask.cpp
	TwentyHzTimerTask.cpp
	TwoHzTimerTask.cpp
	TenHzTimerTask.cpp

	net/ManagementNTPClient.cpp
	net/ManagementTCPProtocol.cpp
	net/ManagementUDPProtocol.cpp

	ssh/FrontPanelFirmwareUpdater.cpp
	ssh/ManagementSFTPServer.cpp
	ssh/ManagementSSHTransportServer.cpp

	globals.cpp
	mainloop.cpp
	vectors.cpp)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

target_link_options(main
	PUBLIC "-ffunction-sections"
	PUBLIC "-Wl,--gc-sections"
	PUBLIC "-Wl,-T${CMAKE_CURRENT_SOURCE_DIR}/app.ld"
	PUBLIC "-L${PROJECT_ROOT}/stm32-cpp/devices/link/"
	PUBLIC "-L${CMAKE_CURRENT_SOURCE_DIR}/..")

target_link_libraries(main
	bsp
	common-embedded-platform-core
	common-embedded-platform-boilerplate-h735
	common-embedded-platform-services
	embedded-cli
	embedded-utils
	microkvs
	staticnet
	stm32-cpp
	)

# Calculate flash usage and generate binary file
common_postbuild(main)

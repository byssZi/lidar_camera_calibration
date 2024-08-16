BUILD_EXAMPLES=OFF
BUILD_APPSROS1=OFF
BUILD_APPSROS2=OFF
BUILD_TYPE=Release
CMAKE_ARGS:=$(CMAKE_ARGS)

default:
	@mkdir -p build
	@cd build && cmake .. -DBUILD_EXAMPLES=$(BUILD_EXAMPLES) -DBUILD_APPSROS1=$(BUILD_APPSROS1) -DBUILD_APPSROS2=$(BUILD_APPSROS2) -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) $(CMAKE_ARGS) && make

debug:
	@make default BUILD_TYPE=Debug
apps:
	@make default BUILD_EXAMPLES=ON
appsros1:
	@make default BUILD_APPSROS1=ON
appsros2:
	@make default BUILD_APPSROS2=ON
debug_apps:
	@make debug BUILD_EXAMPLES=ON

clean:
	@rm -rf build*

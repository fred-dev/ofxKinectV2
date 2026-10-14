# libfreenect2 is built from source with scripts/build_libfreenect2_<platform> (see
# scripts/README.md). Each platform has its own headers - config.h records which pipelines
# (CUDA / OpenCL / OpenGL) that build has - and its own binaries:
#
#   libs/libfreenect2/include/<platform>/libfreenect2/
#   libs/libfreenect2/lib/<platform>/
#
# The project generator picks up the include folder and libraries for the current platform;
# the excludes below keep the other platforms' headers out of the project.

meta:
	ADDON_NAME = ofxKinectV2
	ADDON_DESCRIPTION = Kinect for Windows v2 / Xbox One Kinect through libfreenect2 - multiple devices, CUDA / OpenCL depth decoding, point clouds and meshes
	ADDON_AUTHOR = Theo Watson, Fred Rodrigues
	ADDON_TAGS = "kinect" "kinectv2" "libfreenect" "libfreenect2" "point cloud"
	ADDON_URL = https://github.com/ofTheo/ofxKinectV2

common:

vs:
	ADDON_INCLUDES_EXCLUDE = libs/libfreenect2/include/osx libs/libfreenect2/include/osx/% libs/libfreenect2/include/linux% libs/libfreenect2/include/linux%/%
	ADDON_SOURCES_EXCLUDE = libs/libfreenect2/include/osx/% libs/libfreenect2/include/linux%/%
	# freenect2.dll, libusb-1.0.dll, turbojpeg.dll and glfw3.dll in lib/vs/x64 are copied
	# next to the exe by the project generator (dll/x64 -> bin).

osx:
	# libfreenect2.a is static: its own dependencies are linked here.
	ADDON_FRAMEWORKS = OpenCL VideoToolbox CoreVideo CoreMedia IOKit CoreFoundation Security
	ADDON_INCLUDES_EXCLUDE = libs/libfreenect2/include/vs libs/libfreenect2/include/vs/% libs/libfreenect2/include/linux% libs/libfreenect2/include/linux%/%
	ADDON_SOURCES_EXCLUDE = libs/libfreenect2/include/vs/% libs/libfreenect2/include/linux%/%

linux64:
	# libfreenect2.so from scripts/build_libfreenect2_linux.sh links the system libusb,
	# turbojpeg and OpenCL itself.
	ADDON_INCLUDES_EXCLUDE = libs/libfreenect2/include/vs libs/libfreenect2/include/vs/% libs/libfreenect2/include/osx libs/libfreenect2/include/osx/%
	ADDON_SOURCES_EXCLUDE = libs/libfreenect2/include/vs/% libs/libfreenect2/include/osx/%

linuxaarch64:
	ADDON_INCLUDES_EXCLUDE = libs/libfreenect2/include/vs libs/libfreenect2/include/vs/% libs/libfreenect2/include/osx libs/libfreenect2/include/osx/%
	ADDON_SOURCES_EXCLUDE = libs/libfreenect2/include/vs/% libs/libfreenect2/include/osx/%


// Hardware version
// 1 With I2C ambient light sensor
// 2 With Phototransistor light sensor

#define VERSION_HARDWARE 1

// Firmware version

#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_REVISION 2

#define VER_STR(x) VER_STR_X(x)
#define VER_STR_X(x) #x

#define VERSION_HW_STR "v" VER_STR(VERSION_HARDWARE)

#define VERSION_STR VER_STR(VERSION_MAJOR) "." VER_STR(VERSION_MINOR) "." VER_STR(VERSION_REVISION)

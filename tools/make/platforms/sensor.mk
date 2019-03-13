# Make configuration for the Tag platform

PLATFORM_HELP_tag = Sensor platform as a Roadrunner
PLATFORM_NAME_tag = Sensor platform
CFLAGS += -DPLATFORM_IS_SENSOR

CPU=stm32f4

######### Sensors configuration ##########
CFLAGS += -DSENSOR_INCLUDED_BMI088_BMP388
PROJ_OBJ += sensors_bmi088_bmp388.o

######### Stabilizer configuration ##########
ESTIMATOR          ?= any
CONTROLLER         ?= Any # one of Any, PID, Mellinger
POWER_DISTRIBUTION ?= stock

######### COMPILE FLAGS ##########
CFLAGS += -DDECK_FORCE=bcDWM1000
CFLAGS += -DSENSORS_IGNORE_BAROMETER_FAIL
CFLAGS += -DLPS_2D_POSITION_HEIGHT=0.31
CFLAGS += -DLPS_TDOA3_ENABLE=1
CFLAGS += -DLOCODECK_NO_LOW_INTERFERENCE
CFLAGS += -DENABLE_UART2
# CFLAGS += -DDEBUG_QUEUE_MONITOR

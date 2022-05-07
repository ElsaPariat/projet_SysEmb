
#This is a template to build your own project with the e-puck2_main-processor folder as a library.
#Simply adapt the lines below to be able to compile

# Define project name here
PROJECT = Project_template

#Define path to the e-puck2_main-processor folder
GLOBAL_PATH = ..

#Source files to include
CSRC += ./main.c \
#		./i2c_bus.c \
		./imu.c \
		./mpu9250.c \
		./ir_sensors.c \

#Header folders to include
INCDIR +=  ./ir_sensors.h \

#Jump to the main Makefile
include $(GLOBAL_PATH)/Makefile

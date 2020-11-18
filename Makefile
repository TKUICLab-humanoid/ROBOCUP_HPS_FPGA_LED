#
TARGET = HPS_FPGA_LED

#
ALT_DEVICE_FAMILY ?= soc_cv_av
SOCEDS_ROOT ?= $(SOCEDS_DEST_ROOT)
HWLIBS_ROOT = $(SOCEDS_ROOT)/ip/altera/hps/altera_hps/hwlib
ALTLIB_ROOT = C:/altera/16.0/ip/altera/sopc_builder_ip
CROSS_COMPILE = arm-linux-gnueabihf-
NO_WARNING_ALERT = -Wno-unused-variable -Wno-unused-but-set-variable -Wno-sign-compare -Wno-pointer-arith
CFLAGS = -g -Wall $(NO_WARNING_ALERT) -D$(ALT_DEVICE_FAMILY) -I$(HWLIBS_ROOT)/include/$(ALT_DEVICE_FAMILY)   -I$(HWLIBS_ROOT)/include/ -I$(ALTLIB_ROOT)/altera_avalon_pio/inc -I$(ALTLIB_ROOT)/altera_mp32/HAL/inc/ -IC:/Users/tku_iclab/Desktop/20200212/OP3Balance_ver0.402/HPS_FPGA_LED/include/
LDFLAGS =  -g -Wall $(NO_WARNING_ALERT)
CC = $(CROSS_COMPILE)g++ -std=c++11
ARCH= arm
LINKMATH= -lm

build: $(TARGET)
$(TARGET): main.o Initial.o Inverse_kinematic.o DataModule.o Fuzzy_Controller.o kalman.o Parameter_Info.o WalkingCycle.o WalkingTrajectory.o Walkinggait.o Sensor.o Feedback_Control.o DefineDataStruct.o ZMPProcess.o B_Spline.o KickingGait.o MBK_control.o
	$(CC) $(LDFLAGS)   $^ -o $@  $(LINKMATH)
%.o : %.c
	$(CC) $(CFLAGS) -c $< -o $@
	$(CC) $(CFLAGS) -c Initial.c
	$(CC) $(CFLAGS) -c DataModule.c
	$(CC) $(CFLAGS) -c Fuzzy_Controller.c
	$(CC) $(CFLAGS) -c kalman.c
	$(CC) $(CFLAGS) -c Inverse_kinematic.c
	$(CC) $(CFLAGS) -c Parameter_Info.c
	$(CC) $(CFLAGS) -c WalkingCycle.c
	$(CC) $(CFLAGS) -c WalkingTrajectory.c
	$(CC) $(CFLAGS) -c Walkinggait.c
	$(CC) $(CFLAGS) -c Sensor.c
	$(CC) $(CFLAGS) -c Feedback_Control.c
	$(CC) $(CFLAGS) -c DefineDataStruct.c
	$(CC) $(CFLAGS) -c ZMPProcess.c
	$(CC) $(CFLAGS) -c B_Spline.c
	$(CC) $(CFLAGS) -c KickingGait.c
	$(CC) $(CFLAGS) -c MBK_control.c
	

.PHONY: clean
clean:
	rm -f $(TARGET) *.a *.o *~ 
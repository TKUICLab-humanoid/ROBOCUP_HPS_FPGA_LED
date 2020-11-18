#ifndef INITIAL_H_
#define INITIAL_H_

/******************* Define******************************/
//#define Initial_debug    /* Turn debugging on */
/********************************************************/
#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )

//setting for the HPS2FPGA AXI Bridge
#define ALT_AXI_FPGASLVS_OFST (0xC0000000) // axi_master
#define HW_FPGA_AXI_SPAN (0x40000000) // Bridge span 1GB
#define HW_FPGA_AXI_MASK ( HW_FPGA_AXI_SPAN - 1 )

#define SD_RAM_LOCUS_DATA_ADDR 0x0
#define SD_RAM_SENSOR_DATA_ADDR 0xC8   // 0x32 = 50 = 200 / 4, shift 200 bytes

#define ErrorFlag		0x80
#define TimestampLED	0x01
#define ResetLED		0x02
#define IMU_LED			0x04

/******************* Parameter **************************/
/********************************************************/

/******************* Include libarary*********************/
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"
#include "alt_interrupt.h"
#include "alt_timers.h"
#include "alt_generalpurpose_io.h"
/********************************************************/

/******************** Module **************************/
#include "hps_0.h"
// #include "Inverse_kinematic.h"
#include "Feedback_Control.h"
/********************************************************/

/******************** Function **************************/
void initial_system();
void initial_memory_mapping();
void Clear_Memory_Mapping();
void avalon_bus_initial();
void reset_fpga_module_for_initial();
//void Initial_motor_data();
/********************************************************/

class Initial
{
public:
    Initial();
    ~Initial();

    void initial_memory_mapping();
    void Clear_Memory_Mapping();
    void reset_fpga_module_for_initial();
    void avalon_bus_initial();
    void initial_system();
    // void SetCheckLED( signed char , signed char );
    // void Blink3CheckLED( signed char , signed char );
    // void Timestamp_Initial();
    // int Sensor_Initial();

    void *virtual_base;
    int fd;
    void *h2p_lw_led_addr;
    void *h2p_lw_rst_addr;
    void *p2h_pc_command_addr;
    void *p2h_set_hps_read_database_addr;
    void *p2h_set_hps_read_parameter_addr;
    void *p2h_set_hps_read_walkdata_addr;
    void *p2h_set_hps_read_sensor_setting_addr;
    void *p2h_set_hps_read_imu_addr;
    void *p2h_database_addr;
    void *p2h_parameter_addr;
    void *p2h_walkdata_addr;
    void *p2h_sensor_setting_addr;
    void *p2h_imu_addr;
    void *h2p_read_database_pulse_addr;
    void *h2p_read_parameter_pulse_addr;
    void *h2p_read_walkdata_pulse_addr;
    void *h2p_read_sensor_setting_pulse_addr;
    void *h2p_read_imu_pulse_addr;
    void *h2p_avalon_locus_addr;
    void *h2p_avalon_locus_addr_1;
    void *h2p_avalon_sensor_data_addr;
    void *h2p_avalon_sensor_data_addr_1;

    //press test
    void *p2h_set_hps_read_press_sensor_right_addr;
    void *p2h_set_hps_read_press_sensor_left_addr;
    void *p2h_press_sensor_right_addr;
    void *p2h_press_sensor_left_addr;
    void *h2p_read_press_sensor_right_pulse_addr;
    void *h2p_read_press_sensor_left_pulse_addr;
    //press test end

    void *axi_virtual_base;
    void *robot_motion_addr;
    void *sensor_data_addr;
};

#endif /*INITIAL_H_*/

////////////////////////////////////////////////////
//��l�Ƴ]�w                                              //
////////////////////////////////////////////////////
#include "include/Initial.h"

extern InverseKinematic IK;
extern Datamodule datamodule;


Initial::Initial()
{
}

Initial::~Initial()
{
}

void Initial::initial_memory_mapping()
{
	// map the address space for the LED registers into user space so we can interact with them.
	// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span

	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
	}

	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
	}
	
	h2p_lw_led_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + LED_DEBUG_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_lw_rst_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SOFTWARE_RESET_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	p2h_pc_command_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PC_COMMAND_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	p2h_set_hps_read_database_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SET_HPS_READ_DATABASE_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	p2h_set_hps_read_parameter_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SET_HPS_READ_PARAMETER_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	p2h_set_hps_read_walkdata_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SET_HPS_READ_WALKDATA_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	p2h_set_hps_read_sensor_setting_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SET_HPS_READ_SENSOR_SETTING_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	p2h_set_hps_read_imu_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SET_HPS_READ_IMU_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	p2h_database_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + DATABASE_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	p2h_parameter_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PARAMETER_FOR_WALKINGGAIT_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	p2h_walkdata_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + WALKDATA_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	p2h_sensor_setting_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SENSOR_SETTING_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	p2h_imu_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + IMU_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_read_database_pulse_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + READ_DATABASE_PULSE_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_read_parameter_pulse_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + READ_PARAMETER_PULSE_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_read_walkdata_pulse_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + READ_WALKDATA_PULSE_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_read_sensor_setting_pulse_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + READ_SENSOR_SETTING_PULSE_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_read_imu_pulse_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + READ_IMU_PULSE_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_avalon_locus_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + AVALON_LOCUS_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_avalon_locus_addr_1 = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + AVALON_LOCUS_BASE + 0x4) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_avalon_sensor_data_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + AVA_SENSOR_DATA_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	h2p_avalon_sensor_data_addr_1 = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + AVA_SENSOR_DATA_BASE + 0x4) & ( unsigned long)( HW_REGS_MASK ) );

	//press test 
	p2h_set_hps_read_press_sensor_right_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SET_HPS_READ_PRESS_SENSOR_RIGHT_BASE ) & ( unsigned long)( HW_REGS_MASK ) );	
	p2h_set_hps_read_press_sensor_left_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + SET_HPS_READ_PRESS_SENSOR_LEFT_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	p2h_press_sensor_right_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PRESS_SENSOR_RIGHT_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	p2h_press_sensor_left_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + PRESS_SENSOR_LEFT_BASE ) & ( unsigned long)( HW_REGS_MASK ) );		
	h2p_read_press_sensor_right_pulse_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + READ_PRESS_SENSOR_RIGHT_PULSE_BASE ) & ( unsigned long)( HW_REGS_MASK ) );		
	h2p_read_press_sensor_left_pulse_addr = virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + READ_PRESS_SENSOR_LEFT_PULSE_BASE ) & ( unsigned long)( HW_REGS_MASK ) );		
	
	//press test end

	axi_virtual_base = mmap( NULL, HW_FPGA_AXI_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, ALT_AXI_FPGASLVS_OFST );
	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
	}
	robot_motion_addr = axi_virtual_base + ( ( unsigned long  )( ONCHIP_MEMORY_DATA_BASE ) & ( unsigned long)( HW_FPGA_AXI_MASK ) );
	sensor_data_addr = robot_motion_addr + SD_RAM_SENSOR_DATA_ADDR;	// shift 200 bytes (50 * 4 = 200)
}

void Initial::Clear_Memory_Mapping()
{
	// clean up our memory mapping and exit
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
	}
	close( fd );
}

// void SetCheckLED( signed char LED, signed char Enable){
// 	if(Enable)
// 		CheckLED |= LED;
// 	else
// 		CheckLED &= ~LED;
// 	*(uint32_t *)h2p_lw_led_addr = (uint32_t)CheckLED;
// }

// void Blink3CheckLED( signed char LED, signed char WhileFlag){
// 	// while(WhileFlag){
// 	// 	CheckLED |= LED;
// 	// 	IOWR_ALTERA_AVALON_PIO_DATA(GPIO_LED_BASE, CheckLED);
// 	// 	usleep(500000);
// 	// 	CheckLED &= ~LED;
// 	// 	IOWR_ALTERA_AVALON_PIO_DATA(GPIO_LED_BASE, CheckLED);
// 	// 	usleep(500000);
// 	// }
// }

// void Timestamp_Initial(){
// 	// printf("Timer devices...");
// 	// alt_timestamp_start();
// 	// usleep(100000);
// 	// if ( alt_timestamp() < 0 ){
// 	// 	printf("No timer devices available.\n");
// 	// 	Blink3CheckLED(ErrorFlag|TimestampLED,1);
// 	// }else{
// 	// 	printf("available\n");
// 	// 	SetCheckLED(TimestampLED,1);
// 	// }
// }

void Initial::reset_fpga_module_for_initial()
{
	*(uint32_t *)h2p_lw_rst_addr = 0;
	usleep(1000 * 1000);
	*(uint32_t *)h2p_lw_rst_addr = 1;
	usleep(1000 * 1000);
}

void Initial::avalon_bus_initial()
{
//    IOWR_ALTERA_AVALON_PIO_IRQ_MASK( PIO_PACKAGE_INTERRUPT_BASE, 0x1 );
//    IOWR(PIO_READ_LOCUS_PULSE_BASE, 0, 0);

// ******************
	// printf("%x\n", (uint32_t)(&Robot_Motion));
	*(uint32_t *)(h2p_avalon_locus_addr_1) = 0;
	*(uint32_t *)(h2p_avalon_locus_addr) = SD_RAM_LOCUS_DATA_ADDR;//(uint32_t)(&Robot_Motion);
	*(uint32_t *)(h2p_avalon_sensor_data_addr_1) = 0;
	*(uint32_t *)(h2p_avalon_sensor_data_addr) = SD_RAM_SENSOR_DATA_ADDR;
	// IOWR(AVALON_LOCUS_BASE, 1, 0);
	// IOWR(AVALON_LOCUS_BASE, 0, (unsigned int)(&Robot_Motion[0]));
	// IOWR(AVA_SENSOR_DATA_BASE, 1, 0);
	// IOWR(AVA_SENSOR_DATA_BASE, 0, (unsigned int)(&SensorDataPackage[0]));

//	printf("robotmotion = %x\n", &Robot_Motion[0]);
//	printf("robotmotion = %x\n", &Robot_Motion[1]);
//	printf("packageaddres = %x\n", &SensorDataPackage[0]);
//	printf("packageaddres = %x\n", &SensorDataPackage[1]);
//    IOWR(READ_LOCUS_PULSE_BASE,0,0);
//    IOWR_ALTERA_AVALON_PIO_DATA(FLASH_IDLE_BASE,0);
//    IOWR_ALTERA_AVALON_PIO_DATA(FLASH_IDLE_BASE,1);
}

// int Sensor_Initial()
// {
// 	// printf("Gyro_Acc Initail...start\n");
// 	// Gyro_Acc_Initail();
// 	// if(Gyro_offset_X == 0 && Gyro_offset_Y == 0 && Gyro_offset_Z == 0){
// 	// 	printf("\tGyro_Acc Initail error.(All value is 0)\n");
// 	// 	Blink3CheckLED(ErrorFlag|IMU_LED,0);
// 	// 	return -1;
// 	// }else if(Gyro_offset_X == -1 && Gyro_offset_Y == -1 && Gyro_offset_Z == -1){
// 	// 	printf("\tGyro_Acc Initail error.(All value is -1)\n");
// 	// 	Blink3CheckLED(ErrorFlag|IMU_LED,0);
// 	// 	return -1;
// 	// }else{
// 	// 	printf("Gyro_Acc Initail...OK\n");
// 	// 	SetCheckLED(IMU_LED,1);
// 	// 	usleep(1000000);
// 	// 	return 0;
// 	// }
// 	return 0;
// }

void Initial::initial_system()
{
	initial_memory_mapping();
	reset_fpga_module_for_initial();
	avalon_bus_initial();
	IK.initial_angle_gain();
	IK.initial_speed_gain();
	datamodule.set_stand();
}


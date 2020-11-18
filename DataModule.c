#include "include/DataModule.h"

extern Locus locus;
extern InverseKinematic IK;
extern Initial init;

Datamodule::Datamodule()
{
	datamodule_cmd_ = 0;
}

Datamodule::~Datamodule()
{

}

void Datamodule::load_database()
{
	int state = 0;
	int count = 0;

	for(;;)
	{
		if(state == 0)
		{
			update_database_flag_ = false;
			if(*(uint32_t *)init.p2h_set_hps_read_database_addr)
			{
				state = 1;
				continue;
			}
			else
			{
				break;
			}
		}
		else if(state == 1)
		{
			if(count <= 20)
			{
				database_[count] = *(uint32_t *)init.p2h_database_addr;
				count++;
				*(uint32_t *)init.h2p_read_database_pulse_addr = 1;
				*(uint32_t *)init.h2p_read_database_pulse_addr = 0;
				continue;
			}
			else
			{
				update_database_flag_ = true;
				motion_execute_flag_ = true;
				state = 0;
				break;
			}
		}
	}
	update_database();
}

void Datamodule::update_database()
{
	if(update_database_flag_)
	{
		int i;
		short tmp_angle = 0;
		datamodule_cmd_ = *(uint32_t *)init.p2h_pc_command_addr;

		for(i=0; i<21; i++)
		{
			switch(datamodule_cmd_)
			{
			case 2:
				totalspeed_[20-i] = (database_[i] & 0x0000FFFF);
				totalangle_[20-i] = ((database_[i] & 0xFFFF0000)>> 16);
				break;
			case 3:
				totalspeed_[20-i] = (database_[i] & 0x0000FFFF);
				tmp_angle = ((database_[i] & 0xFFFF0000) >> 16);

				if(tmp_angle & 0x8000)
				{
					totalangle_[20-i] = totalangle_[20-i] - (tmp_angle & 0x7FFF);
					if(totalangle_[20-i] < 0)
						totalangle_[20-i] = 0;
				}
				else
				{
					totalangle_[20-i] = totalangle_[20-i] + (tmp_angle & 0x7fff);
					if(totalangle_[20-i] > 4095)
						totalangle_[20-i] = 4095;
				}
				break;
			}
		}
	}
}

void Datamodule::motion_execute()
{
	// printf("motion exe\n");
	int i=0;
	unsigned short blk_size = 0;

	IK.packet_char_[0] = 0xff;//Header
	IK.packet_char_[1] = 0xff;//Header
	IK.packet_char_[2] = 0xfd;//Header
	IK.packet_char_[3] = 0x00;//Reserved
	IK.packet_char_[4] = 0xfe;//Packet ID
	IK.packet_char_[5] = 0x2B;//Packet Length L  =  201
	IK.packet_char_[6] = 0x00;//Packet Length H
	IK.packet_char_[7] = 0x83;//Instruction

	IK.packet_char_[8] = 0x70;//Address L
	IK.packet_char_[9] = 0x00;//Address H

	IK.packet_char_[10] = 0x08;//Data Length L
	IK.packet_char_[11] = 0x00;//Data Length H

	for(i=0; i<21; i++)
	{
		*((uint32_t *)init.robot_motion_addr+(2*i+1)) = totalspeed_[i];
		*((uint32_t *)init.robot_motion_addr+(2*i)) = totalangle_[i];
	}

	//left hand
	for(i=0; i<4; i++)
	{
		IK.packet_char_[12+i*9] = i+1;      						//MotorID
		IK.packet_char_[13+i*9] = totalspeed_[i] & 0xFF;    		//Profile Velocity      //initial value = 0
		IK.packet_char_[14+i*9] = (totalspeed_[i] >> 8) & 0xFF;
		IK.packet_char_[15+i*9] = (totalspeed_[i] >> 16) & 0xFF;
		IK.packet_char_[16+i*9] = (totalspeed_[i] >> 24) & 0xFF;
		IK.packet_char_[17+i*9] = totalangle_[i] & 0xFF;    		// positionL
		IK.packet_char_[18+i*9] = (totalangle_[i] >> 8) & 0xFF;
		IK.packet_char_[19+i*9] = (totalangle_[i] >> 16) & 0xFF;
		IK.packet_char_[20+i*9] = (totalangle_[i] >> 24) & 0xFF;
	}

	blk_size = 5 + IK.packet_char_[5];
	unsigned short lh_crc_value = IK.update_crc(0, IK.packet_char_, blk_size);

	//right hand
	for(i=0; i<4; i++)
	{
		IK.packet_char_[12+i*9] = i+5;      						//MotorID
		IK.packet_char_[13+i*9] = totalspeed_[i+4] & 0xFF;    		//Profile Velocity      //initial value = 0
		IK.packet_char_[14+i*9] = (totalspeed_[i+4] >> 8) & 0xFF;
		IK.packet_char_[15+i*9] = (totalspeed_[i+4] >> 16) & 0xFF;
		IK.packet_char_[16+i*9] = (totalspeed_[i+4] >> 24) & 0xFF;
		IK.packet_char_[17+i*9] = totalangle_[i+4] & 0xFF;    		// positionL
		IK.packet_char_[18+i*9] = (totalangle_[i+4] >> 8) & 0xFF;
		IK.packet_char_[19+i*9] = (totalangle_[i+4] >> 16) & 0xFF;
		IK.packet_char_[20+i*9] = (totalangle_[i+4] >> 24) & 0xFF;
	}

	blk_size = 5 + IK.packet_char_[5];
	unsigned short rh_crc_value = IK.update_crc(0, IK.packet_char_, blk_size);

	//foot
	IK.packet_char_[5] = 0x46; // 0x46 = 70  77-7

	IK.packet_char_[12] = 0x09;
	IK.packet_char_[13] = totalspeed_[8] & 0xFF;
	IK.packet_char_[14] = (totalspeed_[8] >> 8) & 0xFF;
	IK.packet_char_[15] = (totalspeed_[8] >> 16) & 0xFF;
	IK.packet_char_[16] = (totalspeed_[8] >> 24) & 0xFF;
	IK.packet_char_[17] = totalangle_[8] & 0xFF;
	IK.packet_char_[18] = (totalangle_[8] >> 8) & 0xFF;
	IK.packet_char_[19] = (totalangle_[8] >> 16) & 0xFF;
	IK.packet_char_[20] = (totalangle_[8] >> 24) & 0xFF;

	//left foot
	for(i=0; i<6; i++)
	{
		IK.packet_char_[21+i*9] = i+10;      						//MotorID
		IK.packet_char_[22+i*9] = totalspeed_[i+9] & 0xFF;    		//Profile Velocity      //initial value = 0
		IK.packet_char_[23+i*9] = (totalspeed_[i+9] >> 8) & 0xFF;
		IK.packet_char_[24+i*9] = (totalspeed_[i+9] >> 16) & 0xFF;
		IK.packet_char_[25+i*9] = (totalspeed_[i+9] >> 24) & 0xFF;
		IK.packet_char_[26+i*9] = totalangle_[i+9] & 0xFF;    		// positionL
		IK.packet_char_[27+i*9] = (totalangle_[i+9] >> 8) & 0xFF;
		IK.packet_char_[28+i*9] = (totalangle_[i+9] >> 16) & 0xFF;
		IK.packet_char_[29+i*9] = (totalangle_[i+9] >> 24) & 0xFF;
	}

	blk_size = 5 + IK.packet_char_[5];
	unsigned short lf_crc_value = IK.update_crc(0, IK.packet_char_, blk_size);

	//right foot
	for(i=0; i<6; i++)
	{
		IK.packet_char_[21+i*9] = i+16;      						//MotorID
		IK.packet_char_[22+i*9] = totalspeed_[i+15] & 0xFF;			//Profile Velocity      //initial value = 0
		IK.packet_char_[23+i*9] = (totalspeed_[i+15] >> 8) & 0xFF;
		IK.packet_char_[24+i*9] = (totalspeed_[i+15] >> 16) & 0xFF;
		IK.packet_char_[25+i*9] = (totalspeed_[i+15] >> 24) & 0xFF;
		IK.packet_char_[26+i*9] = totalangle_[i+15] & 0xFF;    		// positionL
		IK.packet_char_[27+i*9] = (totalangle_[i+15] >> 8) & 0xFF;
		IK.packet_char_[28+i*9] = (totalangle_[i+15] >> 16) & 0xFF;
		IK.packet_char_[29+i*9] = (totalangle_[i+15] >> 24) & 0xFF;
	}

	blk_size = 5 + IK.packet_char_[5];
	unsigned short rf_crc_value = IK.update_crc(0, IK.packet_char_, blk_size);

	*((uint32_t *)init.robot_motion_addr+(42)) = 30;
	*((uint32_t *)init.robot_motion_addr+(43)) = 0x00000070;
	*((uint32_t *)init.robot_motion_addr+(44)) = (lh_crc_value << 16) + rh_crc_value;
	*((uint32_t *)init.robot_motion_addr+(45)) = (lf_crc_value << 16) + rf_crc_value;

	locus.do_motion();
	IK.initial_inverse_kinematic();

	datamodule_cmd_ = 0;
	update_database_flag_ = false;
	motion_execute_flag_ = false;
}

void Datamodule::set_stand()
{
	int i=0;
	for(i=0; i<21; i++)
	{
		if(i==12||i==18)
			totalspeed_[i] = 80;
		else
			totalspeed_[i] = 40;
	}

	totalangle_[0] = 3072;
	totalangle_[1] = 1898;
	totalangle_[2] = 2048;
	totalangle_[3] = 2048;
	totalangle_[4] = 1024;
	totalangle_[5] = 2198;
	totalangle_[6] = 2048;
	totalangle_[7] = 2048;
	totalangle_[8] = 2048;
	totalangle_[9] = 2048;
	totalangle_[10] = 2028;
	totalangle_[11] = 1652;
	totalangle_[12] = 2737;
	totalangle_[13] = 2410;
	totalangle_[14] = 2047;
	totalangle_[15] = 2068;
	totalangle_[16] = 2058;
	totalangle_[17] = 2410;
	totalangle_[18] = 1337;
	totalangle_[19] = 1674;
	totalangle_[20] = 2047;

	motion_execute();
}
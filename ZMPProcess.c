#include "include/ZMPProcess.h"
ZMPProcess::ZMPProcess()
{
	ZMP_kg_offset = new int*[8];
	ZMP_kg_offset[0] = new int[5]{550,6650,14300,20100,26200};
	ZMP_kg_offset[1] = new int[5]{0  ,4300,10800,17500,25000};
	ZMP_kg_offset[2] = new int[5]{0  ,6730,13500,20100,27000};
	ZMP_kg_offset[3] = new int[5]{0  ,2000,6500 ,13500,21000};
	ZMP_kg_offset[4] = new int[5]{0,6700,13500,20150,27000};
	ZMP_kg_offset[5] = new int[5]{0,6700,13400,20400,26000};
	ZMP_kg_offset[6] = new int[5]{0,6550,13200,19700,26000};
	ZMP_kg_offset[7] = new int[5]{0,6750,13700,20100,26500};
	ZMP_kg_table = new int[5]{0,1,2,3,4};
	initialize();
}

ZMPProcess::~ZMPProcess()
{
	for(int i = 0; i < 8; i++)delete[] ZMP_kg_offset[i];
	delete[] ZMP_kg_offset;
	delete[] ZMP_kg_table;
}

void ZMPProcess::initialize()
{
	ZMP.initialize();
	for(int i = 0; i < 8; i++)sensor_offset_data_sum[i] = 0;
	for(int i = 0; i < 8; i++)origen_sensor_data[i] = 0;
	for(int i = 0; i < 8; i++)sensor_force[i] = 0;
	sensor_offset_data_count = 0;
	force_left = 0;
	force_right = 0;
	torque_left_x = 0;
	torque_left_y = 0;
}

void ZMPProcess::getSensorValue()
{

}

void ZMPProcess::digital2KGProcess()
{
	double 	ZMP_S[8];	
    for(int press_count = 0;press_count < 8;press_count++)
	{
		if(origen_sensor_data[press_count]<0)
		{
			origen_sensor_data[press_count] = 0;
		}
		ZMP_S[press_count] = 0;
		for(int kg_count = 0; kg_count < 4; kg_count++)
		{
			if ((origen_sensor_data[press_count] >= ZMP_kg_offset[press_count][kg_count]) && (origen_sensor_data[press_count] < ZMP_kg_offset[press_count][kg_count+1]))
			{
				if((ZMP_kg_offset[press_count][kg_count+1] - ZMP_kg_offset[press_count][kg_count]) >= 0)
				{
					ZMP_S[press_count] = (double)ZMP_kg_table[kg_count] + (double)((ZMP_kg_table[kg_count+1]-ZMP_kg_table[kg_count])*((double)(origen_sensor_data[press_count] - ZMP_kg_offset[press_count][kg_count])/(double)(ZMP_kg_offset[press_count][kg_count+1] - ZMP_kg_offset[press_count][kg_count])));
                    break;
                }
			}
            else if(kg_count == 3 && origen_sensor_data[press_count] >= ZMP_kg_offset[press_count][kg_count+1])
            {
                ZMP_S[press_count] = 5;
            }
		}
	}
    double sensor_digital_	        = ZMP_S[0]+ZMP_S[1]+ZMP_S[2]+ZMP_S[3] + ZMP_S[4]+ZMP_S[5]+ZMP_S[6]+ZMP_S[7];
	double sensor_digital_left_	    = ZMP_S[0]+ZMP_S[1]+ZMP_S[2]+ZMP_S[3];
	double sensor_digital_right_	= ZMP_S[4]+ZMP_S[5]+ZMP_S[6]+ZMP_S[7];

    force_left   = sensor_digital_left_;//*9.8;
    force_right  = sensor_digital_right_;

    for(int i = 0; i < 8; i++)
    {
        sensor_force[i] = ZMP_S[i];
    }
    torque_left_x = ((ZMP_S[0]+ZMP_S[1]) - (ZMP_S[2]+ZMP_S[3]))*SINGLE_FOOT_WEIGHT_X/100.0*9.8;
    torque_left_y = ((ZMP_S[0]+ZMP_S[3]) - (ZMP_S[1]+ZMP_S[2]))*SINGLE_FOOT_WEIGHT_EQUAL_Y/100.0*9.8;
    if(sensor_digital_ > 0.2)
	{
		ZMP.feet_pos.y = (double)((((ZMP_S[0]+ZMP_S[3]-ZMP_S[5]-ZMP_S[6])*DOUBLE_FEET_WEIGHT_FAR_Y) + ((ZMP_S[1]+ZMP_S[2]-ZMP_S[4]-ZMP_S[7]) * DOUBLE_FEET_WEIGHT_NEAR_Y) )/(double)sensor_digital_);
		ZMP.feet_pos.x = (double)((ZMP_S[0]+ZMP_S[1]+ZMP_S[4]+ZMP_S[5]-ZMP_S[2]-ZMP_S[3]-ZMP_S[6]-ZMP_S[7]) * DOUBLE_FEET_WEIGHT_X )/(double)sensor_digital_;
	}
	else
	{
		ZMP.feet_pos.y = 0;
		ZMP.feet_pos.x = 0;
	}
    if(sensor_digital_left_ > 0.2)
	{
		ZMP.left_pos.y = ((ZMP_S[0]+ZMP_S[3]-ZMP_S[1]-ZMP_S[2]) * SINGLE_FOOT_WEIGHT_EQUAL_Y)/sensor_digital_left_;
		ZMP.left_pos.x = ((ZMP_S[0]+ZMP_S[1]-ZMP_S[2]-ZMP_S[3]) * SINGLE_FOOT_WEIGHT_X)/sensor_digital_left_;
	}
	else
	{
		ZMP.left_pos.y = 0;
		ZMP.left_pos.x = 0;
	}
    if(sensor_digital_right_ > 0.2)
	{
		ZMP.right_pos.y = ((ZMP_S[4]+ZMP_S[7]-ZMP_S[5]-ZMP_S[6]) * SINGLE_FOOT_WEIGHT_EQUAL_Y)/sensor_digital_right_;
		ZMP.right_pos.x = ((ZMP_S[4]+ZMP_S[5]-ZMP_S[6]-ZMP_S[7]) * SINGLE_FOOT_WEIGHT_X)/sensor_digital_right_;			
	}
	else
	{ 
		ZMP.right_pos.y = 0;
		ZMP.right_pos.x = 0;		
	}
}

void ZMPProcess::setpSensorDataOffset(void *origen_sensor_data)
{
	for(int i = 0; i < 8; i++)
	{
		sensor_offset_data_sum[i] += ((int*)origen_sensor_data)[i];
	}
	sensor_offset_data_count++;
}

void ZMPProcess::setpOrigenSensorData(void *origen_sensor_data)
{
	if(sensor_offset_data_count == 0)
	{
		for(int i = 0; i < 8; i++)
		{
			this->origen_sensor_data[i] = ((int*)origen_sensor_data)[i];
		}
	}
	else
	{
		for(int i = 0; i < 8; i++)
		{
			this->origen_sensor_data[i] = ((int*)origen_sensor_data)[i] - sensor_offset_data_sum[i]/sensor_offset_data_count;
		}
	}
}

ZMPParam ZMPProcess::getZMPValue()
{
    digital2KGProcess();
    return this->ZMP;
}

float ZMPProcess::getForceLeft()
{
    return this->force_left;
}

float ZMPProcess::getForceRight()
{
    return this->force_right;
}

double *ZMPProcess::getpSensorForce()
{
    return this->sensor_force;
}

int *ZMPProcess::getpOrigenSensorData()
{
    return this->origen_sensor_data;
}

float ZMPProcess::getTorqueLeftX()
{
    return this->torque_left_x;
}

float ZMPProcess::getTorqueLeftY()
{
    return this->torque_left_y;
}
#include "include/WalkingCycle.h"

WalkingCycle::WalkingCycle()
{
    Sample_points_quater = 0;
    IsStop = false;
}

WalkingCycle::~WalkingCycle()
{

}

void WalkingCycle::walkingkindfunction(int walking_mode)
{
    switch(walking_mode)
    {
    case 6:
    case 5:
    case 4:
    case 2://LC
    case 3:
    case 0://Single Step
        singlestepfunction(walking_mode);
        break;
    case 8:
    case 7:
    case 1://Continuous
        continuoustepfunction();
        break;
    default:
        break;
    }
}

void WalkingCycle::singlestepfunction(int walking_mode)
{
    //    //printf("singlestepfunction");

    parameterinfo->complan.sample_point_++;
    parameterinfo->complan.time_point_ = parameterinfo->complan.sample_point_*(parameterinfo->parameters.Period_T/parameterinfo->parameters.Sample_Time);
    if (walking_mode == 0 || walking_mode == 4 || walking_mode == 5 || walking_mode == 6)
    {
        singlestepwalkingprocess();
    }else{
        LCwalkingprocess();
    }
    
}

void WalkingCycle::continuoustepfunction()
{
//    //printf("continuoustepfunction");

//parameterinfo->parameters.system_end = clock();
//printf("%f %d\n",((double)(parameterinfo->parameters.system_end-parameterinfo->parameters.system_start)/(CLOCKS_PER_SEC)),parameterinfo->complan.sample_point_);
    parameterinfo->complan.sample_point_++;
    parameterinfo->complan.time_point_ = parameterinfo->complan.sample_point_*(parameterinfo->parameters.Period_T/parameterinfo->parameters.Sample_Time);
//parameterinfo->parameters.system_start = clock();

    ////printf("%d  %d",parameterinfo->complan.time_point_,parameterinfo->complan.sample_point_);  
    //wosccontinuouswalkingprocess();
    continuouswalkingprocess();
}

void WalkingCycle::singlestepwalkingprocess()
{
    ////printf("singlestepwalkingprocess");

    //wchar_t* statetext;
    parameterinfo->counter++;
    switch(parameterinfo->complan.walking_state)
    {
        case StartStep:
            //printf("StartStep, %d", parameterinfo->counter);
            //statetext = L"StartStep";
            parameterinfo->complan.walking_stop = false;

            forwardValue_ = 0;
            slopeCounter_ = 0;
            forwardCounter_ = 0;

            if (!parameterinfo->IsParametersLoad)//!WalkingProcess->IsParametersLoad
            {
                //parameterinfo->NowStep.data = 0;
                parameterinfo->complan.isfirststep = true;
                parameterinfo->IsParametersLoad = true;
                parameterinfo->complan.islaststep = false;
                parameterinfo->complan.isLfootfirst = false;

                parameterinfo->XUpdate = parameterinfo->X;
                parameterinfo->YUpdate = parameterinfo->Y;
                //parameterinfo->ZUpdate = parameterinfo->Z;
                parameterinfo->THTAUpdate = parameterinfo->THTA;

                Lockrange_tmp = parameterinfo->parameters.OSC_LockRange;
                COM_Y_tmp = parameterinfo->parameters.Y_Swing_Range;
                parameterinfo->parameters.OSC_LockRange  = parameterinfo->parameters.OSC_LockRange;
                parameterinfo->parameters.Y_Swing_Range = parameterinfo->parameters.Y_Swing_Range + 0.5;

            }
            if (parameterinfo->complan.time_point_ == parameterinfo->parameters.Period_T2)
            {

                parameterinfo->parameters.Y_Swing_Range = COM_Y_tmp;
                parameterinfo->parameters.OSC_LockRange = Lockrange_tmp;
                //            WalkingProcess->Y = process_Value->Y;
                parameterinfo->complan.isfirststep = false;

            }
            if (parameterinfo->complan.time_point_ == parameterinfo->parameters.Period_T*3/4) // The first Step
            {
                parameterinfo->complan.walking_state = FirstStep;
                parameterinfo->IsParametersLoad = false;
                slopeCounter_++;
                forwardValue_ = slopeCounter_*INCREASE_SLOPE;
                forwardCounter_ += forwardValue_;
                //            WalkingProcess->Y = process_Value->Y;
                //            WalkingProcess->Thta = process_Value->Theta;
            }
            else
            {
                parameterinfo->XUpdate = forwardValue_;
            }

            break;
        case FirstStep:
            //printf("FirstStep, %d", parameterinfo->counter);
            //statetext = L"FirstStep";
            parameterinfo->complan.isfirststep = false;

            //		// Load Mark Time Parameters
            //		if (!WalkingProcess->IsParametersLoad) {
            //			WalkingProcess->IsParametersLoad = true;
            //			WalkingProcess->Parameters = WalkingProcess->WlakingStateParameters[etMarkTimeParameter];
            //		}

            if (parameterinfo->X > forwardCounter_ )
            {
                parameterinfo->XUpdate = forwardValue_;
                if (parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T / 4)
                {
                    parameterinfo->complan.walking_state = Repeat;
                    //				enable_repeat = false;
                    parameterinfo->IsParametersLoad = false;
                    slopeCounter_++;
                    forwardValue_ = slopeCounter_*INCREASE_SLOPE;
                    forwardCounter_ += forwardValue_;
                }
            }
            else
            {
                parameterinfo->XUpdate = parameterinfo->X;

                if (parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T / 4)
                {
                    parameterinfo->complan.walking_state = StopStep;
                    parameterinfo->IsParametersLoad = false;
                    parameterinfo->complan.islaststep = true;//rotate compensation
                }
            }
            break;
        case Repeat:
            //printf("Repeat, %d", parameterinfo->counter);
            //statetext = L"Repeat";
            if (parameterinfo->X > forwardCounter_ )
            {
                parameterinfo->XUpdate = forwardValue_;
                if(parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T/4 || parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T*3/4)
                {
                    parameterinfo->complan.walking_state = Repeat;
                    //				//enable_repeat = false;
                    parameterinfo->IsParametersLoad = false;
                    if (forwardValue_ >= WALK_MAX_DISTANCE)
                    {
                        forwardValue_ = WALK_MAX_DISTANCE;
                    }
                    else
                    {
                        slopeCounter_++;
                        forwardValue_ = slopeCounter_*INCREASE_SLOPE;
                    }
                    forwardCounter_ += forwardValue_;
                }
            }
            else
            {
                parameterinfo->THTAUpdate /= 1000;
                parameterinfo->YUpdate /= 1000;      //JJ
                if(parameterinfo->X == forwardCounter_)
                {
                    parameterinfo->XUpdate = forwardValue_;

                }
                else
                {
                    parameterinfo->XUpdate = parameterinfo->X - (forwardCounter_ - forwardValue_);
                }

                if (parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T/ 4 || parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T*3/4)
                {
                    parameterinfo->complan.walking_state = StopStep;
                    parameterinfo->IsParametersLoad = false;
                    parameterinfo->complan.islaststep = true;//rotate compensation
                }
            }
            break;
        case StopStep:
            //printf("StopStep, %d", parameterinfo->counter);
            //statetext = L"StopStep";
            parameterinfo->XUpdate = forwardCounter_ = 0;
            if(Sample_points_quater == parameterinfo->parameters.Sample_Time/4 - 1)
            {
                parameterinfo->complan.walking_state = StopStep;
                parameterinfo->complan.walking_stop = true;
                parameterinfo->IsParametersLoad = false;
                parameterinfo->WalkFlag = false;
                parameterinfo->FpgaFlag = false;
                //        			this->WalkEnable = 0;
                //                  WalkingProcess->Parameters->Initial();
                Sample_points_quater = 0;
                parameterinfo->complan.sample_point_ = 0;
            }
            else
            {
                Sample_points_quater++;
            }
            break;
        default:
            parameterinfo->complan.walking_state = StartStep;
            break;
    }
    parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;

    //if(parameterinfo->complan.time_point_ % (parameterinfo->parameters.Period_T/2) == 0 && !parameterinfo->complan.isfirststep)
        //parameterinfo->NowStep.data++;
}

void WalkingCycle::continuouswalkingprocess()
{
    // printf("continuouswalkingprocess\n");
    parameterinfo->counter++;
    switch (parameterinfo->complan.walking_state)
    {
        
        case StartStep:
            // printf("StartStep, %d", parameterinfo->counter);
            parameterinfo->complan.walking_stop = false;
            parameterinfo->complan.isfirststep = true;
            parameterinfo->Isfrist_right_shift = true;
            if(!parameterinfo->IsParametersLoad) 
            {
                //parameterinfo->NowStep.data = 0;
                parameterinfo->IsParametersLoad = true;
                //parameterinfo->XUpdate = parameterinfo->X;
                parameterinfo->YUpdate = parameterinfo->Y;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;
                //parameterinfo->THTAUpdate = parameterinfo->THTA;

            }
            if (parameterinfo->complan.time_point_ == parameterinfo->parameters.Period_T / 4)
            {
                parameterinfo->complan.walking_state = MarkTimeStep;
                parameterinfo->IsParametersLoad = false;
            }
            else
            {
                parameterinfo->XUpdate = 0;
            }
            
            break;

        case MarkTimeStep:
            // printf("MarkTimeStep, %d", parameterinfo->counter);
            //        // Load Mark Time Parameters
            if(!parameterinfo->IsParametersLoad)
            {
                parameterinfo->IsParametersLoad = true;
                //parameterinfo->XUpdate = parameterinfo->X;
                parameterinfo->YUpdate = parameterinfo->Y;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;
                //parameterinfo->THTAUpdate = parameterinfo->THTA;
            }
            if(parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T / 2)
            {
                //parameterinfo->XUpdate = parameterinfo->X;
                parameterinfo->YUpdate = parameterinfo->Y;
                //parameterinfo->ZUpdate = parameterinfo->Z;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;//20160214
                parameterinfo->THTAUpdate = parameterinfo->THTA;

                //parameterinfo->XUpdate = 0;
            }
            if(parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T * 3 / 4)
            {
                parameterinfo->complan.isfirststep = false;
                parameterinfo->XUpdate = parameterinfo->X;
                parameterinfo->YUpdate = parameterinfo->Y;
                //parameterinfo->ZUpdate = parameterinfo->Z;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;//20160214
                parameterinfo->THTAUpdate = parameterinfo->THTA;

                if(parameterinfo->XUpdate > 0)
                {
                    parameterinfo->complan.walking_state = ForwardStep;
                    parameterinfo->IsParametersLoad = false;
                } 
                else if(parameterinfo->XUpdate < 0)
                {
                    parameterinfo->complan.walking_state = BackwardStep;
                    parameterinfo->IsParametersLoad = false;
                } 
                else
                {
                    parameterinfo->complan.walking_state = parameterinfo->complan.walking_state;
                }
            }
            if(parameterinfo->complan.time_point_ == parameterinfo->parameters.Period_T)
                parameterinfo->Isfrist_right_shift = false;
            break;
        
        case ForwardStep:
            // printf("ForwardStep, %d", parameterinfo->counter);
            if(!parameterinfo->IsParametersLoad)
            {
                parameterinfo->IsParametersLoad = true;
            }

            if(parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T / 2)
            {
                //parameterinfo->parameters.OSC_LockRange = 0.25-(abs(parameterinfo->XUpdate)/10);//  JJ
                if(parameterinfo->parameters.OSC_LockRange < 0)
                {
                    parameterinfo->parameters.OSC_LockRange = 0;
                }

                //parameterinfo->XUpdate = parameterinfo->X;
                parameterinfo->YUpdate = parameterinfo->Y;
                //parameterinfo->ZUpdate = parameterinfo->Z;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;//20160214
                parameterinfo->THTAUpdate = parameterinfo->THTA;

            }
            if( parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T ==  parameterinfo->parameters.Period_T *3 / 4)
            {
                parameterinfo->XUpdate = parameterinfo->X;
                parameterinfo->YUpdate = parameterinfo->Y;
                //parameterinfo->ZUpdate = parameterinfo->Z;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;//20160214
                parameterinfo->THTAUpdate = parameterinfo->THTA;

                if(parameterinfo->XUpdate == 0) 
                {
                    parameterinfo->complan.walking_state = MarkTimeStep;
                    parameterinfo->IsParametersLoad = false;
                } 
                else 
                {
                    parameterinfo->complan.walking_state = parameterinfo->complan.walking_state;
                }
            }
            if(parameterinfo->complan.time_point_ == parameterinfo->parameters.Period_T)
                parameterinfo->Isfrist_right_shift = false;
            break;

        case BackwardStep:
            // printf("BackwardStep, %d", parameterinfo->counter);
            
            if(parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T / 2)
            {
                if(parameterinfo->parameters.OSC_LockRange < 0)
                {
                    parameterinfo->parameters.OSC_LockRange = 0;
                }

                //parameterinfo->XUpdate = parameterinfo->X;
                parameterinfo->YUpdate = parameterinfo->Y;
                //parameterinfo->ZUpdate = parameterinfo->Z;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;//20160214
                parameterinfo->THTAUpdate = parameterinfo->THTA;

            }

            if(parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T *3 / 4)
            {
                parameterinfo->XUpdate = parameterinfo->X;
                parameterinfo->YUpdate = parameterinfo->Y;
                //parameterinfo->ZUpdate = parameterinfo->Z;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;//20160214
                parameterinfo->THTAUpdate = parameterinfo->THTA;

                if(parameterinfo->XUpdate == 0) 
                {
                    parameterinfo->complan.walking_state = MarkTimeStep;
                    parameterinfo->IsParametersLoad = false;
                } 
                else 
                {
                    parameterinfo->complan.walking_state = parameterinfo->complan.walking_state;
                }
            }
            if(parameterinfo->complan.time_point_  == parameterinfo->parameters.Period_T)
                parameterinfo->Isfrist_right_shift = false;
            break;

        case StopStep:
            // printf("StopStep, %d", parameterinfo->counter);
            if(!parameterinfo->IsParametersLoad) 
            {
                parameterinfo->IsParametersLoad = true;
            }

            if(Sample_points_quater >= (parameterinfo->parameters.Sample_Time*3/4) && IsStop)
            {
                parameterinfo->XUpdate = 0;
            }
            else if(parameterinfo->counter <= parameterinfo->parameters.Sample_Time*3/4)
            {
                parameterinfo->XUpdate = 0;
            }
            else
            {
                parameterinfo->XUpdate = parameterinfo->X;
                parameterinfo->YUpdate = parameterinfo->Y;
                parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;
                parameterinfo->THTAUpdate = parameterinfo->THTA;
                if(IsStop && (parameterinfo->Y > 0.0) && (Sample_points_quater >= parameterinfo->parameters.Sample_Time * 1/2))
                {
                    parameterinfo->YUpdate = 0;
                }
                Sample_points_quater++;
            }

            if((parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T) == 0 && IsStop)
            {
                parameterinfo->complan.walking_state = StopStep;
                parameterinfo->complan.walking_stop = true;
                parameterinfo->IsParametersLoad = false;
                //parameterinfo->WalkFlag = false;
                parameterinfo->FpgaFlag = false;
                //printf("walking_stop = false");
                parameterinfo->complan.sample_point_ = 0;

                IsStop = false;
                Sample_points_quater = 0;
            }
            else if((parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T) == 0 && !IsStop)
            {
                IsStop = true;
                Sample_points_quater = 0;
            }
            break;
    }
    // printf("\n");
    // if(parameterinfo->complan.time_point_ % (parameterinfo->parameters.Period_T/2) == 0 && !parameterinfo->complan.isfirststep)
    // {
    //     //parameterinfo->NowStep.data++;
    //     // printf("parameterinfo->NowStep.data: %d", parameterinfo->NowStep.data);
    // }
}

void WalkingCycle::LCwalkingprocess()
{
    //    //printf("singlestepwalkingprocess");
    switch(parameterinfo->complan.walking_state)
    {
    case StartStep:
        //printf("StartStep");
        //statetext = L"StartStep";
        parameterinfo->complan.walking_stop = false;


        forwardValue_ = 0;
        slopeCounter_ = 0;
        forwardCounter_ = 0;

        if (!parameterinfo->IsParametersLoad)// !WalkingProcess->IsParametersLoad
        {
            parameterinfo->complan.isfirststep = true;
            parameterinfo->IsParametersLoad = true;
            parameterinfo->complan.islaststep = false;
            parameterinfo->complan.isLfootfirst = false;

            parameterinfo->XUpdate = parameterinfo->X;
            parameterinfo->YUpdate = parameterinfo->Y;
            parameterinfo->ZUpdate = parameterinfo->Z;
            parameterinfo->THTAUpdate = parameterinfo->THTA;

            Lockrange_tmp = parameterinfo->parameters.OSC_LockRange;
            COM_Y_tmp = parameterinfo->parameters.Y_Swing_Range;
            parameterinfo->parameters.OSC_LockRange  = parameterinfo->parameters.OSC_LockRange;
            parameterinfo->parameters.Y_Swing_Range = parameterinfo->parameters.Y_Swing_Range;

        }
        if (parameterinfo->complan.time_point_ == parameterinfo->parameters.Period_T/4) 
        {
            parameterinfo->complan.walking_state = FirstStep;
            parameterinfo->IsParametersLoad = false;
            parameterinfo->complan.isfirststep = false;
            parameterinfo->parameters.Y_Swing_Range = COM_Y_tmp;
            parameterinfo->complan.lift_lock_x = 0;
            //forwardValue_ = slopeCounter_*INCREASE_SLOPE;
            //forwardCounter_ += forwardValue_;
            //            WalkingProcess->Y = process_Value->Y;
            //            WalkingProcess->Thta = process_Value->Theta;
        }
        else
        {
            parameterinfo->XUpdate = 0;
        }

        break;
    case FirstStep:
        //printf("FirstStep");
        //statetext = L"FirstStep";
        parameterinfo->complan.isfirststep = false;
        // if (parameterinfo->X > forwardCounter_ )
        // {
        //     parameterinfo->XUpdate = forwardValue_;
        //     if (parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T / 4)
        //     {
        //         //parameterinfo->complan.walking_state = Repeat;
        //         //enable_repeat = false;
        //         //parameterinfo->IsParametersLoad = false;
        //         slopeCounter_++;
        //         forwardValue_ = slopeCounter_*INCREASE_SLOPE;
        //         forwardCounter_ += forwardValue_;
        //     }
        // }
        
            parameterinfo->XUpdate = parameterinfo->X;

            if (parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T *3 / 4)
            {
                parameterinfo->complan.walking_state = StopStep;
                parameterinfo->IsParametersLoad = false;
                parameterinfo->complan.islaststep = true;//rotate compensation
            }
        
        break;
    case StopStep:
        //printf("StopStep");
        //statetext = L"StopStep";
        parameterinfo->X = 0;//forwardCounter_ = 0;
        parameterinfo->XUpdate = 0;
        if(Sample_points_quater == parameterinfo->parameters.Sample_Time/4 - 1)
        {
            parameterinfo->complan.walking_state = StopStep;
            parameterinfo->complan.walking_stop = true;
            parameterinfo->IsParametersLoad = false;
//            parameterinfo->WalkFlag = false;
            parameterinfo->FpgaFlag = false;
            //                  this->WalkEnable = 0;
            //                  WalkingProcess->Parameters->Initial();
            Sample_points_quater = 0;
            parameterinfo->complan.sample_point_ = 0;
        }
        else
        {
            Sample_points_quater++;
        }
        
        break;
    default:
        parameterinfo->complan.walking_state = StartStep;
        break;
    }

    parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z ;
    parameterinfo->counter++;
}

void WalkingCycle::wosccontinuouswalkingprocess()
{
    //    //printf("continuouswalkingprocess");
    switch (parameterinfo->complan.walking_state){
    case StartStep:
        parameterinfo->complan.walking_stop = false;
        if (!parameterinfo->IsParametersLoad) {
            parameterinfo->IsParametersLoad = true;

            //            WalkingProcess->ZUpdate = WalkingProcess->Parameters->BASE_Default_Z ;
            //            WalkingProcess->WalkingUpdate();

            parameterinfo->XUpdate = parameterinfo->X;
            parameterinfo->YUpdate = parameterinfo->Y;
            parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;
            parameterinfo->THTAUpdate = parameterinfo->THTA;

        }
        if (parameterinfo->complan.time_point_ == parameterinfo->parameters.Period_T / 4)
        {
            parameterinfo->complan.walking_state = MarkTimeStep;
            parameterinfo->IsParametersLoad = false;
        }
        else
        {
            parameterinfo->XUpdate = 0;
        }

        break;
    case MarkTimeStep:
        //        // Load Mark Time Parameters
        if (!parameterinfo->IsParametersLoad)
        {
            parameterinfo->IsParametersLoad = true;
        }
        if (parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T / 2)
        {
            parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;//+ max(abs(parameterinfo->THTAUpdate * 6),max(abs(parameterinfo->XUpdate),abs(parameterinfo->YUpdate)));

            parameterinfo->XUpdate = parameterinfo->X;
            parameterinfo->YUpdate = parameterinfo->Y;
            parameterinfo->ZUpdate = parameterinfo->Z;
            parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;//20160214
            parameterinfo->THTAUpdate = parameterinfo->THTA;

            parameterinfo->XUpdate = 0;
            break;
        }
        if (parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T * 3 / 4)
        {
            parameterinfo->XUpdate = parameterinfo->X;
            parameterinfo->YUpdate = parameterinfo->Y;
            parameterinfo->ZUpdate = parameterinfo->Z;
            parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;//20160214
            parameterinfo->THTAUpdate = parameterinfo->THTA;

            if (parameterinfo->XUpdate != 0)
            {
                parameterinfo->complan.walking_state = ForwardStep;
                parameterinfo->IsParametersLoad = false;
            } else
            {
                parameterinfo->complan.walking_state = parameterinfo->complan.walking_state;
            }
        }
    case ForwardStep:
        if (!parameterinfo->IsParametersLoad)
        {
            parameterinfo->IsParametersLoad = true;
        }

        if (parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T / 2)
        {
            //WalkingProcess->ZUpdate = WalkingProcess->Parameters->BASE_Default_Z + max(abs(WalkingProcess->Thta*4),max(abs(WalkingProcess->X),abs(WalkingProcess->Y)));
            //parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;
            parameterinfo->parameters.OSC_LockRange = 0.25-(abs(parameterinfo->XUpdate)/10);//  JJ
            if (parameterinfo->parameters.OSC_LockRange < 0)
            {
                parameterinfo->parameters.OSC_LockRange = 0;
            }

            parameterinfo->XUpdate = parameterinfo->X;
            parameterinfo->YUpdate = parameterinfo->Y;
            parameterinfo->ZUpdate = parameterinfo->Z;
            parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;//20160214
            parameterinfo->THTAUpdate = parameterinfo->THTA;

        }
        if ( parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T ==  parameterinfo->parameters.Period_T *3 / 4)
        {
            parameterinfo->XUpdate = parameterinfo->X;
            parameterinfo->YUpdate = parameterinfo->Y;
            parameterinfo->ZUpdate = parameterinfo->Z;
            parameterinfo->ZUpdate = parameterinfo->parameters.BASE_Default_Z;//20160214
            parameterinfo->THTAUpdate = parameterinfo->THTA;

            if (parameterinfo->XUpdate == 0) {
                parameterinfo->complan.walking_state = MarkTimeStep;
                parameterinfo->IsParametersLoad = false;
            } else {
                parameterinfo->complan.walking_state = parameterinfo->complan.walking_state;

                ////printf("sample_point_ = %d",parameterinfo->complan.sample_point_);
            }
        }
        break;
    case BackwardStep:
        //        if (!WalkingProcess->IsParametersLoad) {
        //            WalkingProcess->IsParametersLoad = true;
        //            WalkingProcess->Parameters = WalkingProcess->WlakingStateParameters[etBackwardParameter];
        //            WalkingProcess->ZUpdate = WalkingProcess->Parameters->BASE_Default_Z;
        //        }

        if (parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T / 2)
        {
            //WalkingProcess->WalkingUpdate();
        }
        if (parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T == parameterinfo->parameters.Period_T *3 / 4)
        {
            //WalkingProcess->WalkingUpdate();
            if (parameterinfo->XUpdate > 0) {
                parameterinfo->complan.walking_state = ForwardStep;
                parameterinfo->IsParametersLoad = false;
            } else if (parameterinfo->XUpdate == 0) {
                parameterinfo->complan.walking_state = MarkTimeStep;
                parameterinfo->IsParametersLoad = false;
            } else {
                parameterinfo->complan.walking_state = parameterinfo->complan.walking_state;
            }
        }
        break;
    case StopStep:
        if (!parameterinfo->IsParametersLoad) {
            parameterinfo->IsParametersLoad = true;
        }
        parameterinfo->XUpdate = 0;
        parameterinfo->YUpdate = 0;
        parameterinfo->THTAUpdate = 0;

        if((parameterinfo->points.Z_Right_foot == 0)&&(parameterinfo->points.Z_Left_foot == 0)) //if((parameterinfo->complan.time_point_ % parameterinfo->parameters.Period_T*4) == 0)// if(Times == 1)  JJ
        {
            parameterinfo->complan.walking_state = StopStep;
            parameterinfo->complan.walking_stop = true;
            parameterinfo->IsParametersLoad = false;
            //parameterinfo->WalkFlag = false;
            parameterinfo->FpgaFlag = false;
            //printf("walking_stop = false");
            parameterinfo->complan.sample_point_ = 0;

        }
        break;
    }
    parameterinfo->counter++;
    ////printf("value !!!!  [%f %f %f %f    %d]", parameterinfo->XUpdate,parameterinfo->YUpdate,parameterinfo->ZUpdate,parameterinfo->THTAUpdate,parameterinfo->counter);
    ////printf("%d\n",parameterinfo->counter);

}
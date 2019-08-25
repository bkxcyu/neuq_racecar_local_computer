#include"rsband_local_planner/auto_correct.h"


namespace rsband_local_planner{

point_list::point_list()
{
	pi=3.1415926;
	gain_angle = 1;
	unit_distance = 100;
	warning_distance = 1.0; 
	limit_distance = 0.4;
	angle_max = 3.14;
	angle_min = -3.14;

	signal_front=0;
	signal_left=0;
	signal_right=0;
	signal_back=0; 	
}



void point_list::output()
{
	ROS_INFO("The warning point list:");
	if(warning_point.empty())
		printf("null\n");
	else
	{
		int count = warning_point.size();
    	for (int i = 0;i<count;i++)
    	{
			ROS_INFO("in dis=%.2f,ang=%.2f",warning_point[i].distance,warning_point[i].angle);
    		//printf("dis:%f\t | ang:%f\n",warning_point[i].distance,warning_point[i].angle);
    	}	
	} 
}


void point_list::append(float dis,float ang)
{
	struct Obs_point in;
	in.distance=dis;
	in.angle=ang;
	warning_point.push_back(in);	
}


void point_list::clearlist()
{
	warning_point.clear();
	//clear data
	signal_front=0;
	signal_left=0;
	signal_right=0;
	signal_back=0;

	left_count =0;
    right_count=0;
	front_count=0;
	back_count=0;
}


    
void point_list::sortlist()
{
	if(!warning_point.empty())
	{
		int count = warning_point.size();
   		for(int i=0;i<count-1;i++)
    	{
        	for(int j=0;j<count-i-1;j++)
        	{
        		if(warning_point[j].distance > warning_point[j+1].distance)
				{
					float temp_d = warning_point[j].distance;
					warning_point[j].distance = warning_point[j+1].distance;
					warning_point[j+1].distance = temp_d;
				
					float temp_a = warning_point[j].angle;
					warning_point[j].angle = warning_point[j+1].angle;
					warning_point[j+1].angle = temp_a;
				}
        	}
    	}


		

		if(count>0)
        {
          for(int m=0;m<count;m++)
          {
            if(warning_point[m].angle>=-1.57 && warning_point[m].angle<-0.52)
              right_count++;
            else if(warning_point[m].angle>=-0.52 && warning_point[m].angle<0.52)
              front_count++;
			else if(warning_point[m].angle>=0.52 && warning_point[m].angle<1.57)
              left_count++;
			else if((warning_point[m].angle>=1.57&&warning_point[m].angle<3.14)||
					(warning_point[m].angle>=-3.14&&warning_point[m].angle<-1.57))
              back_count++;
          }

		  if(front_count>=left_count && front_count>=right_count && front_count>=back_count)
		  	signal_front=1;
		  else if(left_count>=right_count && left_count>=front_count && left_count>=back_count)
		  	signal_left=1;
		  else if(right_count>=front_count && right_count>=left_count && right_count>=back_count)
		  	signal_right=1;
		  else if(back_count>=front_count && back_count>=left_count && back_count>=right_count)
		  	signal_back=1;
		  

		  ROS_INFO("front:%d | left:%d | right:%d | back:%d",front_count,left_count,right_count,back_count);
        }	
	} 
}




void point_list::v_vector()
{
	float sum_dis=0;
	float sum_ang=0;
	out_point.distance =  warning_point[0].distance;
	int point_count = warning_point.size();
	if(warning_point.empty())
	{
		out_point.angle=0;
		out_point.distance=100;
	}
	else
	{
		if(signal_front=1)
		{
			for(int i=0;i<front_count;i++)
			{
				if(warning_point[i].angle>=-0.52 && warning_point[i].angle<0.52)
				{
					sum_dis += warning_point[i].distance;
					sum_ang += warning_point[i].angle;
				}
			}
			out_point.angle=(float)sum_ang/(float)front_count;
			out_point.distance=(float)sum_dis/(float)front_count;
		}

		else if(signal_left=1)
		{
			for(int i=0;i<left_count;i++)
			{
				if(warning_point[i].angle>=0.52 && warning_point[i].angle<1.57)
				{
					sum_dis += warning_point[i].distance;
					sum_ang += warning_point[i].angle;
				}
			}
			out_point.angle=(float)sum_ang/(float)left_count;
			out_point.distance=(float)sum_dis/(float)left_count;
		}

		else if(signal_right=1)
		{
			for(int i=0;i<right_count;i++)
			{
				if(warning_point[i].angle>=-1.57 && warning_point[i].angle<-0.52)
				{
					sum_dis += warning_point[i].distance;
					sum_ang += warning_point[i].angle;
				}
			}
			out_point.angle=(float)sum_ang/(float)right_count;
			out_point.distance=(float)sum_dis/(float)right_count;
		}

		
		else if(signal_back=1)
		{
			for(int i=0;i<back_count;i++)
			{
				if((warning_point[i].angle>=1.57&&warning_point[i].angle<3.14)||
					(warning_point[i].angle>=-3.14&&warning_point[i].angle<-1.57))
				{
					sum_dis += warning_point[i].distance;
					sum_ang += warning_point[i].angle;
				}
			}
			out_point.angle=(float)sum_ang/(float)back_count;
			out_point.distance=(float)sum_dis/(float)back_count;
		}
	}

}


}//end namespace
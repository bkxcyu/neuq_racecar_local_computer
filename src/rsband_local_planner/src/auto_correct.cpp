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
	signal_intensity_left=0;
	signal_intensity_right=0; 	
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
	signal_intensity_left=0;
	signal_intensity_right=0;
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


		int left_count =0;
		int right_count=0;
		if(count>0)
        {
          for(int m=0;m<count;m++)
          {
            if(warning_point[m].angle>0)
              left_count++;
            else if(warning_point[m].angle<0)
              right_count++;
          }

          signal_intensity_left = (float)left_count/(float)count;
          signal_intensity_right= (float)right_count/(float)count;

		  //ROS_INFO("left_intensity:%.4f | right_intensity:%.4f",signal_intensity_left,signal_intensity_right);
        }	
	} 
}




void point_list::v_vector(float add)
{
	ROS_INFO("add=%.2f",add);
	out_point.distance =  unit_distance;
	if(warning_point.empty())
	{
		out_point.angle=0;
	}
	else
	{
		if(warning_point[0].distance < warning_distance && warning_point[0].distance > limit_distance)
		{
			if(warning_point[0].angle > 0)
				out_point.angle = warning_point[0].angle - add*signal_intensity_left;
			else if(warning_point[0].angle <= 0)
				out_point.angle = warning_point[0].angle + add*signal_intensity_right;
			
			if(out_point.angle > angle_max)
				out_point.angle = angle_max;
			else if(out_point.angle <= angle_min)
				out_point.angle = angle_min;
		} 
		

		else if(warning_point[0].distance <= limit_distance && warning_point[0].distance >= 0)
		{
			if(warning_point[0].angle > 0)
				out_point.angle = angle_min;
			else if(warning_point[0].angle <= 0)
				out_point.angle = angle_max;
		}
	}
}


}//end namespace
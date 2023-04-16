#include "wall_follower_new.h"
  
 void WallFollowerNew::setLaserData(const std::vector<float>& data)
{
	int min_range_index = 0;
    for (size_t i = 1; i<3*data.size()/8; i++)
    {
        if ( data[i] < data[min_range_index])
        	min_range_index = i;
        
    }
  range_eps = data[0] - wall_range;
  angle_eps = cos(3.14/data.size())*data[min_range_index] - data[0];
  ROS_INFO_STREAM("RangeEps: "<<range_eps<<"; AngleEps: "<<angle_eps);
}

//получение управления
void WallFollowerNew::getControl(double& v, double& w)
{
	//w = angle_eps;
	w = K_A * angle_eps + K_R*range_eps;
	v = 1 - 10*abs(angle_eps);
	if(v<0)
		v = 0;
}
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <vector>
#include <cstring>

#include "struct_type.hpp"

class GradientDescent{
    private:
        ros::NodeHandle nh;
        ros::NodeHandle h;   
        ros::Subscriber randomPoint_sub;
        ros::Publisher marker_pub;

    public:
        GradientDescent(float fGain, float sGain){
            randomPoint_sub = h.subscribe ("input", 1, &GradientDescent::RandomPointCallBack, this);
            marker_pub = nh.advertise<visualization_msgs::Marker>("lane", 1);
        }

        ~GradientDescent(){
            std::cout << "Bye" << std::endl;
        }

        void RandomPointCallBack (const sensor_msgs::PointCloud2ConstPtr& msg){
            ST_RAMDOMPOINT st_randompoint;

            sensor_msgs::PointCloud2ConstIterator<float> ptr_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> ptr_y(*msg,  "y");
            sensor_msgs::PointCloud2ConstIterator<float> ptr_z(*msg,  "z");

            for(; (ptr_x != ptr_x.end()) && (ptr_y != ptr_y.end()) && ptr_z != ptr_z.end(); ++ptr_x, ++ptr_y,++ptr_z){
                _POINT* _point = new _POINT; 

                _point->x = *ptr_x;
                _point->y = *ptr_y;
                _point->z = *ptr_z;

                st_randompoint.st_point.push_back(*_point);
                st_randompoint.nPoint++;

                //printf("%d %f %f %f\n",st_randompoint.nPoint-1, st_randompoint.st_point[st_randompoint.nPoint-1].x, st_randompoint.st_point[st_randompoint.nPoint-1].y, st_randompoint.st_point[st_randompoint.nPoint-1].z);
                
                delete _point;
            }

        }

        float OptimizationRun(float& linearCoeff, float time){
            float endTime;

            do{
                

            }while((endTime - time) < 0.96);


            return endTime;
        }

        float GetOptimizationMean(ST_RAMDOMPOINT& _st_randomPoint, float linearCoeff){
            float mean = 0;

            for(int i=0; i < _st_randomPoint.nPoint; i++){
                        

            }

        }


};


int main (int argc, char** argv){

    ros::init(argc, argv, "GradientDescent");    

    GradientDescent gradientDescent(0.0001, 0.0001);

    ros::spin();

    return 0;
}


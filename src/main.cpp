#include    <ros/ros.h>
#include    <visualization_msgs/Marker.h>
#include    <sensor_msgs/PointCloud2.h>
#include    <sensor_msgs/point_cloud2_iterator.h>
#include    <iostream>
#include    <vector>
#include    <cstring>
#include    <cstdlib>
#include    <ctime>
#include    <cmath>
#include    <signal.h>

typedef float float32_t;
typedef double float64_t;

void  INThandler(int sig)
{
     char  c;
 
     signal(sig, SIG_IGN);

     exit(1);
}

typedef struct _POINT{
    float x;
    float y;
    float z;
}_POINT;

typedef struct ST_RAMDOMPOINT{
    std::vector<_POINT> st_point;
    unsigned int nPoint;
}ST_RAMDOMPOINT;

class GradientDescent{
    private:
        ros::NodeHandle nh;
        ros::NodeHandle h;   
        ros::Subscriber randomPoint_sub;
        ros::Publisher marker_pub;


    public:
        float32_t linearCoeff[2] = {0, };
        int32_t nCnt = 0;
        float32_t stepSize = 0;
        float32_t aStep = 0;
        float32_t bStep = 0;
        
        GradientDescent(void){
            srand((unsigned) time(NULL));

            randomPoint_sub = h.subscribe ("input", 1, &GradientDescent::RandomPointCallBack, this);
            marker_pub = nh.advertise<visualization_msgs::Marker>("lane", 1);

            for(int32_t i=0; i<2; i++){
                linearCoeff[i] = (float)(rand() % 10);
            }
        }

        ~GradientDescent(){
            std::cout << "Bye" << std::endl;
        }

        void RandomPointCallBack (const sensor_msgs::PointCloud2ConstPtr& msg){
            //printf("nCnt : %d\n",nCnt);
            nCnt++;
            ST_RAMDOMPOINT st_randomPoint;

            sensor_msgs::PointCloud2ConstIterator<float> ptr_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> ptr_y(*msg,  "y");
            sensor_msgs::PointCloud2ConstIterator<float> ptr_z(*msg,  "z");

            for(; (ptr_x != ptr_x.end()) && (ptr_y != ptr_y.end()) && ptr_z != ptr_z.end(); ++ptr_x, ++ptr_y,++ptr_z){
                _POINT* _point = new _POINT; 

                _point->x = *ptr_x;
                _point->y = *ptr_y;
                _point->z = *ptr_z;

                st_randomPoint.st_point.push_back(*_point);
                st_randomPoint.nPoint++;
            }
            float32_t stTime = clock();  //clock to ms

            OptimizationRun(st_randomPoint, linearCoeff, stTime);
            publishRviz(linearCoeff);
        }

        float32_t GetLossBeta(const ST_RAMDOMPOINT& _st_randomPoint, float32_t* linearCoeff){
            float32_t sum = 0;
            int32_t nCnt = 0;

            for(int32_t i=0; i < _st_randomPoint.nPoint; i++){
                sum = sum + ((linearCoeff[1] + linearCoeff[0]* _st_randomPoint.st_point[i].x) - _st_randomPoint.st_point[i].y);
                nCnt++;
            }
            //std::cout << sum << std::endl;
            return sum/(nCnt);
        }

        float32_t GetLossAlpha(const ST_RAMDOMPOINT& _st_randomPoint, float32_t* linearCoeff){
            float32_t sum = 0;
            int32_t nCnt = 0;

            for(int32_t i=0; i < _st_randomPoint.nPoint; i++){
                sum = sum + ((linearCoeff[1] + linearCoeff[0]* _st_randomPoint.st_point[i].x) - _st_randomPoint.st_point[i].y)*_st_randomPoint.st_point[i].x; 
                nCnt++;
            }
            //std::cout << sum << std::endl;

            return sum/(nCnt);
        }

        float32_t OptimizationRun(ST_RAMDOMPOINT& _st_randomPoint, float32_t* linearCoeff, float32_t time){
            float32_t endTime;
            float32_t deadLine = 0;
            int32_t nOptimization = 0;
            float32_t step = 0.01;
            float32_t alphaLoss = 0;
            float32_t betaLoss = 0;      

            while(1){
                alphaLoss = GetLossAlpha(_st_randomPoint, linearCoeff);
                betaLoss = GetLossBeta(_st_randomPoint, linearCoeff);

                linearCoeff[0] = linearCoeff[0] - step * alphaLoss;
                linearCoeff[1] = linearCoeff[1] - step * betaLoss;

                endTime = clock();
                deadLine = (endTime - time) / (CLOCKS_PER_SEC);
                //printf("in time : %f loss : %f\n", deadLine, loss);
                signal(SIGINT, INThandler);
                nOptimization++;
                if(deadLine > 0.0086) {
                    printf("timeout! %d  ", nOptimization);
                    break;
                }
                else if(fabs(alphaLoss) < 0.0001 && fabs(betaLoss) < 0.0001){
                    printf("Optimization! %d  ", nOptimization);
                    break;
                }
                //printf("alpha : %f beta : %f\n", alphaLoss, betaLoss);
            };

            printf("%fx + %f time : %f %f %f\n",linearCoeff[0], linearCoeff[1], deadLine*10, alphaLoss, betaLoss);

            return endTime;
        }

        void publishRviz(float32_t* linearCoeff){
            visualization_msgs::Marker line_sqtsq; 

            line_sqtsq.header.frame_id = "map";
            line_sqtsq.ns =  "marker";
            line_sqtsq.action = visualization_msgs::Marker::ADD;
            line_sqtsq.pose.orientation.w = 1.0;

            line_sqtsq.id = 0;
            line_sqtsq.type = visualization_msgs::Marker::LINE_STRIP;

            line_sqtsq.scale.x = 0.1;
            line_sqtsq.color.b = 1.0;
            line_sqtsq.color.a = 1.0;

            geometry_msgs::Point p;

            float32_t drawPosx[2] = {-5, 5};

            for(int32_t i =0; i<2; i++){
                p.x = drawPosx[i];
                p.y = linearCoeff[0] * p.x + linearCoeff[1];
                p.z = 0;

                line_sqtsq.points.push_back(p);
            }

            marker_pub.publish(line_sqtsq);

        }

};


int32_t main (int32_t argc, char** argv){

    ros::init(argc, argv, "GradientDescent");    

    GradientDescent gradientDescent();
    
    ros::spin();

    return 0;
}


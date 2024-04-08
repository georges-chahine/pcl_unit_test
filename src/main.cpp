#define BOOST_BIND_NO_PLACEHOLDERS
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class LidarMapper : public rclcpp::Node
{
private:

    /* ------------------------------ variables------------------------------------------------------------------------- */
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_, publisher2_;
    size_t count_;
    //float d, w, s;
    float step;
    unsigned int counter;
    /* ------------------------------functions -------------------------------------------------------------------------*/


    void spawn_close_object(pcl::PointCloud<pcl::PointXYZ>::Ptr& input){
        for(float i=1; i<3; i=i+step){
            for(float j=1; j<3; j=j+step){
                for(float k=0.5; k<2.5; k=k+step){
                    pcl::PointXYZ p;
                    p.x=i;
                    p.y=j;
                    p.z=k;
                    input->points.push_back(p);
                }
            }
        }
    }

    void spawn_far_object(pcl::PointCloud<pcl::PointXYZ>::Ptr& input){
        for(float i=13; i<15; i=i+step){
            for(float j=13; j<15; j=j+step){
                for(float k=0.5; k<2.5; k=k+step){
                    pcl::PointXYZ p;
                    p.x=i;
                    p.y=j;
                    p.z=k;
                    input->points.push_back(p);
                }
            }
        }
    }

    void spawn_warn_object(pcl::PointCloud<pcl::PointXYZ>::Ptr& input){
        for(float i=9; i<11; i=i+step){
            for(float j=5; j<7; j=j+step){
                for(float k=0.5; k<2.5; k=k+step){
                    pcl::PointXYZ p;
                    p.x=i;
                    p.y=j;
                    p.z=k;
                    input->points.push_back(p);
                }
            }
        }
    }

    void spawn_warn_submerged_object(pcl::PointCloud<pcl::PointXYZ>::Ptr& input){
        for(float i=5; i<7; i=i+step){
            for(float j=5; j<7; j=j+step){
                for(float k=-1.5; k<0.5; k=k+step){
                    pcl::PointXYZ p;
                    p.x=i;
                    p.y=j;
                    p.z=k;
                    input->points.push_back(p);
                }
            }
        }
    }

    void spawn_warn_georef_object(pcl::PointCloud<pcl::PointXYZ>::Ptr& input){
        for(float i=2; i<4; i=i+step){
            for(float j=2; j<4; j=j+step){

                pcl::PointXYZ p;
                p.x=i;
                p.y=j;
                p.z=0;
                input->points.push_back(p);

            }
        }
    }


    void lidar_callback()
    {
        //std::cout<<"Entered callback..."<<std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr obst_danger (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obst_warning (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obst_warning_submerged (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obst_far (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr geo_ref (new pcl::PointCloud<pcl::PointXYZ>);


        if (counter==0){spawn_close_object(obst_danger);}
        if (counter==2){spawn_warn_object(obst_warning);}
        if (counter==3){spawn_far_object(obst_far);}
        if (counter==1){spawn_warn_submerged_object(obst_warning_submerged);}
        if (counter==4){
            spawn_close_object(obst_danger);
            spawn_warn_object(obst_warning);
            spawn_far_object(obst_far);
            spawn_warn_submerged_object(obst_warning_submerged);
        }
        spawn_warn_georef_object(geo_ref);
        counter++;
        if (counter==5){
            counter=0;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);

        *cloud_in= *obst_danger + *obst_warning + *obst_far + *obst_warning_submerged;

        sensor_msgs::msg::PointCloud2 pc2_msg_, pc2_msg2_;
        pcl::toROSMsg(*cloud_in, pc2_msg_);
        pcl::toROSMsg(*geo_ref, pc2_msg2_);

        pc2_msg_.header.frame_id = "base_link";
        pc2_msg2_.header.frame_id = "base_link";
        rclcpp::Time now = this->get_clock()->now();
        pc2_msg_.header.stamp = now;
        pc2_msg2_.header.stamp = now;

        publisher_->publish(pc2_msg_);

        publisher2_->publish(pc2_msg2_);



    }
public:
    LidarMapper()
        : Node("pcl_unit_test"), count_(0)
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("base_projected_cloud", 1); //to visualize obstacle points
        publisher2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("geo_ref_cloud", 1); //to visualize obstacle points
        rclcpp::TimerBase::SharedPtr timer_{nullptr};
        step=0.1;
        counter=0;
        while (true){
            lidar_callback();
            rclcpp::sleep_for(std::chrono::nanoseconds(1000ms));
        }
        /* hard coded parameters */
        /* hard coded parameters */
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarMapper>());
    rclcpp::shutdown();
    return 0;
}

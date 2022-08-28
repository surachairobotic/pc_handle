#include <memory>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/duration.hpp>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

//#include <sensor_msgs/point_cloud2_iterator.h>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <tf2_ros/transform_listener.h>

#include "geometry_msgs/msg/point32.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
    public:
        MinimalSubscriber()
        : Node("minimal_subscriber"), bFirst(false), count(0) {
            subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/camera/depth/color/points", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
            sub_imu = this->create_subscription<sensor_msgs::msg::Imu>("/camera/imu", 10, std::bind(&MinimalSubscriber::cb_imu, this, _1));

            publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pc", 10);
        }

    private:
        void cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
            double accelX = msg->linear_acceleration.x;
            double accelY = msg->linear_acceleration.y;
            double accelZ = msg->linear_acceleration.z;
            pitch = 180 * std::atan2(accelX, std::sqrt(accelY*accelY + accelZ*accelZ))/3.14159;
            roll = 180 * std::atan2(accelY, std::sqrt(accelX*accelX + accelZ*accelZ))/3.14159;
            lRoll[indx] = roll;
            lPitch[indx] = pitch;
            indx = (indx+1) % 100;
            double sR=0.0, sP=0.0;
            for(int i=0; i<100; i++) {
                sR += lRoll[i];
                sP += lPitch[i];
            }
            roll = sR / 100.0;
            pitch = sP / 100.0;
            //RCLCPP_INFO(this->get_logger(), "acc : %lf, %lf, %lf : %lf, %lf", msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z, roll, pitch);
        }
        void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            if(!bFirst) {
                this->t1 = rclcpp::Clock().now();
                this->bFirst = true;
            }
            
            //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->header.frame_id.c_str());
            int num = msg->fields.size();
            //for(int i=0; i<num; i++)
                //RCLCPP_INFO(this->get_logger(), "fields: '%s'", msg->fields[i].name.c_str());
            
            sensor_msgs::msg::PointCloud2 pc_out = *msg;
            sensor_msgs::PointCloud2Iterator<float> iter_x(pc_out, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(pc_out, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(pc_out, "z");
            //sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*msg, "r");
            //sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*msg, "g");
            //sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*msg, "b");

            geometry_msgs::msg::Point32 pnt;
            double _roll = (roll+90)/180.0*3.1459, _pitch = pitch/90.0*3.1459;
            RCLCPP_INFO(this->get_logger(), "roll,pitch : %lf, %lf", roll, pitch);

            //int ix = 0;
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z/*, ++iter_r, ++iter_g, ++iter_b*/) {
                //*iter_x = (ix-5000)*0.001;
                //*iter_y = 0;
                //*iter_z = 0;
                //ix = (ix+1)%10001;
                pnt.x = *iter_x;
                pnt.y = *iter_y;
                pnt.z = *iter_z;
                this->rotateP('z', _roll, &pnt);
                this->rotateP('x', _pitch, &pnt);
                *iter_x = pnt.x;
                *iter_y = pnt.y;
                *iter_z = pnt.z;
                //*iter_r = 255;
                //*iter_g = 0;
                //*iter_b = 0;
            }

            //transform.translation.x = count/10.0;
            //tf2::doTransform(*msg, pc_out, transform);
            publisher_->publish(pc_out);
            count = (count + 1) % 40;
            
            rclcpp::Duration dt = rclcpp::Clock().now() - t1;
            this->t1 = rclcpp::Clock().now();
            double fps = 1.0 / (dt.nanoseconds()*0.000000001);
            
            //RCLCPP_INFO(this->get_logger(), "FPS : %lf", fps);
        }

        void rotateP(char axes, double radian, geometry_msgs::msg::Point32 *_pnt) {
            double c=cos(radian), s=sin(radian);
            if(axes == 'z') {
                double x=_pnt->x;
                double y=_pnt->y;
                _pnt->x = (x*c)-(y*s);
                _pnt->y = (x*s)+(y*c);
            }
            else if(axes == 'y') {
                double x=_pnt->x;
                double z=_pnt->z;
                _pnt->x =  (x*c)+(z*s);
                _pnt->z = -(x*s)+(z*c);
            }
            else if(axes == 'x') {
                double z=_pnt->z;
                double y=_pnt->y;
                _pnt->y = (y*c)-(z*s);
                _pnt->z = (y*s)+(z*c);
            }
        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        
        geometry_msgs::msg::TransformStamped transform;
        int count;
        
        double timeloop_sc, roll, pitch, lRoll[100], lPitch[100];
        int indx=0;
        bool bFirst;
        rclcpp::Time t1;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}

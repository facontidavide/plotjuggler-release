#ifndef SENSOR_IMU_MSG_H
#define SENSOR_IMU_MSG_H


#include <sensor_msgs/Imu.h>
#include "ros_parser_base.h"

class ImuMsgParser: public RosParserBase
{
public:

    ImuMsgParser()
    {
        _data.emplace_back( "/header/seq" );
        _data.emplace_back( "/header/stamp" );

        _data.emplace_back( "/orientation/x" );
        _data.emplace_back( "/orientation/y" );
        _data.emplace_back( "/orientation/z" );
        _data.emplace_back( "/orientation/w" );

        _data.emplace_back( "/orientation/roll_deg" );
        _data.emplace_back( "/orientation/pitch_deg" );
        _data.emplace_back( "/orientation/yaw_deg" );

        _data.emplace_back( "/angular_velocity/x" );
        _data.emplace_back( "/angular_velocity/y" );
        _data.emplace_back( "/angular_velocity/z" );

        _data.emplace_back( "/linear_acceleration/x" );
        _data.emplace_back( "/linear_acceleration/y" );
        _data.emplace_back( "/linear_acceleration/z" );

        for(int i=0; i<6; i++)
        {
            for(int j=i; j<6; j++)
            {
                char buffer[100];
                sprintf(buffer,"/orientation_covariance/[%d,%d]",i,j);
                _data.emplace_back(buffer);

                sprintf(buffer,"/angular_velocity_covariance/[%d,%d]",i,j);
                _data.emplace_back(buffer);

                sprintf(buffer,"/linear_acceleration_covariance/[%d,%d]",i,j);
                _data.emplace_back(buffer);
            }
        }
    }

    static const std::string& getCompatibleKey()
    {
        static std::string str = ros::message_traits::MD5Sum<sensor_msgs::Imu>::value();
        return str;
    }

    const std::unordered_set<std::string>& getCompatibleKeys() const override
    {
        static std::unordered_set<std::string> temp = { getCompatibleKey() };
        return temp;
    }

    virtual void pushMessageRef(const std::string& ,
                                const MessageRef& msg,
                                double timestamp) override
    {
        sensor_msgs::Imu imu;
        ros::serialization::IStream is( const_cast<uint8_t*>(msg.data()), msg.size() );
        ros::serialization::deserialize(is, imu);

        if( _use_header_stamp )
        {
            timestamp = imu.header.stamp.toSec();
        }

        int count = 0;
        _data[count++].pushBack( {timestamp, static_cast<double>(imu.header.seq)} );
        _data[count++].pushBack( {timestamp, imu.header.stamp.toSec()} );

        _data[count++].pushBack( {timestamp, imu.orientation.x} );
        _data[count++].pushBack( {timestamp, imu.orientation.y} );
        _data[count++].pushBack( {timestamp, imu.orientation.z} );
        _data[count++].pushBack( {timestamp, imu.orientation.w} );

        const auto& q = imu.orientation;
        double roll, pitch, yaw;
        // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
        double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q.w * q.y - q.z * q.x);
        if (std::abs(sinp) >= 1){
          pitch = std::copysign(M_PI_2, sinp); // use 90 degrees if out of range
        }
        else{
          pitch = std::asin(sinp);
        }

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        yaw = std::atan2(siny_cosp, cosy_cosp);

        const double RAD_TO_DEG = 180.0/M_PI;

        _data[count++].pushBack( {timestamp, RAD_TO_DEG * roll} );
        _data[count++].pushBack( {timestamp, RAD_TO_DEG * pitch} );
        _data[count++].pushBack( {timestamp, RAD_TO_DEG * yaw} );

        _data[count++].pushBack( {timestamp, imu.angular_velocity.x} );
        _data[count++].pushBack( {timestamp, imu.angular_velocity.y} );
        _data[count++].pushBack( {timestamp, imu.angular_velocity.z} );

        _data[count++].pushBack( {timestamp, imu.linear_acceleration.x} );
        _data[count++].pushBack( {timestamp, imu.linear_acceleration.y} );
        _data[count++].pushBack( {timestamp, imu.linear_acceleration.z} );


        for(int i=0; i<6; i++)
        {
            for(int j=i; j<6; j++)
            {
                _data[count++].pushBack( {timestamp, imu.orientation_covariance[i*6+j]} );
                _data[count++].pushBack( {timestamp, imu.angular_velocity_covariance[i*6+j]} );
                _data[count++].pushBack( {timestamp, imu.linear_acceleration_covariance[i*6+j]} );
            }
        }
    }

    void extractData(PlotDataMapRef& plot_map, const std::string& prefix) override
    {
        for (auto& it: _data)
        {
            appendData(plot_map, prefix + it.name(), it);
        }
    }

private:
    std::vector<PlotData> _data;
};

#endif // SENSOR_IMU_MSG_H

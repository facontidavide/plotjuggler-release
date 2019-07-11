#ifndef ODOMETRY_MSG_H
#define ODOMETRY_MSG_H


#include <nav_msgs/Odometry.h>
#include "ros_parser_base.h"

class OdometryMsgParser: public RosParserBase
{
public:

    OdometryMsgParser()
    {
        _data.emplace_back( "/header/seq" ); //0
        _data.emplace_back( "/header/stamp" ); //1

        _data.emplace_back( "/pose/position/x" ); //2
        _data.emplace_back( "/pose/position/y" ); //3
        _data.emplace_back( "/pose/position/z" ); //4

        _data.emplace_back( "/pose/orientation/quat_x" ); // 5
        _data.emplace_back( "/pose/orientation/quat_y" ); // 6
        _data.emplace_back( "/pose/orientation/quat_z" ); // 7
        _data.emplace_back( "/pose/orientation/quat_w" ); // 8
        _data.emplace_back( "/pose/orientation/yaw_degrees" ); // 9

        _data.emplace_back( "/twist/linear/x" );// 10
        _data.emplace_back( "/twist/linear/y" );// 11
        _data.emplace_back( "/twist/linear/z" );// 12

        _data.emplace_back( "/twist/angular/x" );// 13
        _data.emplace_back( "/twist/angular/y" );// 14
        _data.emplace_back( "/twist/angular/z" );// 15
    }

    static const std::string& getCompatibleKey()
    {
        static std::string str = ros::message_traits::MD5Sum<nav_msgs::Odometry>::value();
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
        nav_msgs::Odometry odom;
        ros::serialization::IStream is( const_cast<uint8_t*>(msg.data()), msg.size() );
        ros::serialization::deserialize(is, odom);

        if( _use_header_stamp )
        {
            timestamp = odom.header.stamp.toSec();
        }


        _data[0].pushBack( {timestamp, static_cast<double>(odom.header.seq)} );
        _data[1].pushBack( {timestamp, odom.header.stamp.toSec()} );

        _data[2].pushBack( {timestamp, odom.pose.pose.position.x} );
        _data[3].pushBack( {timestamp, odom.pose.pose.position.y} );
        _data[4].pushBack( {timestamp, odom.pose.pose.position.z} );


        const auto& q = odom.pose.pose.orientation;
        _data[5].pushBack( {timestamp, q.x} );
        _data[6].pushBack( {timestamp, q.y} );
        _data[7].pushBack( {timestamp, q.z} );
        _data[8].pushBack( {timestamp, q.w} );

        // yaw (z-axis rotation)
        double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        double yaw = atan2(siny_cosp, cosy_cosp) * 180 / M_PI;

        if( _data[9].size() == 0 )
        {
            _yaw_warp = 0;
        }
        else{
            double prev_yaw = _data[9].back().y - _yaw_warp;
            if( prev_yaw - yaw > 355 )
                _yaw_warp += 360;
            else if( yaw - prev_yaw > 355)
                _yaw_warp -= 360;
        }

        _data[9].pushBack( {timestamp, yaw + _yaw_warp} );

        _data[10].pushBack( {timestamp, odom.twist.twist.linear.x} );
        _data[11].pushBack( {timestamp, odom.twist.twist.linear.y} );
        _data[12].pushBack( {timestamp, odom.twist.twist.linear.z} );

        _data[13].pushBack( {timestamp, odom.twist.twist.angular.x} );
        _data[14].pushBack( {timestamp, odom.twist.twist.angular.y} );
        _data[15].pushBack( {timestamp, odom.twist.twist.angular.z} );

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
    double _yaw_warp;
};

#endif // ODOMETRY_MSG_H

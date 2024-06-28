#ifndef ORIENTATION_VISUALIZER_H
#define ORIENTATION_VISUALIZER_H

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <QLabel>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/Marker.h>
#include <QPainter>
#include <QPushButton>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>




namespace orientation_visualizer
{
    class Orientation : public rviz::Panel
    {
        Q_OBJECT
    public:
        explicit Orientation(QWidget* parent = 0);
        virtual ~Orientation();

    protected:
        QLabel* compositeArrowLabel_;
        ros::NodeHandle nh_;
        ros::Subscriber linkStatesSub_;
        ros::Subscriber jointStatesSub_;
        ros::Publisher ArrowPub_;
        ros::Subscriber tfSub_;

        void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg);
        void publishArrowMarkerForJoint(const std::string& joint_frame_id, int id, const std::string& ns, float r, float g, float b, const geometry_msgs::Pose& pose);
        void publishArrowMarker(const std::string& joint_frame_id, const geometry_msgs::Pose& pose, int id, const std::string& ns, float r, float g, float b, bool alignWithYAxis);
        void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
        void updateCompositeImage(double cameraYaw, double wristYaw);
        void drawArrow(QPainter *painter, QPointF center, int angle, const QColor &color, int length);
        void linkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
        double quaternionToYaw(const geometry_msgs::Quaternion& q);
        void publishArrow(const std::string& frame_id, const std::string& ns, int id, double position, const std::string& color);
    private:
        // Optional: Add any additional helper methods or variables here
    };
}

#endif


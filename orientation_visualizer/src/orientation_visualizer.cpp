#include "orientation_visualizer/orientation_visualizer.h"
#include <QVBoxLayout>
#include <QPainter>
#include <QLabel>
#include <gazebo_msgs/LinkStates.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace orientation_visualizer
{
    Orientation::Orientation(QWidget* parent)
        : rviz::Panel(parent), nh_("~")
    {
        QVBoxLayout* layout = new QVBoxLayout(this);
        compositeArrowLabel_ = new QLabel(this);
        layout->addWidget(compositeArrowLabel_);
        setLayout(layout);

        tfSub_ = nh_.subscribe("/tf", 1, &Orientation::tfCallback, this);

        //linkStatesSub_ = nh_.subscribe("/gazebo/link_states", 1, &MultiViewPanel::linkStatesCallback, this);

        jointStatesSub_ = nh_.subscribe("/joint_states",1, &Orientation::jointStatesCallback, this);
        ArrowPub_ = nh_.advertise<visualization_msgs::Marker>("arrow_marker", 1);
        updateCompositeImage(0,0);
    }


    void Orientation::updateCompositeImage(double cameraYaw, double wristYaw) {
        QPixmap compositePixmap(250, 250); // Adjusted for additional drawing space
        compositePixmap.fill(Qt::transparent);

        QPainter painter(&compositePixmap);
        painter.setRenderHint(QPainter::Antialiasing);

        // Draw robot base first
        QRectF baseRect(40, 80, 120, 60);
        painter.setBrush(Qt::lightGray);
        painter.drawRoundedRect(baseRect, 10, 10);

        QPointF baseCenter = baseRect.center();
        QPointF redArrowStartPoint = baseCenter - QPointF(0, 30);
        QPointF blueArrowStartPoint = baseCenter + QPointF(70, 0);

        drawArrow(&painter, redArrowStartPoint, 0, Qt::red, 60);

        drawArrow(&painter, baseCenter, -static_cast<int>(cameraYaw), Qt::green, 40);

        drawArrow(&painter, blueArrowStartPoint, -static_cast<int>(wristYaw), Qt::blue, 30);

        compositeArrowLabel_->setPixmap(compositePixmap);
    }


    void Orientation::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        double headPanAngleRadians = 0.0;
        double wristYawAngleRadians = 0.0;

        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "joint_head_pan") {
                headPanAngleRadians = msg->position[i];
            } else if (msg->name[i] == "joint_wrist_yaw") {
                wristYawAngleRadians = msg->position[i];
            }
        }

        // Convert radians to degrees
        double headPanAngleDegrees = headPanAngleRadians * (180.0 / M_PI);
        double wristYawAngleDegrees = (wristYawAngleRadians * (180.0 / M_PI))-90;
        updateCompositeImage(headPanAngleDegrees, wristYawAngleDegrees);
    }
    void Orientation::drawArrow(QPainter *painter, QPointF center, int angle, const QColor &color, int length) {
        painter->save();
        painter->translate(center);
        painter->rotate(angle);

        // Arrow stick
        painter->setPen(QPen(color, 3)); // Set the color and thickness of the line
        painter->drawLine(0, 0, 0, -length); // Draw line upwards from center

        // Arrow head
        int headSize = 10; // Size of the arrow head
        QPolygon arrowHead;
        arrowHead << QPoint(-headSize, -length) << QPoint(0, -length-headSize)
                  << QPoint(headSize, -length);
        painter->setBrush(color); // Fill color for the arrow head
        painter->drawPolygon(arrowHead);

        painter->restore();
    }



    double Orientation::quaternionToYaw(const geometry_msgs::Quaternion& q) {
        // Calculate yaw (rotation around z-axis)
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);
        return yaw;
    }

    void Orientation::tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {
        for (const auto& transform : msg->transforms) {
            if (transform.child_frame_id == "link_head_pan" || transform.child_frame_id == "link_wrist_yaw") {
                // Extract the pose
                geometry_msgs::Pose pose;
                pose.position.x = transform.transform.translation.x;
                pose.position.y = transform.transform.translation.y;
                pose.position.z = transform.transform.translation.z;

                // Now call publishArrowMarkerForJoint with this new pose
                int id = (transform.child_frame_id == "link_head_pan") ? 1 : 2;
                std::string ns = (transform.child_frame_id == "link_head_pan") ? "head_pan_direction" : "wrist_yaw_direction";
                if (transform.child_frame_id =="link_wrist_yaw"){
                  publishArrowMarker(transform.child_frame_id, pose, id, ns, 0.0, 0.0, 1.0, true);
                }
                else{
                publishArrowMarker(transform.child_frame_id, pose, id, ns, 0.0, 1.0, 0.0, false);
            }
                }
        }
    }



    void Orientation::publishArrowMarker(const std::string& joint_frame_id, const geometry_msgs::Pose& pose, int id, const std::string& ns, float r, float g, float b, bool alignWithYAxis = false) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = joint_frame_id; // Use a consistent reference frame, like "base_link"
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        if (alignWithYAxis) {
            // Set the orientation to align with the Y-axis directly
            tf2::Quaternion orientationAlignedWithY;
            orientationAlignedWithY.setRPY(0, 0, M_PI / 2); // Rotate 90 degrees around Z-axis
            marker.pose.orientation = tf2::toMsg(orientationAlignedWithY);
        } else {
            // Use the original orientation
            marker.pose = pose;
        }

        marker.scale.x = 0.3; // Length
        marker.scale.y = 0.03; // Width
        marker.scale.z = 0.05; // Height
        marker.color.a = 1.0; // Opacity
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;

        ArrowPub_.publish(marker);
    }





    void Orientation::publishArrowMarkerForJoint(const std::string& joint_frame_id, int id, const std::string& ns, float r, float g, float b, const geometry_msgs::Pose& pose) {
        visualization_msgs::Marker marker;
        marker.header.frame_id =joint_frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;

        // Set pose directly from the argument
        marker.pose = pose;

        marker.scale.x = 0.1; // Arrow length
        marker.scale.y = 0.02; // Arrow width
        marker.scale.z = 0.02; // Arrow height

        marker.color.a = 1.0;
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;

        ArrowPub_.publish(marker);
    }





      Orientation::~Orientation() {}
}




#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(orientation_visualizer::Orientation, rviz::Panel)

#ifndef SOUND_H
#define SOUND_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QLabel>
#include <QPushButton>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace robot_sound_control
{
    class Sound : public rviz::Panel
    {
        Q_OBJECT
    public:
        explicit Sound(QWidget* parent = nullptr);
        virtual ~Sound();

    protected:

        QPushButton* yesButton_;
        QPushButton* noButton_;
        QPushButton* repeatButton_;
        QPushButton* followmeButton_;
        QPushButton* iwillfollowyouButton_;
        QPushButton* excusemeButton_;
        QPushButton* commandButton;
        QPushButton* itemishereButton;
        QPushButton* pleasewaitButton;
        QPushButton* nextitemButton;
        QPushButton* helpmeButton;
        ros::NodeHandle nh_;
        ros::Publisher sound_pub_;
        void playSound(std::string s);


    protected Q_SLOTS:
            void onYesButtonClicked();
            void onNoButtonClicked();
            void onRepeatButtonClicked();
            void onFollowMeButtonClicked();
            void onExcuseMeButtonClicked();
            void onCommandButtonClicked(const QString &text);
            void onItemIsHereButtonClicked();
            void onPleaseWaitButtonClicked();
            void onIWillFollowYouButtonClicked();
            void onNextItemButtonClicked();
            void onPleaseHelpMeButtonClicked();

    private:
        // Optional: Add any additional helper methods or variables here
    };
}

#endif // MULTIVIEW_PANEL_H


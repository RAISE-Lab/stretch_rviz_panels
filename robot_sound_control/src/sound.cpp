#include "robot_sound_control/sound.h"
#include <QVBoxLayout>
#include <QPushButton>
#include <sound_play/SoundRequest.h>
#include <QInputDialog>

std::string voice = "voice_cmu_us_slt_arctic_hts"; //voice_us1_mbrola
float volume = 1.0;

namespace robot_sound_control {
Sound::Sound(QWidget *parent)
  : rviz::Panel(parent)
 {


  QVBoxLayout* layout = new QVBoxLayout(this);
  
  
  // QLineEdit for user to type in speech
  	
  	
  QLineEdit* speechInput = new QLineEdit(this);
  speechInput->setPlaceholderText("Type text here and press Enter");
  layout->addWidget(speechInput);


  connect(speechInput, &QLineEdit::returnPressed, [this, speechInput]() {
  QString text = speechInput->text();
      if (!text.isEmpty()) {
          onCommandButtonClicked(text);
          speechInput->clear();
      	}
  	});


  QPushButton* yesButton = new QPushButton("Yes", this);
  layout->addWidget(yesButton);
  connect(yesButton, &QPushButton::clicked, this, &Sound::onYesButtonClicked);

  QPushButton* noButton = new QPushButton("No", this);
  layout->addWidget(noButton);
  connect(noButton, &QPushButton::clicked, this, &Sound::onNoButtonClicked);

  QPushButton* repeatButton = new QPushButton("Ask to Repeat", this);
  layout->addWidget(repeatButton);
  connect(repeatButton, &QPushButton::clicked, this, &Sound::onRepeatButtonClicked);

  QPushButton* nextitemButton = new QPushButton("What is the next item?", this);
  layout->addWidget(nextitemButton);
  connect(nextitemButton, &QPushButton::clicked, this, &Sound::onNextItemButtonClicked);

  QPushButton* followmeButton = new QPushButton("Follow Me", this);
  layout->addWidget(followmeButton);
  connect(followmeButton, &QPushButton::clicked, this, &Sound::onFollowMeButtonClicked);

  QPushButton* iwillfollowyouButton = new QPushButton("I will follow you", this);
  layout->addWidget(iwillfollowyouButton);
  connect(iwillfollowyouButton, &QPushButton::clicked, this, &Sound::onIWillFollowYouButtonClicked);

  QPushButton* pleasewaitButton = new QPushButton("Please wait", this);
  layout->addWidget(pleasewaitButton);
  connect(pleasewaitButton, &QPushButton::clicked, this, &Sound::onPleaseWaitButtonClicked);
  
   QPushButton* excusemeButton = new QPushButton("Excuse Me, Please Move", this);
  layout->addWidget(excusemeButton);
  connect(excusemeButton, &QPushButton::clicked, this, &Sound::onExcuseMeButtonClicked);

  QPushButton* itemishereButton = new QPushButton("Item is here", this);
  layout->addWidget(itemishereButton);
  connect(itemishereButton, &QPushButton::clicked, this, &Sound::onItemIsHereButtonClicked);

  QPushButton* helpmeButton = new QPushButton("Help me pick up", this);
  layout->addWidget(helpmeButton);
  connect(helpmeButton, &QPushButton::clicked, this, &Sound::onPleaseHelpMeButtonClicked);


  setLayout(layout);

  sound_pub_ = nh_.advertise<sound_play::SoundRequest>("robotsound", 1);

  
}


  void Sound::onNoButtonClicked() {
    // ROS_INFO("No button clicked");
    std::string s = "No";
    ROS_INFO("[TTS Message] %s", s.c_str());
    Sound::playSound(s);
    // std::string voice = "voice_cmu_us_slt_arctic_hts";
    // float volume = 1.0;

    // ROS_INFO("Saying: %s", s.c_str());
    // ROS_INFO("Voice: %s", voice.c_str());
    // ROS_INFO("Volume: %f", volume);

    // if (sound_pub_ && sound_pub_.getNumSubscribers() > 0) {
    //         sound_play::SoundRequest sound;
    //         sound.sound = sound_play::SoundRequest::SAY;
    //         sound.command = sound_play::SoundRequest::PLAY_ONCE;
    //         sound.volume = volume;
    //         sound.arg = s;
    //         sound.arg2 = voice;

    //         sound_pub_.publish(sound);
    //         // ROS_INFO("Sound message published.");
    //     } else {
    //         ROS_WARN("No subscribers found on the sound topic, not publishing sound.");
    //     }
  }

  void Sound::onYesButtonClicked() {
      std::string s = "Yes";
      ROS_INFO("[TTS Message] %s", s.c_str());
      Sound::playSound(s);
  }

  void Sound::onRepeatButtonClicked() {
    std::string s = "Can you please repeat what you just said?";
      ROS_INFO("[TTS Message] %s", s.c_str());
      Sound::playSound(s);
  }

    void Sound::onFollowMeButtonClicked() {
      std::string s = "Please follow me.";
      ROS_INFO("[TTS Message] %s", s.c_str());
      Sound::playSound(s);
    }

    void Sound::onExcuseMeButtonClicked() {
      std::string s = "Excuse me, I would like to move where you are standing.";
      ROS_INFO("[TTS Message] %s", s.c_str());
      Sound::playSound(s);
    }

  void Sound::onNextItemButtonClicked() {
      std::string s = "What is the next item on your shopping list?";
      ROS_INFO("[TTS Message] %s", s.c_str());
      Sound::playSound(s);
    }

    void Sound::onItemIsHereButtonClicked() {
      std::string s = "The item you are looking for is here.";
      ROS_INFO("[TTS Message] %s", s.c_str());
      Sound::playSound(s);
    }

    void Sound::onPleaseWaitButtonClicked() {
      std::string s = "Please wait.";
      ROS_INFO("[TTS Message] %s", s.c_str());
      Sound::playSound(s);
    }

  void Sound::onIWillFollowYouButtonClicked() {
      std::string s = "I will follow you.";
      ROS_INFO("[TTS Message] %s", s.c_str());
      Sound::playSound(s);
    }

  void Sound::onPleaseHelpMeButtonClicked() {
      std::string s = "Can you please help me pick up this item.";
      ROS_INFO("[TTS Message] %s", s.c_str());
      Sound::playSound(s);
    }  

  void Sound::onCommandButtonClicked(const QString &text) {
  	if (!text.isEmpty()) {
      	std::string command = text.toStdString();
      	// std::string voice = "voice_cmu_us_slt_arctic_hts";
      	// float volume = 1.0;

      	ROS_INFO("[TTS Message] %s", command.c_str());
      	sound_play::SoundRequest sound;
      	sound.sound = sound_play::SoundRequest::SAY;
      	sound.command = sound_play::SoundRequest::PLAY_ONCE;
      	sound.volume = volume;
      	sound.arg = command;
      	sound.arg2 = voice;

      	sound_pub_.publish(sound);
      	// ROS_INFO("Speech published as sound.");
  	}
  }

  void Sound::playSound(std::string s) {
    if (sound_pub_ && sound_pub_.getNumSubscribers() > 0) {
              sound_play::SoundRequest sound;
              sound.sound = sound_play::SoundRequest::SAY;
              sound.command = sound_play::SoundRequest::PLAY_ONCE;
              sound.volume = volume;
              sound.arg = s;
              sound.arg2 = voice;

              sound_pub_.publish(sound);
            //   ROS_INFO("Sound message published.");
          } else {
              ROS_WARN("No subscribers found on the sound topic, not publishing sound.");
          }
  }


    Sound::~Sound() {}
}




#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robot_sound_control::Sound, rviz::Panel)



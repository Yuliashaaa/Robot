/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/Joystick.h>
#include <frc/PWMVictorSPX.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/GenericHID.h>
#include <frc/DigitalOutput.h>
#include <cameraserver/CameraServer.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>


frc::DigitalOutput Pin0(0);    // Пин 0 L298N ворота подъём
frc::DigitalOutput Pin1(1);    // Пин 1 L298N ворота спуск
frc::DigitalOutput Pin2(2);    // Пин 2 L298N цветовое колесо влево
frc::DigitalOutput Pin3(3);    // Пин 3 L298N цветовое колесо вправо
frc::DigitalOutput Pin4(4);    // Пин 4 подъём шаров
frc::DigitalOutput Pin5(5);    // Пин 5 спуск шаров

class Robot : public frc::TimedRobot {
  static void VisionThread()
  {
    cs::UsbCamera camera = frc::CameraServer::GetInstance()->StartAutomaticCapture();
    camera.SetResolution(640, 480);
    cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
    cs::CvSource outputStreamStd = frc::CameraServer::GetInstance()->PutVideo("Gray", 640, 480);
    cv::Mat source;
    cv::Mat output;
    while(true) {
      if (cvSink.GrabFrame(source) == 0) {
        continue;
      }
      cvtColor(source, output, cv::COLOR_BGR2GRAY);
      outputStreamStd.PutFrame(output);
    }
  }
 public:

  Robot() {
    Pin0.frc::DigitalOutput::Set(false);
    Pin1.frc::DigitalOutput::Set(false);
    Pin2.frc::DigitalOutput::Set(false);
    Pin3.frc::DigitalOutput::Set(false);
    Pin4.frc::DigitalOutput::Set(true);
    Pin5.frc::DigitalOutput::Set(true);
    m_robotDrive.SetExpiration(0.1);
    m_timer.Start();
  }

  void AutonomousInit() override {
    m_timer.Reset();
    m_timer.Start();
  }

  void AutonomousPeriodic() override {
    // Drive for 2 seconds
    if (m_timer.Get() < 2.0) {
      // Drive forwards half speed
      m_robotDrive.ArcadeDrive(-0.5, 0.0);
    } else {
      // Stop robot
      m_robotDrive.ArcadeDrive(0.0, 0.0);
    }
  }
 
  private:
 
  void BindPinToButton(frc::DigitalOutput& Pin, frc::DigitalOutput& NOPin, int button, int NObutton){
   if(M_stick.GetRawButton(button) && !M_stick.GetRawButton(NObutton)){
     NOPin.frc::DigitalOutput::Set(false);
     Pin.frc::DigitalOutput::Set(true); 
    } 
    else Pin.frc::DigitalOutput::Set(false);
  }

  void REVBindPinToButton(frc::DigitalOutput& Pin, frc::DigitalOutput& NOPin, int button, int NObutton){
    if(M_stick.GetRawButton(button) && !M_stick.GetRawButton(NObutton)){
      NOPin.frc::DigitalOutput::Set(true);
      Pin.frc::DigitalOutput::Set(false);
    }
    else Pin.frc::DigitalOutput::Set(true);
  }
 
  public:

  void TeleopInit() override {
    std::thread visionThread(VisionThread);
    visionThread.detach();
  }

  void TeleopPeriodic() override {
   /*
    *  MAIN FUNCTION
    */
  
    double coef = m_stick.GetRawButton(5) ? 1 : 0.6;
   
    /*
     *   Первый аргумент -- пин, второй -- кнопка для пина, третий -- кнопка-предохранитель. 
     *   По умолчанию не ставил чтобы не было как с преобразователем
     */
   
    BindPinToButton(Pin0, Pin1, 1, 2);
    BindPinToButton(Pin1, Pin0, 2, 1);
    BindPinToButton(Pin2, Pin3, 3, 4);
    BindPinToButton(Pin3, Pin2, 4, 3);

    /*
     *   Всё так же как и с функцией выше. но они работают обратно
     */
    REVBindPinToButton(Pin4, Pin5, 5, 6);
    REVBindPinToButton(Pin5, Pin4, 6, 5);

    m_robotDrive.TankDrive(m_stick.GetRawAxis(5) * coef, m_stick.GetRawAxis(1) * coef);
  }

  void TestPeriodic() override {}

 private:
  // Robot drive system
  frc::PWMVictorSPX m_left{0};
  frc::PWMVictorSPX m_right{1};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};

  frc::Joystick m_stick{0};
  frc::Joystick M_stick{1};

  
  frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
  frc::Timer m_timer;
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

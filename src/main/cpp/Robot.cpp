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
 

class Robot : public frc::TimedRobot {
 public:
  Robot() {
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
 
  void BindPinToButton(frc::DigitalOutput& Pin, int button, int NObutton){
   if(m_stick.GetRawButton(button) && !m_stick.GetRawButton(NObutton)){
     Pin.frc::DigitalOutput::Set(true); 
    } 
    else{
     Pin.frc::DigitalOutput::Set(false);
    }
  }
 
  public:

  void TeleopInit() override {}

  void TeleopPeriodic() override {
   /*
    *  MAIN FUNCTION
    */
    frc::DigitalOutput Pin0(0);    // Пин 0 L298N ворота подъём
    frc::DigitalOutput Pin1(1);    // Пин 1 L298N ворота спуск
    frc::DigitalOutput Pin2(2);    // Пин 2 L298N цветовое колесо влево
    frc::DigitalOutput Pin3(3);    // Пин 3 L298N цветовое колесо вправо
    frc::DigitalOutput Pin4(4);    // Пин 4 подъём шаров
    frc::DigitalOutput Pin5(5);    // Пин 5 спуск шаров
    
    double coef = m_stick.GetRawButton(5) ? 1 : 0.6;
   
    /*
     *   Первый аргумент -- пин, второй -- кнопка для пина, третий -- кнопка-предохранитель. По умолчанию не ставил, 
     *   Чтобы не было как с преобразователем
     */
   
    BindPinToButton(Pin0, 11, 12);
    BindPinToButton(Pin1, 12, 11);
    BindPinToButton(Pin2, 13, 14);
    BindPinToButton(Pin3, 14, 13);
    BindPinToButton(Pin4, 15, 16);
    BindPinToButton(Pin5, 16, 15);

    m_robotDrive.TankDrive(m_stick.GetRawAxis(5) * coef, m_stick.GetRawAxis(1) * coef);
  }

  void TestPeriodic() override {}

 private:
  // Robot drive system
  frc::PWMVictorSPX m_left{0};
  frc::PWMVictorSPX m_right{1};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};

  frc::Joystick m_stick{0};


  
  frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
  frc::Timer m_timer;
};

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

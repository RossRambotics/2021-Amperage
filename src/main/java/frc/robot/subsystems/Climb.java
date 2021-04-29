// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climb extends SubsystemBase {

  private CANSparkMax m_leftWinch = null;
  private CANSparkMax m_rightWinch = null;
  private CANSparkMax m_rightSlide = null;
  private CANSparkMax m_leftSlide = null;

  private double m_leftWinchRetractSpeed = -.2;
  private double m_rightWinchRetractSpeed = -.2;
  private double m_leftSlidePowerCoefficent = 1;
  private double m_rightSlidePowerCoefficent = -1;

  public Joystick m_smallOperatorJoystick = null;

  /** Creates a new ExampleSubsystem. */
  public Climb() {
    m_leftWinch = new CANSparkMax(7, MotorType.kBrushless);
    m_rightWinch = new CANSparkMax(8, MotorType.kBrushless);
    m_leftSlide = new CANSparkMax(9, MotorType.kBrushless);
    m_rightSlide = new CANSparkMax(10, MotorType.kBrushless);

    m_smallOperatorJoystick = new Joystick(3);

    this.setDefaultCommand(new frc.robot.commands.Climb.RunSlides(this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void leftWinchRetract() { // sets the left winch to retract
    m_leftWinch.set(m_leftWinchRetractSpeed);
  }

  public void leftWinchStop() {
    m_leftWinch.set(0);
  }

  public void rightWinchRetract() {
    m_rightWinch.set(m_rightWinchRetractSpeed);
  }

  public void rightWinchStop() {
    m_rightWinch.set(0);
  }

  public void setRightSlide(double power) {
    if (Math.abs(power) < 0.1) {
      m_rightSlide.set(0);
    }

    m_rightSlide.set(m_rightSlidePowerCoefficent * power);
  }

  public void setLeftSlide(double power) {
    if (Math.abs(power) < 0.1) {
      m_leftSlide.set(0);
    }

    m_leftSlide.set(m_leftSlidePowerCoefficent * power);
  }

}

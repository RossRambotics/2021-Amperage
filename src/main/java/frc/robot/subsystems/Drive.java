// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */

  private WPI_TalonFX m_rightDriveTalon;
  private WPI_TalonFX m_leftDriveTalon;
  private WPI_TalonFX m_rightDriveTalonFollower;
  private WPI_TalonFX m_leftDriveTalonFollower;
  private SpeedControllerGroup m_leftDriveGroup;
  private SpeedControllerGroup m_rightDriveGroup;

  private DifferentialDrive m_differentialDrive;

  public Drive() {
    // right side talons
    m_rightDriveTalon = new WPI_TalonFX(21);
    m_rightDriveTalonFollower = new WPI_TalonFX(23);

    // left side talons
    m_leftDriveTalon = new WPI_TalonFX(22);
    m_leftDriveTalonFollower = new WPI_TalonFX(24);

    // speed controller groups
    m_leftDriveGroup = new SpeedControllerGroup(m_leftDriveTalon, m_leftDriveTalonFollower);
    m_rightDriveGroup = new SpeedControllerGroup(m_rightDriveTalon, m_rightDriveTalonFollower);

    // create the differential drive
    m_differentialDrive = new DifferentialDrive(m_leftDriveGroup, m_rightDriveGroup);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void tankDrive(Double leftSpeed, Double rightSpeed) {
    m_differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }
}

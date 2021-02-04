// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  /** Creates a new Drive. */

  private Double m_maxDriveOutput = 0.5;
  private NetworkTableEntry m_maxDriveOutputEntry = null;


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

    //configures the inversion and peak output for the talons
    m_rightDriveTalon.setInverted(true);
    m_rightDriveTalonFollower.setInverted(true);
   
    // left side talons
    m_leftDriveTalon = new WPI_TalonFX(22);
    m_leftDriveTalon.setInverted(true);
    m_leftDriveTalonFollower = new WPI_TalonFX(24);
    m_leftDriveTalonFollower.setInverted(true);

    //configures the inversion and peak output for the talons
    m_leftDriveTalon.setInverted(true);
    m_leftDriveTalonFollower.setInverted(true);
    
    // speed controller groups
    m_leftDriveGroup = new SpeedControllerGroup(m_leftDriveTalon, m_leftDriveTalonFollower);
    m_rightDriveGroup = new SpeedControllerGroup(m_rightDriveTalon, m_rightDriveTalonFollower);

    // create the differential drive
    m_differentialDrive = new DifferentialDrive(m_leftDriveGroup, m_rightDriveGroup);

    setPeakOutputs(m_maxDriveOutput);
    createShuffleBoardTab();
  }
  
  public void createShuffleBoardTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Sub.Indexer");

    ShuffleboardLayout shooterCommands = tab.getLayout("Drive Calibration", BuiltInLayouts.kList).withSize(2, 2)
        .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

    m_maxDriveOutputEntry = shooterCommands.add("MAX POWER!", 0.5).withWidget(BuiltInWidgets.kNumberSlider)
        .withSize(4, 1).withProperties(Map.of("min", 0, "max", 1)).getEntry();
  }
  @Override
  public void periodic() {

    if(m_maxDriveOutput != m_maxDriveOutputEntry.getDouble(0.5)){
      m_maxDriveOutput = m_maxDriveOutputEntry.getDouble(0.5);
      setPeakOutputs(m_maxDriveOutput);
    }
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void tankDrive(Double leftSpeed, Double rightSpeed) {
    m_differentialDrive.tankDrive(leftSpeed, rightSpeed);
  }

  private void setPeakOutputs(Double peakOutput) //sets the max peak outputs on a motor to prevent a brownout
  {
    m_leftDriveTalon.configPeakOutputForward(peakOutput);
    m_leftDriveTalon.configPeakOutputReverse(-peakOutput);
    m_leftDriveTalonFollower.configPeakOutputForward(peakOutput); // you must set this on followers too...
    m_leftDriveTalonFollower.configPeakOutputReverse(-peakOutput); // you must set this on followers too...

    m_rightDriveTalon.configPeakOutputForward(peakOutput);
    m_rightDriveTalon.configPeakOutputReverse(-peakOutput);
    m_rightDriveTalonFollower.configPeakOutputForward(peakOutput); // you must set this on followers too...
    m_rightDriveTalonFollower.configPeakOutputReverse(-peakOutput); // you must set this on followers too...
  }
}

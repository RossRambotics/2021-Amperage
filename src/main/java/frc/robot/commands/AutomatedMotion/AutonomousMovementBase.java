package frc.robot.commands.AutomatedMotion;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class AutonomousMovementBase extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drive m_drive;

  private double m_targetMeters; // target distance to travel
  private double m_targetFinalStepsLeft; // the intended steps at the end of the command
  private double m_targetFinalStepsRight; // the intended steps at the end of the command
  private double m_turnKd; // pid for turning -- drives striaght
  private double m_turnKi;// pid for turning -- drives striaght
  private double m_turnKp;// pid for turning -- drives striaght
  private double m_initalYaw; // the initial heading for the robot
  private double m_previousYaw; // the heading of the robot in the previous loop
  private double m_turnErrorSum; // the turn error sum
  private double m_previousDistanceRemaining;
  private double m_distanceRemaining;
  private List<Double> m_straightErrorArray = new ArrayList<Double>(); // used for calculating I
  private double m_straightErrorSum; // used for calculating I
  private double m_intialStepsLeft;
  private double m_timeSinceLastUpdate = 0;
  private double m_intialStepsRight; // the inital step count when the command begins
  private Timer m_straightTimer; // the straight correction timer must be different from the turn timer because
                                 // the falcon 500 encoders do not update every loop of the robot code
  private Timer m_turnTimer;

  public AutonomousMovementBase(Drive drive, double targetMeters, double targetMaxVelocity) { // max velocity in mps
    m_drive = drive;
    m_targetMeters = targetMeters;

    m_turnKp = m_drive.m_handlingValues.getAngleAdjustmentkP();
    m_turnKi = 0; // m_drive.m_handlingValues.getAngleAdjustmentkI();
    m_turnKd = m_drive.m_handlingValues.getAngleAdjustmentkD();

    addRequirements(drive);
    m_straightTimer = new Timer();
    m_turnTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intialStepsLeft = m_drive.getLeftTalonEncoderPosition();
    m_intialStepsRight = m_drive.getRightTalonEncoderPostion();
    m_initalYaw = m_drive.getPigeonYaw();
    m_previousYaw = m_initalYaw;

    m_targetFinalStepsLeft = m_intialStepsLeft + m_targetMeters / m_drive.getDistancePerStep();
    m_targetFinalStepsRight = m_intialStepsRight + m_targetMeters / m_drive.getDistancePerStep();

    m_previousDistanceRemaining = (Math.abs(m_drive.getLeftTalonEncoderPosition() - m_targetFinalStepsLeft)
        + Math.abs(m_drive.getRightTalonEncoderPostion() - m_targetFinalStepsRight)) / 2;
    m_distanceRemaining = m_previousDistanceRemaining;

    m_straightTimer.start();
    m_straightTimer.reset(); // start the timer over again
    m_turnTimer.start();
    m_turnTimer.reset(); // start the timer over again
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentDistanceRemaining = ((m_drive.getLeftTalonEncoderPosition() - m_targetFinalStepsLeft)
        + (m_drive.getRightTalonEncoderPostion() - m_targetFinalStepsRight)) / 2;
    double targetVelocity = getTargetVelocityFromDistanceRemaining(currentDistanceRemaining); // velocities in 1 per
                                                                                              // 100ms
    double currentVelocity = m_drive.getAverageRobotEncoderVelocity();

    if (currentDistanceRemaining != m_distanceRemaining) { // tells wether or not the encoders updated
      m_previousDistanceRemaining = m_distanceRemaining; // shifts the distance remaining value
      m_distanceRemaining = currentDistanceRemaining;
      m_timeSinceLastUpdate = m_straightTimer.get();
      m_straightTimer.reset();

      if (Math.abs(targetVelocity - currentVelocity) < 1500) {
        m_straightErrorArray.add(m_timeSinceLastUpdate * (targetVelocity - currentVelocity));
        if (m_straightErrorArray.size() >= 7) {
          m_straightErrorArray.remove(0);
        }

        m_straightErrorSum = 0;
        for (int i = 0; i < m_straightErrorArray.size(); i++) {
          m_straightErrorSum = m_straightErrorArray.get(i);
        }
      } // do not accumlate error under hard acceleration
    }

    double pValue = m_drive.m_handlingValues.getStraightVelocityControlkP() * (targetVelocity - currentVelocity);
    double iValue = m_drive.m_handlingValues.getStraightVelocityControlkI() * m_straightErrorSum;

    double straightValue = pValue + iValue;// - dValue - ffValue;
    /*
     * if (currentDistanceRemaining < 0) { straightValue = straightValue * -1; }
     */

    System.out.println("Distance Remaining: " + currentDistanceRemaining + " Ivalue:" + iValue);
    System.out
        .println("pValue: " + pValue + " Target Velocity: " + targetVelocity + " Current Velocity: " + currentVelocity);

    straightValue = Math.min(1, straightValue);
    straightValue = Math.max(-1, straightValue);

    // Makes it drive straight
    double currentYaw = m_drive.getPigeonYaw();
    double secondsSinceLastLoop = m_turnTimer.get();
    m_turnTimer.reset();
    double dCorrection = m_turnKd * (currentYaw - m_previousYaw) / secondsSinceLastLoop; // degrees over seconds
    double pCorrection = m_turnKp * (currentYaw - m_initalYaw);
    double iCorrection = m_turnKi * m_turnErrorSum * secondsSinceLastLoop;
    double totalCorrection = -dCorrection - pCorrection - iCorrection;
    totalCorrection = -totalCorrection; // invert to correct for gyro direction
    // if this value is positive speed up right motor or slow left
    // if this value is negative speed up left motor or slow right
    // note both motors and joystcicks are inverted ;)

    if (straightValue == 0) { // adds deadzone to prevent the bot from moving while stopped
      return;
    }

    if (straightValue > 0) { // if the robot is moving backward
      if (totalCorrection > 0) {
        double leftValue = straightValue + totalCorrection;
        leftValue = Math.min(1, leftValue); // ensure the values are in the range
        leftValue = Math.max(-1, leftValue);

        m_drive.tankDriveRaw(leftValue, straightValue);// if total correction is positive slow left
      } else {
        double rightValue = straightValue - totalCorrection;
        rightValue = Math.min(1, rightValue); // ensure the values are in the range
        rightValue = Math.max(-1, rightValue);

        m_drive.tankDriveRaw(straightValue, rightValue);// if total correction is positive slow right
      }
    } else { // if the robot is moving forward
      if (totalCorrection > 0) {
        double leftValue = straightValue + totalCorrection;
        leftValue = Math.min(1, leftValue); // ensure the values are in the range
        leftValue = Math.max(-1, leftValue);

        m_drive.tankDriveRaw(leftValue, straightValue);// if total correction is positive slow left
      } else {
        double rightValue = straightValue - totalCorrection;
        rightValue = Math.min(1, rightValue); // ensure the values are in the range
        rightValue = Math.max(-1, rightValue);

        m_drive.tankDriveRaw(straightValue, rightValue);// if total correction is positive slow right
      }
    }

    m_turnErrorSum = m_turnErrorSum + (currentYaw - m_initalYaw); // before or after current round of calculations??
    m_previousYaw = currentYaw;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_previousDistanceRemaining) < 1000) {
      return true;
    }

    return false;
  }

  public double getTargetVelocityFromDistanceRemaining(double distanceRemaining) {
    double cruiseVelocity = 12000;
    double slowVelocity = 1000;
    double slowZone = 50000;
    if (Math.abs(distanceRemaining) < slowZone) {
      if (distanceRemaining < 0) {
        slowVelocity = -slowVelocity;
        cruiseVelocity = -cruiseVelocity;
      }

      return slowVelocity + -(cruiseVelocity - slowVelocity) * distanceRemaining / slowZone;
    }

    if (distanceRemaining < 0) {
      cruiseVelocity = -cruiseVelocity;
    }

    return cruiseVelocity;
  }
}
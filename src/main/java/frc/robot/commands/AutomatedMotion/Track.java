package frc.robot.commands.AutomatedMotion;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.helper.Tracking.PowerCellTracker;
import frc.robot.subsystems.Drive;

public class Track extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Drive m_drive;

    private double m_errorSum = 0; // also known as the intgral of error
    private double m_previousYaw; // from the previous execute loop
    private Timer m_timer; // allows for the calculation of the derivative
    private double m_targetYaw; // the target heading for the robot
    private double m_previousFrameCount; // the most recent acknowledged frame

    private double m_Kp = 0.006;
    private double m_Ki = 0.02;
    private double m_Kd = 0.0013;

    private PowerCellTracker m_powerPowerCellTracker;

    public Track(Drive drive) { // calling joystick corrosponds to port number
        m_drive = drive;
        addRequirements(drive);

        m_timer = new Timer();
        m_powerPowerCellTracker = new PowerCellTracker();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.start();
        m_timer.reset(); // start the timer over again
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("Tracking Power Cell");
        double currentYaw = m_drive.getGyroYaw();

        if(m_previousFrameCount != m_powerPowerCellTracker.getFrameCounter()){ // only sets a new target yaw when a new frame is ready
          m_targetYaw = currentYaw + m_powerPowerCellTracker.getPowerCellAngle(); // defines the target Yaw for the power cell tracker
          m_previousFrameCount = m_powerPowerCellTracker.getFrameCounter();
        }

        double secondsSinceLastLoop = m_timer.get(); // gets the loop time
        m_timer.reset();

        // calculated the error correction
        double dCorrection = m_Kd * (currentYaw - m_previousYaw) / secondsSinceLastLoop; // degrees over seconds
        double pCorrection = m_Kp * (currentYaw - m_targetYaw);
        double iCorrection = m_Ki * m_errorSum * secondsSinceLastLoop;
        double totalCorrection = dCorrection + pCorrection + iCorrection;
        //totalCorrection = -totalCorrection; // invert to correct for gyro direction
        // if this value is positive speed up right motor or slow left
        // if this value is negative speed up left motor or slow right
        // note both motors and joystcicks are inverted ;)

        double basePower = 0.1; // the base value for moving to power cells
        //should move the robot in reverse

        if (basePower > 0) { // if the robot is moving backward
            if (totalCorrection > 0) {
                double leftValue = basePower + totalCorrection;
                leftValue = Math.min(1, leftValue); // ensure the values are in the range
                leftValue = Math.max(-1, leftValue);

                m_drive.tankDrive(leftValue, basePower);// if total correction is positive slow left
            } else {
                double rightValue = basePower - totalCorrection;
                rightValue = Math.min(1, rightValue); // ensure the values are in the range
                rightValue = Math.max(-1, rightValue);

                m_drive.tankDrive(basePower, rightValue);// if total correction is positive slow right
            }
        } else { // if the robot is moving forward
            if (totalCorrection > 0) {
                double leftValue = basePower + totalCorrection;
                leftValue = Math.min(1, leftValue); // ensure the values are in the range
                leftValue = Math.max(-1, leftValue);

                m_drive.tankDrive(leftValue, basePower);// if total correction is positive slow left
            } else {
                double rightValue = basePower - totalCorrection;
                rightValue = Math.min(1, rightValue); // ensure the values are in the range
                rightValue = Math.max(-1, rightValue);

                m_drive.tankDrive(basePower, rightValue);// if total correction is positive slow right
            }
        }

        m_errorSum = m_errorSum + (currentYaw - m_targetYaw); // before or after current round of calculations??
        m_previousYaw = currentYaw;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      if(!m_powerPowerCellTracker.getPowerCellFound()){ // if the powercell is not found  -- quit
        return true;
      }

        return false;
    }
}

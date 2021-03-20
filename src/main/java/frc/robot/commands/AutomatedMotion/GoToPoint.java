package frc.robot.commands.AutomatedMotion;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.helper.WayPoint;
import frc.robot.helper.Tracking.PowerCellTracker;
import frc.robot.subsystems.Drive;

public class GoToPoint extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Drive m_drive;

    private double m_errorSum = 0; // also known as the intgral of error
    private double m_previousYaw; // from the previous execute loop
    private Timer m_timer; // allows for the calculation of the derivative
    private double m_previousFrameCount; // the most recent acknowledged frame

    private double m_basePower = -0.4; // the base value for moving to power cells
    private double m_Kp = 0.08;
    private double m_Ki = 0.02;
    private double m_Kd = 0.0013;

    private int m_reportCounter = 10;

    private WayPoint m_WayPoint;

    public GoToPoint(Drive drive, WayPoint wayPoint) { // calling joystick corrosponds to port number
        m_drive = drive;
        m_WayPoint = wayPoint;
        addRequirements(drive);

        m_Kp = m_Kp * Math.abs(m_basePower);
        m_timer = new Timer();
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
        double[] absolutePosition = m_drive.getAbsolutePosition(); // gets the absolute position of the robot
        double xPosition = absolutePosition[0];
        double yPosition = absolutePosition[1];
        double currentYaw = absolutePosition[2];

        double targetYaw = m_WayPoint.getHeadingTo(xPosition, yPosition);
        double relativeTurn = -targetYaw + currentYaw;
        if (relativeTurn < -180) {
            relativeTurn = relativeTurn + 360;
        } else if (relativeTurn > 180) {
            relativeTurn = relativeTurn - 360;
        }

        double basePower = m_basePower;
        if (Math.abs(relativeTurn) > 90) {
            basePower = -basePower; // instead of turning around, go backwards

            if (relativeTurn > 0) {
                relativeTurn = relativeTurn - 180;
            } else if (relativeTurn < 0) {
                relativeTurn = relativeTurn + 180;
            }
        }

        m_WayPoint.getDistanceFrom(xPosition, yPosition);

        double secondsSinceLastLoop = m_timer.get(); // gets the loop time
        m_timer.reset();

        // calculated the error correction
        double dCorrection = m_Kd * (currentYaw - m_previousYaw) / secondsSinceLastLoop; // degrees over seconds
        double pCorrection = m_Kp * (relativeTurn);
        // double iCorrection = m_Ki * m_errorSum * secondsSinceLastLoop;
        double totalCorrection = (dCorrection + pCorrection);// - iCorrection;
        // totalCorrection = -totalCorrection; // invert to correct for gyro direction
        // if this value is positive speed up right motor or slow left
        // if this value is negative speed up left motor or slow right
        // note both motors and joystcicks are inverted ;)

        if (m_reportCounter >= 10) {
            System.out.println("RelaitveTurn: " + relativeTurn);
            System.out.println("pValue: " + pCorrection + " dValue: " + dCorrection);// + " errorSum: " + iCorrection);
            System.out.println("Gyro Heading: " + currentYaw + " TargetAngle: " + targetYaw);
            m_reportCounter = 0;
        } else {
            m_reportCounter = m_reportCounter + 1;
        }
        if (basePower > 0) { // if the robot is moving backward
            if (totalCorrection < 0) {
                double leftValue = basePower + totalCorrection;
                leftValue = Math.min(1, leftValue); // ensure the values are in the range
                leftValue = Math.max(-1, leftValue);

                m_drive.tankDriveRaw(leftValue, basePower);// if total correction is positive slow left
            } else {
                double rightValue = basePower - totalCorrection;
                rightValue = Math.min(1, rightValue); // ensure the values are in the range
                rightValue = Math.max(-1, rightValue);

                m_drive.tankDriveRaw(basePower, rightValue);// if total correction is positive slow right
            }
        } else { // if the robot is moving forward
            if (totalCorrection > 0) {
                double leftValue = basePower + totalCorrection;
                leftValue = Math.min(1, leftValue); // ensure the values are in the range
                leftValue = Math.max(-1, leftValue);

                m_drive.tankDriveRaw(leftValue, basePower);// if total correction is positive slow left
            } else {
                double rightValue = basePower - totalCorrection;
                rightValue = Math.min(1, rightValue); // ensure the values are in the range
                rightValue = Math.max(-1, rightValue);

                m_drive.tankDriveRaw(basePower, rightValue);// if total correction is positive slow right
            }
        }

        m_errorSum = m_errorSum + (currentYaw - targetYaw); // before or after current round of calculations??
        m_previousYaw = currentYaw;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_WayPoint.cleared) { // if the waypoint has been reached
            System.out.println("Way Point Cleared");
            return true;
        }

        return false;
    }
}

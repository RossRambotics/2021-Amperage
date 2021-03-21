package frc.robot.commands.AutomatedMotion;

import java.util.List;

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
    private double m_Kp = 0.05;
    private double m_Ki = 0.02;
    private double m_Kd = 0.0013;

    private int m_reportCounter = 10;

    private List<WayPoint> m_WayPoints;

    public GoToPoint(Drive drive, List<WayPoint> wayPoints) { // calling joystick corrosponds to port number
        m_drive = drive;
        m_WayPoints = wayPoints;
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

        double targetYaw = getTargetYaw(xPosition, yPosition);

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

        for (WayPoint waypoint : m_WayPoints) {
            waypoint.getDistanceFrom(xPosition, yPosition);
        }

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
        int counter = 0; // counts the loop
        for (WayPoint waypoint : m_WayPoints) {
            if (waypoint.cleared) {
                System.out.println("Way Point Cleared");

                if (counter > 0) { // clear the previous waypoints
                    while (counter >= 0) {
                        counter = counter - 1;
                        m_WayPoints.get(counter).cleared = true;
                    }
                }

                return true;
            }
            counter = counter + 1;
        }

        return false;
    }

    private double getTargetYaw(double xPosition, double yPosition) {
        double cumulativeX = 0;
        double cumulativeY = 0;
        double cumulativeCounter = 0;
        double counter = 1;
        double counterIncrement = 1;

        for (WayPoint waypoint : m_WayPoints) {
            cumulativeX = cumulativeX + (waypoint.getAbsoluteX() - xPosition) * counter;
            cumulativeY = cumulativeY + (waypoint.getAbsoluteY() - yPosition) * counter;
            cumulativeCounter = cumulativeCounter + counter;

            counter = counter + counterIncrement;
        }

        double averageX = cumulativeX / cumulativeCounter;
        double averageY = cumulativeY / cumulativeCounter;

        return getHeadingTo(averageX, averageY);

    }

    private double getHeadingTo(double xTarget, double yTagret) {

        // acot because y is straight ahead and x is side to side -- x and y are flipped
        double radians = Math.PI / 2;

        if (Math.abs(yTagret) > .00001) { // prevents NAN errorz
            radians = -Math.atan((xTarget) / (yTagret));
        }

        if ((yTagret) < 0) { // accounts for the return of arc only covering half the range
            radians = radians + Math.PI;
        }

        if (radians < 0) { // normalizes to 0 to 2pi scale
            radians = radians + 2 * Math.PI;
        }

        return 57.32 * radians; // degrees

    }
}

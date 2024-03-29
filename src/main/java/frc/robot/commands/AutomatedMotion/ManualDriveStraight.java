package frc.robot.commands.AutomatedMotion;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drive;

public class ManualDriveStraight extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Drive m_drive;
    private double m_callingJoystick;

    private double m_initalYaw = 0; // aka the target direction
    private double m_errorSum = 0; // also known as the intgral of error
    private double m_previousYaw; // from the previous execute loop
    private Timer m_timer; // allows for the calculation of the derivative

    private double m_Kp = 0.005;
    private double m_Ki = 0.01;
    private double m_Kd = 0.0013;

    public ManualDriveStraight(Drive drive, double callingJoystick) { // calling joystick corrosponds to port number
        m_callingJoystick = callingJoystick;
        m_drive = drive;
        addRequirements(drive);
        m_timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.start();
        m_timer.reset(); // start the timer over again
        m_initalYaw = m_drive.getPigeonYaw();
        m_previousYaw = m_initalYaw; // so it has a reasonable value
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentYaw = m_drive.getPigeonYaw();
        double secondsSinceLastLoop = m_timer.get(); // gets the loop time
        m_timer.reset();

        System.out.println(currentYaw);

        // calculated the error correction
        double dCorrection = m_Kd * (currentYaw - m_previousYaw) / secondsSinceLastLoop; // degrees over seconds
        double pCorrection = m_Kp * (currentYaw - m_initalYaw);
        double iCorrection = m_Ki * m_errorSum * secondsSinceLastLoop;
        double totalCorrection = -dCorrection - pCorrection - iCorrection;
        totalCorrection = -totalCorrection; // invert to correct for gyro direction
        // if this value is positive speed up right motor or slow left
        // if this value is negative speed up left motor or slow right
        // note both motors and joystcicks are inverted ;)

        System.out.println("Total Correction: " + totalCorrection);
        double joystickValue = 0;
        if (m_callingJoystick == 0) {
            joystickValue = m_drive.getRightJoystickY();
        } else {
            joystickValue = m_drive.getLeftJoystickY();
        }

        if (joystickValue > 0) { // if the robot is moving backward
            if (totalCorrection > 0) {
                double leftValue = joystickValue + totalCorrection;
                leftValue = Math.min(1, leftValue); // ensure the values are in the range
                leftValue = Math.max(-1, leftValue);

                m_drive.tankDrive(leftValue, joystickValue);// if total correction is positive slow left
            } else {
                double rightValue = joystickValue - totalCorrection;
                rightValue = Math.min(1, rightValue); // ensure the values are in the range
                rightValue = Math.max(-1, rightValue);

                m_drive.tankDrive(joystickValue, rightValue);// if total correction is positive slow right
            }
        } else { // if the robot is moving forward
            if (totalCorrection > 0) {
                double leftValue = joystickValue + totalCorrection;
                leftValue = Math.min(1, leftValue); // ensure the values are in the range
                leftValue = Math.max(-1, leftValue);

                m_drive.tankDrive(leftValue, joystickValue);// if total correction is positive slow left
            } else {
                double rightValue = joystickValue - totalCorrection;
                rightValue = Math.min(1, rightValue); // ensure the values are in the range
                rightValue = Math.max(-1, rightValue);

                m_drive.tankDrive(joystickValue, rightValue);// if total correction is positive slow right
            }
        }

        m_errorSum = m_errorSum + (currentYaw - m_initalYaw); // before or after current round of calculations??
        m_previousYaw = currentYaw;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

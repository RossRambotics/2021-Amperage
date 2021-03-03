package frc.robot.commands.AutomatedMotion;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drive;

public class AutonomousMovementBaseII extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Drive m_drive;
    private double m_callingJoystick;

    private double m_initalYaw = 0; // aka the target direction
    private double m_errorSum = 0; // also known as the intgral of error
    private double m_previousYaw; // from the previous execute loop
    private Timer m_timer; // allows for the calculation of the derivative

    private double m_Kp = 0.006;
    private double m_Ki = 0.02;
    private double m_Kd = 0.0013;

    private double m_boostCoefficent = .35; // relates the PDP volate to the max power the boost function can use

    public AutonomousMovementBaseII(Drive drive, double distance) { // calling joystick corrosponds to port
                                                                             // number
       // m_callingJoystick = callingJoystick;
        m_drive = drive;
        addRequirements(drive);
        m_timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_timer.start();
        m_timer.reset(); // start the timer over again
        m_initalYaw = m_drive.getGyroYaw();
        m_previousYaw = m_initalYaw; // so it has a reasonable value
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentYaw = m_drive.getGyroYaw();
        double secondsSinceLastLoop = m_timer.get(); // gets the loop time
        m_timer.reset();

        // calculated the error correction
        double dCorrection = m_Kd * (currentYaw - m_previousYaw) / secondsSinceLastLoop; // degrees over seconds
        double pCorrection = m_Kp * (currentYaw - m_initalYaw);
        double iCorrection = m_Ki * m_errorSum * secondsSinceLastLoop;
        double totalCorrection = -dCorrection - pCorrection - iCorrection;
        totalCorrection = -totalCorrection; // invert to correct for gyro direction
        // if this value is positive speed up right motor or slow left
        // if this value is negative speed up left motor or slow right
        // note both motors and joystcicks are inverted ;)

        double maxPower = m_drive.getPDPVoltage() * m_boostCoefficent;
        double joystickValue = 0; // used to find the direction of movement
        if (m_callingJoystick == 0) {
            joystickValue = m_drive.getRightJoystickY();
        } else {
            joystickValue = m_drive.getLeftJoystickY();
        }

        if (joystickValue > 0) {
            maxPower = -maxPower;
        }

        if (maxPower == 0) { // adds deadzone to prevent the bot from moving while stopped
            return;
        }

        if (maxPower > 0) { // if the robot is moving backward
            if (totalCorrection > 0) {
                double leftValue = maxPower + totalCorrection;
                leftValue = Math.min(1, leftValue); // ensure the values are in the range
                leftValue = Math.max(-1, leftValue);

                m_drive.tankDrive(leftValue, maxPower);// if total correction is positive slow left
            } else {
                double rightValue = maxPower - totalCorrection;
                rightValue = Math.min(1, rightValue); // ensure the values are in the range
                rightValue = Math.max(-1, rightValue);

                m_drive.tankDrive(maxPower, rightValue);// if total correction is positive slow right
            }
        } else { // if the robot is moving forward
            if (totalCorrection > 0) {
                double leftValue = maxPower + totalCorrection;
                leftValue = Math.min(1, leftValue); // ensure the values are in the range
                leftValue = Math.max(-1, leftValue);

                m_drive.tankDrive(leftValue, maxPower);// if total correction is positive slow left
            } else {
                double rightValue = maxPower - totalCorrection;
                rightValue = Math.min(1, rightValue); // ensure the values are in the range
                rightValue = Math.max(-1, rightValue);

                m_drive.tankDrive(maxPower, rightValue);// if total correction is positive slow right
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

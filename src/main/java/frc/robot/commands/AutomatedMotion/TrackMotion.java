package frc.robot.commands.AutomatedMotion;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class TrackMotion extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private final Drive m_drive;
    private double m_currentHeading = 0; // the direction the robot is currently facing in degrees
    private double m_currentYPosition = 0; // the distance moved from the starting position parellel to the direction
                                           // the robot initially faced
    private double m_currentXPosition = 0; // the distnance moved from the starting position perpendicular to the
                                           // direction the robot initially faced
    private double m_currentLeftSteps; // the left steps at the current time
    private double m_currentRightSteps; // the right steps at the current time

    public TrackMotion(Drive drive) { // calling joystick corrosponds to port number
        m_drive = drive;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_currentLeftSteps = m_drive.getLeftTalonEncoderPosition();
        m_currentRightSteps = m_drive.getRightTalonEncoderPostion();

        m_currentYPosition = 0;
        m_currentXPosition = 0;
        m_currentHeading = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double newRightSteps = m_drive.getRightTalonEncoderPostion();

        if (newRightSteps != m_currentRightSteps) { // only runs if the encoders have updated
            double newLeftSteps = m_drive.getLeftTalonEncoderPosition();

            double[] relativePosition = m_drive.getXYTranslationFromEncoderMovement(newRightSteps - m_currentRightSteps,
                    newLeftSteps - m_currentLeftSteps, m_currentHeading);
            // calcuates the ABSOLUTE heading and relative Y and X position

            // updates the position
            m_currentHeading = relativePosition[2];
            m_currentXPosition = m_currentXPosition + relativePosition[1];
            m_currentYPosition = m_currentYPosition + relativePosition[0];

            // updates the current encoder counts
            m_currentRightSteps = newRightSteps;
            m_currentLeftSteps = newLeftSteps;
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        System.out.println("Final Heading: " + m_currentHeading);
        System.out.println("Final Y: " + m_currentYPosition * m_drive.getDistancePerStep() + " Final X: "
                + m_currentXPosition * m_drive.getDistancePerStep());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

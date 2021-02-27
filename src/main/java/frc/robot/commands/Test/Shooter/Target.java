package frc.robot.commands.Test.Shooter;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class Target extends CommandBase {
    private Drive m_drive = null;
    private Timer m_timer; // used for calculating rotational velocity

    private double m_deadzoneAngle = 0.2;// the maximum error that the command will return with
    private double m_kp = 0.1;
    private double m_kd = 0;
    private double m_ki = 0.001;

    private double m_creepBackPower = -0.03;

    private double m_targetHeading; // the target position for the robot
    private double m_frameCount;
    private double m_lastAbsoluteRotationalTargetVelocityDifference;
    private double m_lastHeading; // the last gyro heading of the robot
    private List<Double> m_errorPointsList; // used for calculating the I value

    private NetworkTable m_targettingTable;
    private NetworkTableEntry m_targetAngleEntry;
    private NetworkTableEntry m_targetFoundEntry;
    private NetworkTableEntry m_frameCounterEntry;

    /** Creates a new StartShooter. */
    public Target(Drive drive) {
        m_drive = drive;
        m_timer = new Timer();

        m_errorPointsList = new ArrayList();

        m_targetAngleEntry = m_drive.m_shooterLookUpTable.getTargetAngleEntry();
        m_targetFoundEntry = m_drive.m_shooterLookUpTable.getTargetFoundEntry();
        m_frameCounterEntry = m_drive.m_shooterLookUpTable.getFrameCounterEntry();

        this.addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double currentYaw = m_drive.getGyroYaw();
        m_targetHeading = currentYaw + m_targetAngleEntry.getDouble(0); // may need to flip the sign of the
                                                                        // gyro
        m_frameCount = m_frameCounterEntry.getDouble(0); // gets the inital frame count

        m_lastAbsoluteRotationalTargetVelocityDifference = 0;
        m_lastHeading = currentYaw;

        // updates the PID from shuffleboard
        m_kp = m_drive.m_handlingValues.getTargettingTurnKP();
        m_ki = m_drive.m_handlingValues.getTargettingTurnKI();
        m_kd = m_drive.m_handlingValues.getTargettingTurnKD();

        m_timer.start();
        m_timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double currentYaw = m_drive.getGyroYaw();
        double relativeTargetHeading = m_targetHeading - currentYaw; // gets the relative target heading
                                                                     // based of the last frame aviable
        double timeSinceLastLoop = m_timer.get(); // the time since the robot last went through the excute loop
        m_timer.reset();

        if (m_frameCount != m_frameCounterEntry.getDouble(0)) { // if the frame count has updated
            m_frameCount = m_frameCounterEntry.getDouble(0); // updates the new frame count
            m_targetHeading = currentYaw + m_targetAngleEntry.getDouble(0); // may need to flip the sign of
            // the gyro
            relativeTargetHeading = m_targetAngleEntry.getDouble(0); // the realtive target heading is the target angle
                                                                     // from the robot
        }

        double gyroVelocity = (currentYaw - m_lastHeading) / timeSinceLastLoop;
        double absoluteRotationalVelocityDifference = (getTargetVelocityFromTargetRelativeHeading(relativeTargetHeading)
                - Math.abs(gyroVelocity));
        // the differnce between the absolute value of the target velocity and the
        // absolute value of the robot's velocity

        // calculate rolling sum for the error sum -- avoids accumulation of error from
        // when the relative heading was extreme
        m_errorPointsList.add(absoluteRotationalVelocityDifference);
        if (m_errorPointsList.size() > 6) { // when the data in the rolling average is too great...
            m_errorPointsList.remove(0); // remove the oldest error point
        }
        double errorSum = 0;
        for (int i = 0; i < m_errorPointsList.size(); i++) { // rolls through the error points to calculate sum
            errorSum = errorSum + m_errorPointsList.get(i);
        }

        double pValue = m_kp * absoluteRotationalVelocityDifference;
        double iValue = m_ki * errorSum;
        double dValue = m_kd * (absoluteRotationalVelocityDifference - m_lastAbsoluteRotationalTargetVelocityDifference)
                / timeSinceLastLoop; // the
        // change
        // in robot
        // velocity
        // per time
        double turnValue = pValue + iValue - dValue; // the power to be set to the motors

        System.out.println("pValue: " + pValue + " dValue: " + dValue + " errorSum: " + iValue);
        System.out.println("Realtive Target Heading: " + relativeTargetHeading + " GyroVelocity: " + gyroVelocity);
        System.out.println("Gyro Heading: " + currentYaw + " TargetAngle: " + m_targetAngleEntry.getDouble(0));
        if (relativeTargetHeading > 0) { // may need to flip depending on motor direction
            m_drive.tankDriveRaw(-turnValue - m_creepBackPower, turnValue - m_creepBackPower);
        } else {
            m_drive.tankDriveRaw(turnValue - m_creepBackPower, -turnValue - m_creepBackPower);
        }

        m_lastHeading = currentYaw;
        m_lastAbsoluteRotationalTargetVelocityDifference = absoluteRotationalVelocityDifference;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (!m_targetFoundEntry.getBoolean(false)) {
            System.out.println("Targetting Command: Shuffleboard says no target found !");
            m_drive.tankDriveRaw(0, 0);
            return true;
        }

        if (Math.abs(m_targetAngleEntry.getDouble(0)) < m_deadzoneAngle) {
            System.out.println("Targetting Command: Target lock achieved!");
            m_drive.tankDriveRaw(0, 0);
            return true;
        }

        return false;
    }

    private double getTargetVelocityFromTargetRelativeHeading(double relativeTargetHeading) { // degrees /second
        double fastVelocity = 80;
        double slowVelocity = 12;
        double slowZone = 15;
        // inverting to change direction is taken care of in motor power assignment

        if (Math.abs(relativeTargetHeading) < slowZone) {

            return slowVelocity + (fastVelocity - slowVelocity) * Math.abs(relativeTargetHeading) / slowZone;
        }

        return fastVelocity;
    }
}

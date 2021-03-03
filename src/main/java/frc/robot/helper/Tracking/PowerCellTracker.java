package frc.robot.helper.Tracking;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class PowerCellTracker{
    private NetworkTableEntry m_powerCellAngleEntry;    
    private NetworkTableEntry m_powerCellDistanceEntry;
    private NetworkTableEntry m_frameCounterEntry;
    private NetworkTableEntry m_powerCellFoundEntry;

    private NetworkTable m_trackingTable;

    private double m_powerCellCollectionThreshold = 0.7; // if the powercell distance is less than this begin collection

    public PowerCellTracker()
    {
        m_trackingTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("TrackingContours");
        m_powerCellAngleEntry = m_trackingTable.getEntry("PowerCellAngle");
        m_powerCellDistanceEntry = m_trackingTable.getEntry("PowerCellDistance");
        m_frameCounterEntry = m_trackingTable.getEntry("FrameCounter");
        m_powerCellFoundEntry = m_trackingTable.getEntry("PowerCellFound");
    }

    public double getPowerCellAngle(){
        return m_powerCellAngleEntry.getDouble(0);
    }

    public double getPowerCellDistance(){
        return m_powerCellDistanceEntry.getDouble(0);
    }

    public double getFrameCounter(){
        return m_frameCounterEntry.getDouble(0);
    }

    public boolean getPowerCellFound(){
        return m_powerCellFoundEntry.getBoolean(false);
    }

    public boolean startPowerCellCollectionSequence(){ // if the power cell targettedis ready for collection
        if(getPowerCellDistance() < m_powerCellCollectionThreshold){
            return true;
        }
        return false;
    }
}
package frc.robot.helper.Targetting;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.TheRobot;

public class ShooterLookUp // changes distance to speed and hood angle
{
    private Double m_distanceFactor = 1.0; // allows for easy minor adjustments of shooter if needed (multiplier)
    private NetworkTable m_targettingTable = null;
    private Map<Double, ShooterValueSet> m_valueTable = null; // table that assocaited distance to hood and speed values
    private List<Double> m_keyArray = null;
    private NetworkTableEntry m_frameCounterEntry;
    private NetworkTableEntry m_targetAngleEntry;
    private NetworkTableEntry m_targetFoundEntry;
    private NetworkTableEntry m_targetDistanceEntry;

    public ShooterLookUp() {
        m_targettingTable = NetworkTableInstance.getDefault().getTable("Shuffleboard")
                .getSubTable("TargettingContours");
        // m_targettingTable.getEntry("TargetDistanceTest").setDouble(3.5);

        m_frameCounterEntry = m_targettingTable.getEntry("FrameCounter");
        m_targetAngleEntry = m_targettingTable.getEntry("TargetAngle");
        m_targetFoundEntry = m_targettingTable.getEntry("TargetFound");
        m_targetDistanceEntry = m_targettingTable.getEntry("TargetDistance");

        m_keyArray = new ArrayList<Double>();
        m_valueTable = new HashMap<Double, ShooterValueSet>();

        fillTableVaules();
    }

    private void fillTableVaules() { // adds the key value pairs to the lookup table <KEY, HoodAngle, ShooterRPM>
        addTableValue(1.0, 1.0, 2700.0);
        addTableValue(2.0, 12.0, 2700.0);
        addTableValue(3.0, 25.0, 4100.0);
        addTableValue(4.0, 25.7, 4250.0);
        addTableValue(5.0, 26.25, 4400.0);
        addTableValue(6.0, 27.5, 4500.0);
    }

    public boolean isTargetFound() {
        if (m_targetFoundEntry.getBoolean(false)) {
            TheRobot.getInstance().m_LEDController.setColor(LEDColor.kTargetFound);

            if (Math.abs(getTargetAngle()) < 1) {
                TheRobot.getInstance().m_LEDController.setColor(LEDColor.kOnTarget);
            }

            return true;
        } else {
            TheRobot.getInstance().m_LEDController.setColor(LEDColor.kTargetNotFound);
        }
        return false;
    }

    public NetworkTableEntry getTargetFoundEntry() {
        return m_targetFoundEntry;
    }

    public NetworkTableEntry getTargetAngleEntry() {
        return m_targetAngleEntry;
    }

    public NetworkTableEntry getFrameCounterEntry() {
        return m_frameCounterEntry;
    }

    public double getTargetAngle() {
        return m_targetAngleEntry.getDouble(0);
    }

    private void addTableValue(Double Key, Double HoodAngle, Double ShooterSpeed) // appedns a set of values to the look
                                                                                  // up table system
    {
        m_valueTable.put(Key, new ShooterValueSet(HoodAngle, ShooterSpeed));
        m_keyArray.add(Key);
    }

    public ShooterValueSet getCurrentValues(Boolean Interpolate) // gets the hood and shooter speed values; interpolate
                                                                 // ~ if true finds values between two existing
    {
        // -- remove line below & comment when value table is complete Double
        // distanceFromTarget = ntTble.getEntry("TargetDistance").getDouble(0);
        Double distanceFromTarget = m_targetDistanceEntry.getDouble(3.5);

        if (Interpolate) {
            ShooterValueKey positveValueKey = findClosestPostiveKey(distanceFromTarget);
            ShooterValueKey negativeValueKey = findClosestNegativeKey(distanceFromTarget);

            return interpolate(positveValueKey, negativeValueKey);
        }

        ShooterValueKey valueKey = findClosestKey(distanceFromTarget * m_distanceFactor);
        return m_valueTable.get(valueKey.key); // returns the values that match the key
    }

    public ShooterValueSet getCustomValues(Double Distance, Boolean Interpolate) // interpolate ~ if true finds values
                                                                                 // between two existing
    {
        if (Interpolate) {
            ShooterValueKey positveValueKey = findClosestPostiveKey(Distance);
            ShooterValueKey negativeValueKey = findClosestNegativeKey(Distance);

            return interpolate(positveValueKey, negativeValueKey);
        }

        ShooterValueKey valueKey = findClosestKey(Distance);
        return m_valueTable.get(valueKey.key); // returns the values that match the key
    }

    private ShooterValueKey findClosestKey(Double Distance) {
        ShooterValueKey closestKey = new ShooterValueKey(0.0, -1.0); // the keys that is the closest to the distance

        for (Double key : m_keyArray) // goes through each key and find the differnce between the key and distance
        {
            if (((Math.abs(Distance - key) < closestKey.devationFromDistance)) | closestKey.devationFromDistance < 0) {
                closestKey.devationFromDistance = Math.abs(Distance - key);
                closestKey.key = key;
            }
        }

        return closestKey;
    }

    private ShooterValueKey findClosestPostiveKey(Double Distance) // find the closest key that is greater than distance
    {
        ShooterValueKey closestKey = new ShooterValueKey(0.0, -1.0); // the key that is the closest to the distance

        for (Double key : m_keyArray) // goes through each key and find the differnce between the key and distance
        {
            if ((((Distance - key <= 0) && (Math.abs(Distance - key) < closestKey.devationFromDistance))
                    | closestKey.devationFromDistance < 0)) {
                closestKey.devationFromDistance = Math.abs(Distance - key);
                closestKey.key = key;
            }
        }

        return closestKey;
    }

    private ShooterValueKey findClosestNegativeKey(Double Distance) // key is less than distance
    {
        ShooterValueKey closestKey = new ShooterValueKey(0.0, -1.0); // the key that is the closest but nearer than the
                                                                     // distance

        for (Double key : m_keyArray) // goes through each key and find the differnce between the key and distance
        {
            if ((((Distance - key >= 0) && (Math.abs(Distance - key) < closestKey.devationFromDistance))
                    | closestKey.devationFromDistance < 0)) {
                closestKey.devationFromDistance = Math.abs(Distance - key);
                closestKey.key = key;
            }
        }

        return closestKey;
    }

    private ShooterValueSet interpolate(ShooterValueKey Key1, ShooterValueKey Key2) // creates a new set of values by
                                                                                    // taking weighted average
    {
        Double weight1 = Key2.devationFromDistance / (Key1.devationFromDistance + Key2.devationFromDistance);
        Double weight2 = 1 - weight1; // creates the weights for taking a wieghed average

        ShooterValueSet valueSet1 = m_valueTable.get(Key1.key);
        ShooterValueSet valueSet2 = m_valueTable.get(Key2.key);

        double hoodAngle = takeweightedAverage(valueSet1.hoodAngle, valueSet2.hoodAngle, weight1, weight2);
        double shooterRPM = takeweightedAverage(valueSet1.shooterRPM, valueSet2.shooterRPM, weight1, weight2);

        return new ShooterValueSet(hoodAngle, shooterRPM);
    }

    private Double takeweightedAverage(Double Value1, Double Value2, Double weight1, Double weight2) // takes the
                                                                                                     // weighted average
                                                                                                     // of two values
    {
        return (((Value1 * weight1) + (Value2 * weight2)) / (weight1 + weight2));
    }

    public double getFrameCounter() {
        return m_frameCounterEntry.getDouble(0);
    }
}
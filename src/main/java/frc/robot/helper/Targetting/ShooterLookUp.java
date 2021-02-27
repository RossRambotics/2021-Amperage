package frc.robot.helper.Targetting;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShooterLookUp // changes distance to speed and hood angle
{
    private Double distanceFactor = 1.0; // allows for easy minor adjustments of shooter if needed (multiplier)
    private NetworkTableInstance ntInst = null;
    private NetworkTable ntTble = null;
    private Map<Double, ShooterValueSet> valueTable = null; // table that assocaited distance to hood and speed values
    private List<Double> keyArray = null;

    public ShooterLookUp() {
        ntInst = NetworkTableInstance.getDefault();
        ntTble = ntInst.getTable("ContourTable"); // gets the networktable where the target information is stored
        ntTble.getEntry("TargetDistanceTest").setDouble(3.5);

        keyArray = new ArrayList<Double>();
        valueTable = new HashMap<Double, ShooterValueSet>();

        fillTableVaules();
    }

    private void fillTableVaules() // adds the key value pairs to the lookup table <KEY, HoodAngle, ShooterRPM>
    {

        addTableValue(2.5, 22.0, 4200.0);
        addTableValue(3.6, 30.0, 4200.0);
        addTableValue(5.5, 35.0, 5000.0);
        addTableValue(7.0, 36.0, 5200.0);
        addTableValue(4.2, 33.0, 4400.0);
        addTableValue(4.7, 33.7, 4500.0);
        addTableValue(5.85, 32.5, 4500.0);

    }

    public boolean isTargetFound() {
        boolean isFound = ntTble.getEntry("TargetFound").getBoolean(false);

        return isFound;

    }

    public double getTargetAngle() {
        double d = ntTble.getEntry("TargetAngle").getDouble(0);

        return d;
    }
    // final NetworkTableInstance networkTableInstance =
    // NetworkTableInstance.create();
    // networkTableInstance.startClient("10.32.1.105");
    // System.out.println("Network Tables Connected? " +
    // Boolean.toString(networkTableInstance.isConnected()));

    private void addTableValue(Double Key, Double HoodAngle, Double ShooterSpeed) // appedns a set of values to the look
                                                                                  // up table system
    {
        valueTable.put(Key, new ShooterValueSet(HoodAngle, ShooterSpeed));
        keyArray.add(Key);
    }

    public ShooterValueSet getCurrentValues(Boolean Interpolate) // gets the hood and shooter speed values; interpolate
                                                                 // ~ if true finds values between two existing
    {
        // -- remove line below & comment when value table is complete Double
        // distanceFromTarget = ntTble.getEntry("TargetDistance").getDouble(0);
        Double distanceFromTarget = ntTble.getEntry("TargetDistance").getDouble(3.5);

        if (Interpolate) {
            ShooterValueKey positveValueKey = findClosestPostiveKey(distanceFromTarget);
            ShooterValueKey negativeValueKey = findClosestNegativeKey(distanceFromTarget);

            return interpolate(positveValueKey, negativeValueKey);
        }

        ShooterValueKey valueKey = findClosestKey(distanceFromTarget * distanceFactor);
        return valueTable.get(valueKey.key); // returns the values that match the key
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
        return valueTable.get(valueKey.key); // returns the values that match the key
    }

    private ShooterValueKey findClosestKey(Double Distance) {
        ShooterValueKey closestKey = new ShooterValueKey(0.0, -1.0); // the keys that is the closest to the distance

        for (Double key : keyArray) // goes through each key and find the differnce between the key and distance
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

        for (Double key : keyArray) // goes through each key and find the differnce between the key and distance
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

        for (Double key : keyArray) // goes through each key and find the differnce between the key and distance
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

        ShooterValueSet valueSet1 = valueTable.get(Key1.key);
        ShooterValueSet valueSet2 = valueTable.get(Key2.key);

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
        double d = ntTble.getEntry("FrameCounter").getDouble(0);

        return d;
    }
}
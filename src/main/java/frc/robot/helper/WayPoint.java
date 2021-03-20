package frc.robot.helper;

public class WayPoint {

    // positioning in meters
    private double m_absoluteX;
    private double m_absoluteY;
    private double m_targetRadius = .1;

    public boolean cleared = false; // wether the radian has been cleared

    public WayPoint(double xPosition, double yPosition) {
        m_absoluteX = xPosition;
        m_absoluteY = yPosition;
    }

    public double getDistanceFrom(double x, double y) {
        double distance = Math.sqrt(Math.pow(m_absoluteX - x, 2) + Math.pow(m_absoluteY - y, 2)); // pathagorean therom

        if (distance < m_targetRadius) { // whether or not the target has been cleared
            cleared = true;
        }

        return distance;
    }

    public double getHeadingTo(double x, double y) {

        // acot because y is straight ahead and x is side to side -- x and y are flipped
        double radians = Math.PI / 2;

        if (Math.abs(m_absoluteY - y) > .00001) { // prevents NAN errorz
            radians = -Math.atan((m_absoluteX - x) / (m_absoluteY - y));
        }

        if ((m_absoluteY - y) < 0) { // accounts for the return of arc only covering half the range
            radians = radians + Math.PI;
        }

        if (radians < 0) { // normalizes to 0 to 2pi scale
            radians = radians + 2 * Math.PI;
        }

        return 57.32 * radians; // degrees

    }
}
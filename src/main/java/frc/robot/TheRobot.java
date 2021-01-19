/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import java.text.DecimalFormat;

/**
 * Add your docs here.
 */
public class TheRobot {
    private static DecimalFormat df3 = new DecimalFormat("#.###");
  
    public static Robot m_bot = null;
    public static void SetInstance(Robot bot) {
        m_bot = bot;
    }
    public static Robot getInstance() {
        return m_bot;
    }
    public static void log(String s) {
        System.out.println(s);
    }

    public static String toString(double d) {
        return df3.format(d);
    }
}

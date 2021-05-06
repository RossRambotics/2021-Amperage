package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helper.Targetting.LEDColor;

public class LEDController extends SubsystemBase {
  private double m_Color = 0.0;
  private Spark m_ledController = new Spark(0);
  private int m_flashLength = 10; // in loops
  private int m_flashCounter = 10;
  private boolean m_flashToggle = true;
  private boolean m_runFlash = false;

  /**
   * Creates a new ledController.
   */

  public LEDController() {
    SmartDashboard.putNumber("LEDController/color", m_Color);

  }

  @Override
  public void periodic() {
    // m_Color = SmartDashboard.getNumber("LEDController/color", m_Color);
    if (m_runFlash) {
      runFlash();
    } else {
      m_ledController.set(m_Color);
    }
  }

  private void runFlash() {
    if (m_flashCounter < 0) {
      m_flashCounter = m_flashLength;

      if (m_flashToggle) {
        m_flashToggle = false;
      } else {
        m_flashToggle = true;
      }
    }

    m_flashCounter = m_flashCounter - 1;

    if (m_flashToggle) {
      m_ledController.set(m_Color);
    } else {
      m_ledController.set(0);
    }
  }

  public void disableFlash() {
    m_runFlash = false;
  }

  public void enableFlash() {
    m_runFlash = true;
  }

  public void setColor(LEDColor color) {
    switch (color) {
      case kIndexerFull:
        m_Color = 0.65; // ORANGE
        break;
      case kTargetFound:
        m_Color = 0.69; // YELLOW
        break;
      case kTargetNotFound:
        m_Color = 0.61; // RED
        break;
      case kOnTarget:
        m_Color = 0.77; // GREEN
        break;
      case kSlow:
        m_Color = 0.87; // BLUE
        break;
      case kNormal:
      default:
        m_Color = 0.91; // VIOLET
    }
  }

}

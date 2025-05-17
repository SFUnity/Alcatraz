package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class IntakeConstants {
  // ! Must not be in tuning mode for dashboard switching to work
  public static final LoggedNetworkBoolean groundAlgae =
      new LoggedNetworkBoolean("Ground Algae", false);

  public static final int rollersID = 18;
  public static final int pivotID = 15;

  public static final LoggedTunableNumber kP;

  public static final boolean pivotInverted = true;
  public static final boolean rollersInverted = false;
  public static final double pivotPositionFactor = 2.9;
  public static final double rollersPositionFactor = .2;

  public static final double minAngleRads = Units.degreesToRadians(10);
  public static final double maxAngleRads = Units.degreesToRadians(135);
  public static final double armLengthMeters = Units.inchesToMeters(15);
  public static final double intakePIDMinInput = 0;
  public static final double intakePIDMaxInput = 1 * 360;

  public static final LoggedTunableNumber algaeVoltageThreshold =
      new LoggedTunableNumber("Intake/algaeVoltageThreshold", .5);

  // In rotations
  public static LoggedTunableNumber l1Angle;
  public static LoggedTunableNumber loweredAngle;
  public static LoggedTunableNumber raisedAngle;
  // In volts
  public static LoggedTunableNumber rollersSpeedIn;
  public static LoggedTunableNumber rollersSpeedOut;

  static {
    updateTunables();

    switch (Constants.currentMode) {
      default:
        kP = new LoggedTunableNumber("Intake/kP", 0.028);
        break;
      case SIM:
        kP = new LoggedTunableNumber("Intake/simkP", 0.1);
        break;
    }
  }

  public static void updateTunables() {
    if (groundAlgae.get()) {
      loweredAngle = new LoggedTunableNumber("Intake/loweredAngle", 66);
      raisedAngle = new LoggedTunableNumber("Intake/raisedAngle", 0);
      rollersSpeedIn = new LoggedTunableNumber("Intake/rollerSpeedVoltsIn", 6);
      rollersSpeedOut = new LoggedTunableNumber("Intake/rollerSpeedVoltsOut", 4);
      l1Angle = new LoggedTunableNumber("Intake/L1Angle", 0);
    } else {
      loweredAngle = new LoggedTunableNumber("Intake/loweredAngle", 125);
      raisedAngle = new LoggedTunableNumber("Intake/raisedAngle", 0);
      rollersSpeedIn = new LoggedTunableNumber("Intake/rollerSpeedVoltsIn", 8);
      rollersSpeedOut = new LoggedTunableNumber("Intake/rollerSpeedVoltsOut", 8);
      l1Angle = new LoggedTunableNumber("Intake/L1Angle", 30);
    }
  }
}

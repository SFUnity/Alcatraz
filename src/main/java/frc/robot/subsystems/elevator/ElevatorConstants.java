package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;

public class ElevatorConstants {
  public static final int elevatorMotorID = 16;
  // Profiled PID values
  public static final LoggedTunableNumber kP;
  public static final LoggedTunableNumber kD;
  public static final double elevatorDistanceToleranceInches = 0.3;
  public static final double maxElevatorSpeed = 78.74; // inches/sec
  public static final double maxElevatorAcceleration = 394; // inches/sec^2
  // FF values
  public static final LoggedTunableNumber kG;
  public static final LoggedTunableNumber kV;
  // Sim stuff
  public static final Translation2d elevatorOrigin = new Translation2d(0, 0);
  public static final double carrageMassKg = Units.lbsToKilograms(15);
  public static final double drumRadiusMeters = Units.inchesToMeters(1.4);
  public static final double minHeightInches = 0;
  public static final double maxHeightInches = 23.0;
  public static final double gearRatio = 9;
  public static final double wheelRadius = 1;

  static {
    switch (Constants.currentMode) {
      default:
        kP = new LoggedTunableNumber("Elevator/kP", 0);
        kD = new LoggedTunableNumber("Elevator/kD", 0);
        kG = new LoggedTunableNumber("Elevator/kG", 0);
        kV = new LoggedTunableNumber("Elevator/kV", 0);
        break;
      case SIM:
        kP = new LoggedTunableNumber("Elevator/kP", 0.15);
        kD = new LoggedTunableNumber("Elevator/kD", 0.0);
        kG = new LoggedTunableNumber("Elevator/kG", 1.215);
        kV = new LoggedTunableNumber("Elevator/kV", 0.127);
        break;
    }
  }

  /** In inches */
  public static enum ElevatorHeight {
    L3(new LoggedTunableNumber("Elevator/L3", maxHeightInches)),
    L2(new LoggedTunableNumber("Elevator/L2", 10)),
    L1(new LoggedTunableNumber("Elevator/L1", 0)),
    AlgaeHigh(new LoggedTunableNumber("Elevator/AlgaeHigh", 27)),
    AlgaeLow(new LoggedTunableNumber("Elevator/AlgaeLow", 23)),
    Stow(new LoggedTunableNumber("Elevator/Stow", minHeightInches));

    ElevatorHeight(LoggedTunableNumber height) {
      this.height = height;
    }

    public final LoggedTunableNumber height;

    public double get() {
      return height.get();
    }
  }
}

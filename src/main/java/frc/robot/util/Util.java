package frc.robot.util;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public abstract class Util {
  public static boolean equalsWithTolerance(double a, double b, double tolerance) {
    return (a - tolerance <= b) && (a + tolerance >= b);
  }

  public static void logSubsystem(SubsystemBase s, String sName) {
    sName += "/cmdInfo/";
    Logger.recordOutput(sName + "hasDefault", s.getDefaultCommand() != null);
    Logger.recordOutput(
        sName + "default",
        s.getDefaultCommand() != null ? s.getDefaultCommand().getName() : "none");
    Logger.recordOutput(sName + "hasCommand", s.getCurrentCommand() != null);
    Logger.recordOutput(
        sName + "command",
        s.getCurrentCommand() != null ? s.getCurrentCommand().getName() : "none");
  }

  public static Distance hypot(Distance a, Distance b) {
    return a.unit().of(Math.hypot(a.baseUnitMagnitude(), b.baseUnitMagnitude()));
  }
}

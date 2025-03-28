package frc.robot.subsystems.funnel;

import frc.robot.util.LoggedTunableNumber;

public class FunnelConstants {
  public static final int funnelMotorID = 0;
  public static final boolean inverted = false;
  public static final int positionFactor = 3;

  public static final LoggedTunableNumber rollerSpeedVolts =
      new LoggedTunableNumber("Funnel/rollerSpeed", 2);
}

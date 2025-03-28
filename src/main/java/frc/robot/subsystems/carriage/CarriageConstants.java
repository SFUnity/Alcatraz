package frc.robot.subsystems.carriage;

import frc.robot.util.LoggedTunableNumber;

public class CarriageConstants {
  public static final int rollerMotorID = 10;
  public static final int beamBreakPort = 6;

  public static final boolean inverted = false;
  public static final double positionFactor = 5;

  public static final LoggedTunableNumber algaeVelocityThreshold =
      new LoggedTunableNumber("Carriage/algaeVelocityThreshold", 7000);
  public static final LoggedTunableNumber algaeCurrentThreshold =
      new LoggedTunableNumber("Carriage/algaeCurrentThreshold", 50);

  public static final LoggedTunableNumber intakingSpeedVolts =
      new LoggedTunableNumber("Carriage/intakingSpeedVolts", -5);
  public static final LoggedTunableNumber placeSpeedVolts =
      new LoggedTunableNumber("Carriage/placeSpeedVolts", -8);
  public static final LoggedTunableNumber placeL1SpeedVolts =
      new LoggedTunableNumber("Carriage/placeL1SpeedVolts", -6);
  public static final LoggedTunableNumber highDealgifyingSpeedVolts =
      new LoggedTunableNumber("Carriage/highDealgifyingSpeedVolts", -10);
  public static final LoggedTunableNumber lowDealgifyingSpeedVolts =
      new LoggedTunableNumber("Carriage/lowDealgifyingSpeedVolts", -5);
  public static final LoggedTunableNumber processorSpeedVolts =
      new LoggedTunableNumber("Carriage/processorSpeedVolts", 5);
  public static final LoggedTunableNumber ejectSpeedVolts =
      new LoggedTunableNumber("Carriage/ejectSpeedVolts", 10);
  public static final LoggedTunableNumber holdSpeedVolts =
      new LoggedTunableNumber("Carriage/holdSpeedVolts", -0.5);
  public static final LoggedTunableNumber backwardsIntakeSpeedVolts =
      new LoggedTunableNumber("Carriage/backwardSpeedVolts", 1);
}

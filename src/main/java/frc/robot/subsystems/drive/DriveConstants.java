// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = Units.feetToMeters(17.1);
  public static final double maxAccelerationMetersPerSec =
      Units.feetToMeters(75.0); // This is what 6328
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(20.75);
  public static final double wheelBase = trackWidth;
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final double maxAngularSpeedRadiansPerSec = maxSpeedMetersPerSec / driveBaseRadius;
  public static final double maxAngularAccelerationRadiansPerSec =
      maxAccelerationMetersPerSec / driveBaseRadius;
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };
  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(moduleTranslations);

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(-2.186);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(-2.140);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(1.993);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(-0.031);

  // Motor/encoder inverted values for each module
  public static final boolean frontLeftDriveInverted = false;
  public static final boolean frontRightDriveInverted = true;
  public static final boolean backLeftDriveInverted = false;
  public static final boolean backRightDriveInverted = true;

  public static final boolean frontLeftTurnInverted = true;
  public static final boolean frontRightTurnInverted = true;
  public static final boolean backLeftTurnInverted = true;
  public static final boolean backRightTurnInverted = true;

  public static final boolean frontLeftTurnEncoderInverted = false;
  public static final boolean frontRightTurnEncoderInverted = false;
  public static final boolean backLeftTurnEncoderInverted = false;
  public static final boolean backRightTurnEncoderInverted = false;

  // Device CAN IDs. Based off power port on PDH
  public static final int pigeonCanId = 20;

  public static final int frontLeftDriveCanId = 15;
  public static final int backLeftDriveCanId = 0;
  public static final int frontRightDriveCanId = 10;
  public static final int backRightDriveCanId = 9;

  public static final int frontLeftTurnCanId = 19;
  public static final int backLeftTurnCanId = 2;
  public static final int frontRightTurnCanId = 11;
  public static final int backRightTurnCanId = 1;

  public static final int frontLeftTurnEncoderCanId = 18;
  public static final int backLeftTurnEncoderCanId = 4;
  public static final int frontRightTurnEncoderCanId = 19; // on same power as 18
  public static final int backRightTurnEncoderCanId = 5; // on same power as 4

  public static final String CANBusName = "rio";

  // Drive motor configuration
  public static final int driveMotorSupplyCurrentLimit = 50;
  public static final int driveMotorStatorCurrentLimit = 80;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2);
  public static final double driveMotorReduction = 6.12;
  public static final DCMotor driveGearbox = DCMotor.getKrakenX60(1);

  // Drive PID configuration
  public static final LoggedTunableNumber driveKp;
  public static final LoggedTunableNumber driveKd;
  public static final double driveKs;
  public static final double driveKv;

  static {
    switch (Constants.currentMode) {
      default:
        driveKp = new LoggedTunableNumber("Drive/ModuleTunables/driveKp", 0.0);
        driveKd = new LoggedTunableNumber("Drive/ModuleTunables/driveKd", 0.0);
        driveKs = 0.0;
        driveKv = 0.0;
        break;
      case SIM:
        driveKp = new LoggedTunableNumber("Drive/SimModuleTunables/driveKp", 0.29);
        driveKd = new LoggedTunableNumber("Drive/SimModuleTunables/driveKd", 0.0);
        driveKs = 0.0;
        driveKv = 0.0;
        break;
    }
  }

  // Turn motor configuration
  public static final int turnMotorCurrentLimit = 60;
  public static final double turnMotorReduction = 150 / 7;
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final double turnEncoderPositionFactor =
      2 * Math.PI / turnMotorReduction; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / turnMotorReduction; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final LoggedTunableNumber turnKp;
  public static final LoggedTunableNumber turnKd;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  static {
    switch (Constants.currentMode) {
      default:
        turnKp = new LoggedTunableNumber("Drive/ModuleTunables/turnKp", 0.0);
        turnKd = new LoggedTunableNumber("Drive/ModuleTunables/turnKd", 0.0);
        break;
      case SIM:
        turnKp = new LoggedTunableNumber("Drive/SimModuleTunables/turnKp", 14.0);
        turnKd = new LoggedTunableNumber("Drive/SimModuleTunables/turnKd", 0.0);
        break;
    }
  }

  /**
   * Drive Command Config
   *
   * @param xJoystick - Left Joystick X axis
   * @param yJoystick - Left Joystick Y axis
   * @param omegaJoystick - Right Joystick X axis
   * @param slowMode - If the joystick drive should be slowed down
   * @param slowDriveMultiplier - Multiplier for slow mode
   * @param slowTurnMultiplier - Multiplier for slow mode
   * @param povUp - POV/Dpad Up
   * @param povDown - POV/Dpad Down
   * @param povLeft - POV/Dpad Left
   * @param povRight - POV/Dpad Right
   */
  public static final record DriveCommandsConfig(
      CommandXboxController controller,
      BooleanSupplier slowMode,
      LoggedTunableNumber slowDriveMultiplier,
      LoggedTunableNumber slowTurnMultiplier) {

    private static final boolean simMode = Constants.currentMode == Constants.Mode.SIM;

    public double getXInput() {
      return simMode ? -controller.getLeftX() : controller.getLeftY();
    }

    public double getYInput() {
      return simMode ? controller.getLeftY() : controller.getLeftX();
    }

    public double getOmegaInput() {
      return -controller.getRightX();
    }

    public boolean povUpPressed() {
      return controller.povUp().getAsBoolean();
    }

    public boolean povDownPressed() {
      return controller.povDown().getAsBoolean();
    }

    public boolean povLeftPressed() {
      return controller.povLeft().getAsBoolean();
    }

    public boolean povRightPressed() {
      return controller.povRight().getAsBoolean();
    }
  }
}

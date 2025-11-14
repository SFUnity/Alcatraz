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

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Autos;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = Units.feetToMeters(10.5);
  public static final double nitroMaxSpeedMetersPerSec = Units.feetToMeters(17.5);
  public static final double maxAccelerationMetersPerSec =
      Units.feetToMeters(70.0); // This is what 6328
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
  public static final double frontLeftZeroRotation = -0.241943;
  public static final double frontRightZeroRotation = 0.042969;
  public static final double backLeftZeroRotation = -0.027;
  public static final double backRightZeroRotation = 0.484375;

  // Motor/encoder inverted values for each module
  public static final boolean frontLeftDriveInverted = true;
  public static final boolean frontRightDriveInverted = false;
  public static final boolean backLeftDriveInverted = false;
  public static final boolean backRightDriveInverted = false;

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

  public static final int frontLeftDriveCanId = 3;
  public static final int frontRightDriveCanId = 4; // It's in PDH port 1
  public static final int backLeftDriveCanId = 19;
  public static final int backRightDriveCanId = 13;

  public static final int frontLeftTurnCanId = 8;
  public static final int frontRightTurnCanId = 6;
  public static final int backLeftTurnCanId = 13;
  public static final int backRightTurnCanId = 17;

  public static final int frontLeftTurnEncoderCanId = 19;
  public static final int frontRightTurnEncoderCanId = 4;
  public static final int backLeftTurnEncoderCanId = 18;
  public static final int backRightTurnEncoderCanId = 5;

  public static final String CANBusName = "rio";

  // Drive motor configuration
  public static final int driveMotorSupplyCurrentLimit = 50;
  public static final int driveMotorStatorCurrentLimit = 80;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2.193);
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
        driveKp = new LoggedTunableNumber("Drive/ModuleTunables/driveKp", 0.3);
        driveKd = new LoggedTunableNumber("Drive/ModuleTunables/driveKd", 0.0);
        driveKs = 0.18753;
        driveKv = 0.75276;
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
  public static final double turnMotorReduction = 150.0 / 7.0;
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn PID configuration
  public static final LoggedTunableNumber turnKp;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  static {
    switch (Constants.currentMode) {
      default:
        turnKp = new LoggedTunableNumber("Drive/ModuleTunables/turnKp", 0.5);
        break;
      case SIM:
        turnKp = new LoggedTunableNumber("Drive/SimModuleTunables/turnKp", 14.0);
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

    // private static final boolean simMode = Constants.currentMode == Constants.Mode.SIM;
    private static final boolean simMode = false;

    public double getXInput() {
      return simMode ? -controller.getLeftX() : -controller.getLeftY();
      // return controller.getLeftX();
    }

    public double getYInput() {
      return simMode ? controller.getLeftY() : -controller.getLeftX();
      // return -controller.getLeftY();
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
      return DriverStation.isAutonomousEnabled()
          ? Autos.moveLeft
          : controller.povLeft().getAsBoolean();
    }

    public boolean povRightPressed() {
      return DriverStation.isAutonomousEnabled()
          ? Autos.moveRight
          : controller.povRight().getAsBoolean();
    }

    public boolean finishScoring() {
      return controller.leftTrigger().getAsBoolean();
    }
  }
}

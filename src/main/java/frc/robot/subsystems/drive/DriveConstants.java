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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constantsGlobal.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Util;
import java.util.function.BooleanSupplier;

public class DriveConstants {
  public static final LinearVelocity maxSpeed = MetersPerSecond.of(4.8);
  public static final LinearAcceleration maxAcceleration =
      FeetPerSecondPerSecond.of(75.0); // This is what 6328
  public static final double odometryFrequency = 100.0; // Hz
  public static final Distance trackWidth = Inches.of(26.5);
  public static final Distance wheelBase = Inches.of(26.5);
  public static final Distance driveBaseRadius =
      Util.hypot(trackWidth.div(2.0), wheelBase.div(2.0));
  public static final AngularVelocity maxAngularSpeed =
      RadiansPerSecond.of(maxSpeed.baseUnitMagnitude() / driveBaseRadius.baseUnitMagnitude());
  public static final AngularAcceleration maxAngularAcceleration =
      RadiansPerSecondPerSecond.of(
          maxAcceleration.baseUnitMagnitude() / driveBaseRadius.baseUnitMagnitude());
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth.div(2.0), wheelBase.div(2.0)),
        new Translation2d(trackWidth.div(2.0), wheelBase.div(-2.0)),
        new Translation2d(trackWidth.div(-2.0), wheelBase.div(2.0)),
        new Translation2d(trackWidth.div(-2.0), wheelBase.div(-2.0))
      };
  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(moduleTranslations);

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

  // Motor/encoder inverted values for each module
  public static final boolean frontLeftDriveInverted = false;
  public static final boolean frontRightDriveInverted = false;
  public static final boolean backLeftDriveInverted = false;
  public static final boolean backRightDriveInverted = false;

  public static final boolean frontLeftTurnInverted = false;
  public static final boolean frontRightTurnInverted = false;
  public static final boolean backLeftTurnInverted = false;
  public static final boolean backRightTurnInverted = false;

  public static final boolean frontLeftTurnEncoderInverted = true;
  public static final boolean frontRightTurnEncoderInverted = true;
  public static final boolean backLeftTurnEncoderInverted = true;
  public static final boolean backRightTurnEncoderInverted = true;

  // Device CAN IDs
  public static final int pigeonCanId = 9;

  public static final int frontLeftDriveCanId = 1;
  public static final int backLeftDriveCanId = 3;
  public static final int frontRightDriveCanId = 5;
  public static final int backRightDriveCanId = 7;

  public static final int frontLeftTurnCanId = 2;
  public static final int backLeftTurnCanId = 4;
  public static final int frontRightTurnCanId = 6;
  public static final int backRightTurnCanId = 8;

  public static final int frontLeftTurnEncoderCanId = 2;
  public static final int backLeftTurnEncoderCanId = 4;
  public static final int frontRightTurnEncoderCanId = 6;
  public static final int backRightTurnEncoderCanId = 8;

  public static final String CANBusName = "rio";

  // Drive motor configuration
  public static final int driveMotorSupplyCurrentLimit = 50;
  public static final int driveMotorStatorCurrentLimit = 80;
  public static final Distance wheelRadius = Inches.of(1.5);
  public static final double driveMotorReduction =
      (45.0 * 22.0) / (14.0 * 15.0); // MAXSwerve with 14 pinion teeth and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getKrakenX60(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
  public static final LoggedTunableNumber driveKp;
  public static final LoggedTunableNumber driveKd;
  public static final LoggedTunableNumber driveKs;
  public static final LoggedTunableNumber driveKv;

  static {
    switch (Constants.currentMode) {
      default:
        driveKp = new LoggedTunableNumber("Drive/ModuleTunables/driveKp", 0.0);
        driveKd = new LoggedTunableNumber("Drive/ModuleTunables/driveKd", 0.0);
        driveKs = new LoggedTunableNumber("Drive/ModuleTunables/driveKs", 0.0);
        driveKv = new LoggedTunableNumber("Drive/ModuleTunables/driveKv", 0.1);
        break;
      case SIM:
        driveKp = new LoggedTunableNumber("Drive/SimModuleTunables/driveKp", 0.05);
        driveKd = new LoggedTunableNumber("Drive/SimModuleTunables/driveKd", 0.0);
        driveKs = new LoggedTunableNumber("Drive/SimModuleTunables/driveKs", 0.0);
        driveKv = new LoggedTunableNumber("Drive/SimModuleTunables/driveKv", 0.0789);
        break;
    }
  }

  // Turn motor configuration
  public static final int turnMotorCurrentLimit = 60;
  public static final double turnMotorReduction = 9424.0 / 203.0;
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
        turnKp = new LoggedTunableNumber("Drive/ModuleTunables/turnkP", 2.0);
        turnKd = new LoggedTunableNumber("Drive/ModuleTunables/turnkD", 0.0);
        break;
      case SIM:
        turnKp = new LoggedTunableNumber("Drive/SimModuleTunables/turnkP", 8.0);
        turnKd = new LoggedTunableNumber("Drive/SimModuleTunables/turnkD", 0.0);
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

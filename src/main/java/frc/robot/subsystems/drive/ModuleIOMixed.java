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

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.util.PhoenixUtil.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static frc.robot.util.SparkUtil.*;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder. Configured using a set of module constants from Phoenix.
 *
 * <p>Device configuration and other behaviors not exposed by TunerConstants can be customized here.
 */
public class ModuleIOMixed implements ModuleIO {
  // Module specific constants
  private final boolean driveInverted;
  private final boolean turnInverted;
  private final boolean turnEncoderInverted;
  private final SparkMaxConfig turnConfig;

  // Hardware objects
  private final TalonFX driveTalon;
  private final SparkMax turnSpark;
  private final CANcoder cancoder;
  private final RelativeEncoder turnEncoder;

  // Voltage control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final SparkClosedLoopController turnController;
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Timestamp inputs from Phoenix thread
  private final Queue<Double> timestampQueue;

  // Inputs from drive motor
  private final StatusSignal<Angle> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;

  // Inputs from turn motor and CANcoder
  private final StatusSignal<Angle> turnAbsolutePosition;
  private final Queue<Double> turnPositionQueue;

  // Connection debouncers
  private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnConnectedDebounce = new Debouncer(0.5);
  private final Debouncer turnEncoderConnectedDebounce = new Debouncer(0.5);

  // Motor Configurators
  private final TalonFXConfiguration driveConfig;

  public ModuleIOMixed(int module) {
    driveInverted =
        switch (module) {
          case 0 -> frontLeftDriveInverted;
          case 1 -> frontRightDriveInverted;
          case 2 -> backLeftDriveInverted;
          case 3 -> backRightDriveInverted;
          default -> false;
        };
    turnInverted =
        switch (module) {
          case 0 -> frontLeftTurnInverted;
          case 1 -> frontRightTurnInverted;
          case 2 -> backLeftTurnInverted;
          case 3 -> backRightTurnInverted;
          default -> false;
        };
    turnEncoderInverted =
        switch (module) {
          case 0 -> frontLeftTurnEncoderInverted;
          case 1 -> frontRightTurnEncoderInverted;
          case 2 -> backLeftTurnEncoderInverted;
          case 3 -> backRightTurnEncoderInverted;
          default -> false;
        };
    driveTalon =
        new TalonFX(
            switch (module) {
              case 0 -> frontLeftDriveCanId;
              case 1 -> frontRightDriveCanId;
              case 2 -> backLeftDriveCanId;
              case 3 -> backRightDriveCanId;
              default -> 0;
            },
            DriveConstants.CANBusName);
    turnSpark =
        new SparkMax(
            switch (module) {
              case 0 -> frontLeftTurnCanId;
              case 1 -> frontRightTurnCanId;
              case 2 -> backLeftTurnCanId;
              case 3 -> backRightTurnCanId;
              default -> 0;
            },
            MotorType.kBrushless);
    cancoder =
        new CANcoder(
            switch (module) {
              case 0 -> frontLeftTurnEncoderCanId;
              case 1 -> frontRightTurnEncoderCanId;
              case 2 -> backLeftTurnEncoderCanId;
              case 3 -> backRightTurnEncoderCanId;
              default -> 0;
            },
            DriveConstants.CANBusName);
    turnEncoder = turnSpark.getEncoder();
    turnController = turnSpark.getClosedLoopController();

    // Configure drive motor
    driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    driveConfig.Slot0 =
        new Slot0Configs()
            .withKP(driveKp.get())
            .withKD(driveKd.get())
            .withKS(driveKs)
            .withKV(driveKv);
    driveConfig.Feedback.SensorToMechanismRatio = driveMotorReduction;
    driveConfig.CurrentLimits.SupplyCurrentLimit = driveMotorSupplyCurrentLimit;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = driveMotorStatorCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        driveInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
    tryUntilOk(5, () -> driveTalon.setPosition(0.0, 0.25));

    // Configure turn motor
    turnConfig = sparkConfig(turnInverted, (2 * Math.PI) / turnMotorReduction);
    turnConfig.encoder.velocityConversionFactor((2 * Math.PI) / 60.0 / turnMotorReduction);
    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(turnPIDMinInput, turnPIDMaxInput)
        .pidf(turnKp.get(), 0.0, 0, 0.0);
    turnConfig.signals.primaryEncoderPositionPeriodMs((int) (1000.0 / odometryFrequency));
    turnConfig.idleMode(IdleMode.kCoast);
    configureSpark(turnSpark, turnConfig, true);

    // Configure CANCoder
    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset =
        switch (module) {
          case 0 -> frontLeftZeroRotation;
          case 1 -> frontRightZeroRotation;
          case 2 -> backLeftZeroRotation;
          case 3 -> backRightZeroRotation;
          default -> 0;
        };
    cancoderConfig.MagnetSensor.SensorDirection =
        turnEncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    cancoder.getConfigurator().apply(cancoderConfig);
    tryUntilOk(
        turnSpark, () -> turnEncoder.setPosition(cancoder.getPosition().getValue().in(Radians)));

    // Create timestamp queue
    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    // Create drive status signals
    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    // Create turn status signals
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnEncoder::getPosition);

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(odometryFrequency, drivePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, driveVelocity, driveAppliedVolts, driveCurrent, turnAbsolutePosition);
    ParentDevice.optimizeBusUtilizationForAll(driveTalon, cancoder);

    // logSparkMax("Drive/Module" + Integer.toString(module) + "TurnConfig", turnSpark);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    // Refresh all signals
    var driveStatus =
        BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
    var turnEncoderStatus = BaseStatusSignal.refreshAll(turnAbsolutePosition);

    // Update drive inputs
    inputs.driveConnected = driveConnectedDebounce.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();

    // Update turn inputs
    sparkStickyFault = false;
    ifOk(
        turnSpark,
        turnEncoder::getPosition,
        (value) -> inputs.turnPosition = new Rotation2d(value));
    ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
    ifOk(
        turnSpark,
        new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
        (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
    ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
    inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);
    inputs.turnEncoderConnected = turnEncoderConnectedDebounce.calculate(turnEncoderStatus.isOK());
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());

    // Update odometry inputs
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> new Rotation2d(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveTalon.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnSpark.setVoltage(output);
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    driveTalon.setControl(velocityVoltageRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    double setpoint =
        MathUtil.inputModulus(rotation.getRadians(), turnPIDMinInput, turnPIDMaxInput);
    turnController.setReference(setpoint, ControlType.kPosition);
  }

  @Override
  public void setDrivePIDF(double drivekP, double drivekD) {
    driveConfig.Slot0.kP = drivekP;
    driveConfig.Slot0.kD = drivekD;
    tryUntilOk(5, () -> driveTalon.getConfigurator().apply(driveConfig, 0.25));
  }

  @Override
  public void setTurnPIDF(double turnkP) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pidf(turnkP, 0, 0, 0);
    configureSpark(turnSpark, config, false);
  }

  @Override
  public void setDriveBrakeMode(boolean brake) {
    driveTalon.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setTurnBrakeMode(boolean brake) {
    turnConfig.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    configureSpark(turnSpark, turnConfig, false);
  }
}

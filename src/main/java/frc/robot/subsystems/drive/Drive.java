// Copyright 2021-2024 FRC 6328
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

import choreo.util.AllianceFlipUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.DriveConstants.DriveCommandsConfig;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.Alert;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseManager;
import frc.robot.util.Util;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** For controlling and reading data from the physical drive */
public class Drive extends SubsystemBase {
  // Commands stuff
  private static final double DEADBAND = 0.05;
  private DriveCommandsConfig config;

  private static final LoggedTunableNumber linearkP =
      new LoggedTunableNumber("Drive/Commands/Linear/kP", 3.5);
  private static final LoggedTunableNumber linearkD =
      new LoggedTunableNumber("Drive/Commands/Linear/kD", 0.0);
  private static final LoggedTunableNumber thetakP =
      new LoggedTunableNumber("Drive/Commands/Theta/kP", 6.0);
  private static final LoggedTunableNumber thetakD =
      new LoggedTunableNumber("Drive/Commands/Theta/D", 0.0);
  private static final LoggedTunableNumber linearTolerance =
      new LoggedTunableNumber("Drive/Commands/Linear/tolerance", 0.08);
  private static final LoggedTunableNumber thetaToleranceDeg =
      new LoggedTunableNumber("Drive/Commands/Theta/toleranceDeg", 1.0);

  private static final LoggedTunableNumber maxLinearVelocity =
      new LoggedTunableNumber(
          "Drive/Commands/Linear/maxVelocity", DriveConstants.MAX_LINEAR_VELOCITY);
  private static final LoggedTunableNumber maxLinearAcceleration =
      new LoggedTunableNumber(
          "Drive/Commands/Linear/maxAcceleration", DriveConstants.MAX_LINEAR_ACCELERATION * 0.4);
  private static final LoggedTunableNumber maxAngularVelocity =
      new LoggedTunableNumber(
          "Drive/Commands/Theta/maxVelocity", DriveConstants.MAX_ANGULAR_VELOCITY * 0.8);
  private static final LoggedTunableNumber maxAngularAcceleration =
      new LoggedTunableNumber(
          "Drive/Commands/Theta/maxAcceleration", DriveConstants.MAX_ANGULAR_ACCELERATION * 0.8);

  private final ProfiledPIDController thetaController;
  private final ProfiledPIDController linearController;

  // Subsystem stuff
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final PoseManager poseManager;
  private final SysIdRoutine sysId;

  private final SwerveDriveKinematics kinematics = DriveConstants.kinematics;
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private boolean brakeMode;
  private Timer brakeModeTimer = new Timer();
  private static final double BREAK_MODE_DELAY_SEC = 10.0;

  // Alerts
  private final Alert gyroDisconnected = new Alert("Gyro disconnected!", Alert.AlertType.WARNING);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO,
      PoseManager poseManager,
      DriveCommandsConfig config) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
    this.poseManager = poseManager;
    this.config = config;

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));

    // Configure controllers for commands
    linearController =
        new ProfiledPIDController(
            linearkP.get(), 0, linearkD.get(), new TrapezoidProfile.Constraints(0, 0));
    linearController.setTolerance(linearTolerance.get());

    thetaController =
        new ProfiledPIDController(
            thetakP.get(), 0, thetakD.get(), new TrapezoidProfile.Constraints(0.0, 0.0));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(thetaToleranceDeg.get()));

    updateConstraints();
    updateModuleTunables();
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    updateModuleTunables();

    // Set alerts
    gyroDisconnected.set(!gyroInputs.connected);

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Read wheel positions and deltas from each module
    SwerveModulePosition[] modulePositions = getModulePositions();
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
    for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
      moduleDeltas[moduleIndex] =
          new SwerveModulePosition(
              modulePositions[moduleIndex].distanceMeters
                  - lastModulePositions[moduleIndex].distanceMeters,
              modulePositions[moduleIndex].angle);
      lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
    }

    // Update gyro angle
    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics.toTwist2d(moduleDeltas);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
    }

    // Apply odometry update
    poseManager.addOdometryMeasurement(rawGyroRotation, modulePositions);

    // Update current velocities use gyro when possible
    ChassisSpeeds robotRelativeVelocity = getSpeeds();
    robotRelativeVelocity.omegaRadiansPerSecond =
        gyroInputs.connected
            ? gyroInputs.yawVelocityRadPerSec
            : robotRelativeVelocity.omegaRadiansPerSecond;
    poseManager.addVelocityData(GeomUtil.toTwist2d(robotRelativeVelocity));

    // update the brake mode based on the robot's velocity and state (enabled/disabled)
    updateBrakeMode();

    Util.logSubsystem(this, "Drive");
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    Logger.recordOutput("Drive/TargetSpeeds", speeds);

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    setModuleSetpoints(setpointStates);
  }

  private void setModuleSetpoints(SwerveModuleState[] setpointStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_LINEAR_VELOCITY);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  private void setAllModuleSetpointsToSame(double speed, Rotation2d angle) {
    var moduleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      moduleStates[i] = new SwerveModuleState(speed, angle);
    }
    setModuleSetpoints(moduleStates);
  }

  /** Stops the drive. */
  public void stop() {
    var moduleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      moduleStates[i] = new SwerveModuleState(0, getModuleStates()[i].angle);
    }
    setModuleSetpoints(moduleStates);
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = DriveConstants.moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /**
   * If the robot is enabled and brake mode is not enabled, enable it. If the robot is disabled, has
   * stopped moving for the specified period of time, and brake mode is enabled, disable it.
   */
  private void updateBrakeMode() {
    if (DriverStation.isEnabled() && !brakeMode) {
      brakeMode = true;
      setBrakeMode(true);
      brakeModeTimer.restart();
    } else if (DriverStation.isDisabled()) {
      boolean stillMoving = false;
      double velocityLimit = 0.05; // In meters per second
      ChassisSpeeds measuredChassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
      if (Math.abs(measuredChassisSpeeds.vxMetersPerSecond) > velocityLimit
          || Math.abs(measuredChassisSpeeds.vyMetersPerSecond) > velocityLimit) {
        stillMoving = true;
        brakeModeTimer.restart();
      }

      if (brakeMode && !stillMoving && brakeModeTimer.hasElapsed(BREAK_MODE_DELAY_SEC)) {
        brakeMode = false;
        setBrakeMode(false);
      }
    }
  }

  private void setBrakeMode(boolean enable) {
    for (var module : modules) {
      module.setBrakeMode(enable);
    }
  }

  /** Returns the measured speeds of the robot in the robot's frame of reference. */
  @AutoLogOutput(key = "Drive/MeasuredSpeeds")
  private ChassisSpeeds getSpeeds() {
    return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
  }

  // Drive Commands

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public Command joystickDrive() {
    return run(() -> {
          // Convert to doubles
          double o = config.getOmegaInput();

          // Check for slow mode
          if (config.slowMode().getAsBoolean()) {
            o *= config.slowTurnMultiplier().get();
          }

          // Apply deadband
          double omega = MathUtil.applyDeadband(o, DEADBAND);

          // Square values and scale to max velocity
          omega = Math.copySign(omega * omega, omega);
          omega *= DriveConstants.MAX_ANGULAR_VELOCITY;

          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks();

          // Convert to field relative speeds & send command
          runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX(),
                  linearVelocity.getY(),
                  omega,
                  AllianceFlipUtil.shouldFlip()
                      ? poseManager.getRotation().plus(new Rotation2d(Math.PI))
                      : poseManager.getRotation()));
        })
        .withName("Joystick Drive");
  }

  /**
   * Field relative drive command using one joystick (controlling linear velocity) with a
   * ProfiledPID for angular velocity.
   */
  public Command headingDrive(Supplier<Rotation2d> goalHeading) {
    return run(() -> {
          updateThetaTunables();
          updateThetaConstraints();

          // Get linear velocity
          Translation2d linearVelocity = getLinearVelocityFromJoysticks();

          // Convert to field relative speeds & send command
          runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX(),
                  linearVelocity.getY(),
                  getAngularVelocityFromProfiledPID(goalHeading.get().getRadians()),
                  AllianceFlipUtil.shouldFlip()
                      ? poseManager.getRotation().plus(new Rotation2d(Math.PI))
                      : poseManager.getRotation()));

          Leds.getInstance().alignedWithTarget = thetaAtGoal();
        })
        .beforeStarting(
            () -> {
              resetThetaController();
            })
        .finallyDo(
            () -> {
              stop();
              Leds.getInstance().alignedWithTarget = false;
            })
        .withName("Heading Drive");
  }

  /**
   * Field relative drive command using a ProfiledPID for linear velocity and a ProfiledPID for
   * angular velocity.
   */
  public Command fullAutoDrive(Supplier<Pose2d> goalPose) {
    return run(() -> {
          updateTunables();
          updateConstraints();

          // Calculate linear speed
          Pose2d targetPose = goalPose.get();

          double currentDistance = poseManager.getDistanceTo(targetPose);

          double driveVelocityScalar = linearController.calculate(currentDistance, 0.0);

          if (linearAtGoal()) driveVelocityScalar = 0.0;

          // Calculate angle to target then transform by velocity scalar
          Rotation2d angleToTarget = poseManager.getHorizontalAngleTo(targetPose);

          Translation2d driveVelocity = new Translation2d(driveVelocityScalar, angleToTarget);

          // Calculate theta speed
          double thetaVelocity =
              getAngularVelocityFromProfiledPID(targetPose.getRotation().getRadians());
          if (thetaController.atGoal()) thetaVelocity = 0.0;

          // Send command
          runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  driveVelocity.getX(),
                  driveVelocity.getY(),
                  thetaVelocity,
                  poseManager.getRotation()));

          Leds.getInstance().alignedWithTarget = linearAtGoal() && thetaAtGoal();

          Logger.recordOutput("Drive/Commands/Linear/currentDistance", currentDistance);
        })
        .beforeStarting(
            () -> {
              resetControllers(goalPose.get());
            })
        .finallyDo(
            () -> {
              stop();
              Leds.getInstance().alignedWithTarget = false;
            })
        .withName("Full Auto Drive");
  }

  private Translation2d getLinearVelocityFromJoysticks() {
    // Convert to doubles
    double x = config.getXInput();
    double y = config.getYInput();

    // The speed value here might need to change
    double povMovementSpeed = 0.5;
    if (config.povDownPressed()) {
      x = povMovementSpeed;
    } else if (config.povUpPressed()) {
      x = -povMovementSpeed;
    } else if (config.povLeftPressed()) {
      y = -povMovementSpeed;
    } else if (config.povRightPressed()) {
      y = povMovementSpeed;
    }

    // Check for slow mode
    if (config.slowMode().getAsBoolean()) {
      double multiplier = config.slowDriveMultiplier().get();
      x *= multiplier;
      y *= multiplier;
    }

    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(x, y);

    // Square values and scale to max velocity
    linearMagnitude = linearMagnitude * linearMagnitude;
    linearMagnitude *= DriveConstants.MAX_LINEAR_VELOCITY;

    // Calcaulate new linear velocity
    Translation2d linearVelocity = new Translation2d(linearMagnitude, linearDirection);

    return linearVelocity;
  }

  private double getAngularVelocityFromProfiledPID(double goalHeadingRads) {
    double output =
        thetaController.calculate(
            poseManager.getPose().getRotation().getRadians(), goalHeadingRads);

    Logger.recordOutput("Drive/Commands/Theta/HeadingError", thetaController.getPositionError());
    return output;
  }

  private void updateTunables() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> linearController.setPID(linearkP.get(), 0, linearkD.get()),
        linearkP,
        linearkD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> linearController.setTolerance(linearTolerance.get()), linearTolerance);

    updateThetaTunables();
  }

  private void updateThetaTunables() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> thetaController.setPID(thetakP.get(), 0, thetakD.get()),
        thetakP,
        thetakD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> thetaController.setTolerance(Units.degreesToRadians(thetaToleranceDeg.get())),
        thetaToleranceDeg);
  }

  private void updateConstraints() {
    linearController.setConstraints(
        new TrapezoidProfile.Constraints(maxLinearVelocity.get(), maxLinearAcceleration.get()));
    updateThetaConstraints();
  }

  private void updateThetaConstraints() {
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(maxAngularVelocity.get(), maxAngularAcceleration.get()));
  }

  private void resetControllers(Pose2d goalPose) {
    Twist2d fieldVelocity = poseManager.fieldVelocity();
    double linearVelocity =
        Math.min(
            0.0,
            new Translation2d(fieldVelocity.dx, fieldVelocity.dy)
                .rotateBy(poseManager.getHorizontalAngleTo(goalPose))
                .getX());
    linearController.reset(poseManager.getDistanceTo(goalPose), linearVelocity);
    resetThetaController();
  }

  private void resetThetaController() {
    Pose2d currentPose = poseManager.getPose();
    Twist2d fieldVelocity = poseManager.fieldVelocity();
    thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.dtheta);
  }

  /** Returns true if within tolerance of aiming at goal */
  @AutoLogOutput(key = "Drive/Commands/Linear/AtGoal")
  public boolean linearAtGoal() {
    return linearController.atGoal();
  }

  /** Returns true if within tolerance of aiming at speaker */
  @AutoLogOutput(key = "Drive/Commands/Theta/AtGoal")
  public boolean thetaAtGoal() {
    return Util.equalsWithTolerance(
        thetaController.getSetpoint().position,
        thetaController.getGoal().position,
        Units.degreesToRadians(thetaToleranceDeg.get()));
  }

  // Module Stuff
  private static final LoggedTunableNumber tuningDriveSpeed =
      new LoggedTunableNumber("Drive/ModuleTunables/driveSpeedForTuning", 1);
  private static final LoggedTunableNumber tuningTurnDelta =
      new LoggedTunableNumber("Drive/ModuleTunables/turnDeltaForTuning", 90);

  private void updateModuleTunables() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            DriveConstants.driveFeedback.setPID(
                DriveConstants.drivekP.get(), 0, DriveConstants.drivekD.get()),
        DriveConstants.drivekP,
        DriveConstants.drivekD);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            DriveConstants.turnFeedback.setPID(
                DriveConstants.turnkP.get(), 0, DriveConstants.turnkD.get()),
        DriveConstants.turnkP,
        DriveConstants.turnkD);
  }

  // Tuning Commands
  public Command tuneModuleDrive() {
    return tuningCmdTemplate(
            () -> setAllModuleSetpointsToSame(tuningDriveSpeed.get(), new Rotation2d()),
            () -> setAllModuleSetpointsToSame(-tuningDriveSpeed.get(), new Rotation2d()))
        .withName("tuneModuleDrive");
  }

  public Command tuneModuleTurn() {
    return tuningCmdTemplate(
            () -> setAllModuleSetpointsToSame(0, Rotation2d.fromDegrees(0)),
            () -> setAllModuleSetpointsToSame(0, Rotation2d.fromDegrees(tuningTurnDelta.get())))
        .withName("tuneModuleTurn");
  }

  private Command tuningCmdTemplate(Runnable run1, Runnable run2) {
    return Commands.repeatingSequence(
        run(run1).withTimeout(1),
        run(() -> stop()).withTimeout(1),
        run(run2).withTimeout(1),
        run(() -> stop()).withTimeout(1));
  }
}

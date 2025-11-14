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

import static frc.robot.subsystems.drive.DriveConstants.*;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constantsGlobal.Constants;
import frc.robot.constantsGlobal.Constants.Mode;
import frc.robot.subsystems.drive.DriveConstants.DriveCommandsConfig;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.GeomUtil;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.PoseManager;
import frc.robot.util.Util;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private final PoseManager poseManager;
  private final DriveCommandsConfig config;
  private static final double DEADBAND = 0.1;

  private static final LoggedTunableNumber povMovementSpeed =
      new LoggedTunableNumber("Drive/POV Movement Speed", 0.5);

  private static final LoggedTunableNumber linearkP =
      new LoggedTunableNumber("Drive/Commands/Linear/kP", 3.5);
  private static final LoggedTunableNumber linearkD =
      new LoggedTunableNumber("Drive/Commands/Linear/kD", 0.0);
  private static final LoggedTunableNumber thetakP =
      new LoggedTunableNumber("Drive/Commands/Theta/kP", 6.0);
  private static final LoggedTunableNumber thetakD =
      new LoggedTunableNumber("Drive/Commands/Theta/D", 0.0);
  private static final LoggedTunableNumber linearTolerance =
      new LoggedTunableNumber("Drive/Commands/Linear/tolerance", 0.05);
  private static final LoggedTunableNumber thetaToleranceDeg =
      new LoggedTunableNumber("Drive/Commands/Theta/toleranceDeg", 2.0);
  private static final LoggedTunableNumber partialAutoLinearkP =
      new LoggedTunableNumber("Drive/Commands/partialAutoLinearkP", 1.0);

  private static final LoggedTunableNumber maxLinearVelocity =
      new LoggedTunableNumber("Drive/Commands/Linear - maxVelocity", Units.feetToMeters(7));
  private static final LoggedTunableNumber maxLinearAcceleration =
      new LoggedTunableNumber("Drive/Commands/Linear - maxAcceleration", 1);
  private static final LoggedTunableNumber maxAngularVelocity =
      new LoggedTunableNumber(
          "Drive/Commands/Theta - maxVelocity", maxAngularSpeedRadiansPerSec * 0.8);
  private static final LoggedTunableNumber maxAngularAcceleration =
      new LoggedTunableNumber(
          "Drive/Commands/Theta - maxAcceleration", maxAngularAccelerationRadiansPerSec * 0.8);

  private static final LoggedTunableNumber ffMinRadius =
      new LoggedTunableNumber("AutoAlign/ffMinRadius", 0.2);
  private static final LoggedTunableNumber ffMaxRadius =
      new LoggedTunableNumber("AutoAlign/ffMaxRadius", 0.8);

  private static final LoggedTunableNumber partialAutoToleranceDeg =
      new LoggedTunableNumber("Drive/Commands/PartialAuto/toleranceDeg", 7.5);
  private static final LoggedTunableNumber partialAutoFallOff =
      new LoggedTunableNumber("Drive/Commands/PartialAuto/fallOff", 2.0);

  private final ProfiledPIDController thetaController;
  private final ProfiledPIDController linearController;
  private Translation2d lastSetpointTranslation;

  private Timer joystickInterruptTimer = new Timer();
  private LoggedTunableNumber joystickInterruptDelay =
      new LoggedTunableNumber("Drive/JoystickInterruptDelay", 1);

  public static boolean nitro = false;

  // Autos
  private final LoggedTunableNumber xkPAuto = new LoggedTunableNumber("Drive/Choreo/xkP", 10);
  private final LoggedTunableNumber xkDAuto = new LoggedTunableNumber("Drive/Choreo/xkD", 0);
  private final LoggedTunableNumber ykPAuto = new LoggedTunableNumber("Drive/Choreo/ykP", 10);
  private final LoggedTunableNumber ykDAuto = new LoggedTunableNumber("Drive/Choreo/ykD", 0);
  private final LoggedTunableNumber rkPAuto = new LoggedTunableNumber("Drive/Choreo/rkP", 10);
  private final LoggedTunableNumber rkDAuto = new LoggedTunableNumber("Drive/Choreo/rkD", 0);

  private final PIDController xAutoController =
      new PIDController(xkPAuto.get(), 0.0, xkDAuto.get());
  private final PIDController yAutoController =
      new PIDController(ykPAuto.get(), 0.0, ykDAuto.get());
  private final PIDController headingAutoController =
      new PIDController(rkPAuto.get(), 0.0, rkDAuto.get());

  // Auto coast mode
  private boolean brakeMode = false;
  private Timer brakeModeTimer = new Timer();
  private static final double BREAK_MODE_DELAY_SEC = 10.0;

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

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

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

    headingAutoController.enableContinuousInput(-Math.PI, Math.PI);

    updateConstraints();
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    if (DriverStation.isDisabled()) {
      // Stop moving when disabled
      for (var module : modules) {
        module.stop();
      }

      // Log empty setpoint states when disabled
      Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - poseManager.lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        poseManager.lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        poseManager.rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        poseManager.rawGyroRotation =
            poseManager.rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }
      Logger.recordOutput("Drive/rawGyroRotation", poseManager.rawGyroRotation);

      // Apply update
      poseManager.addOdometryMeasurementWithTimestamps(sampleTimestamps[i], modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    // Add velocity data to pose manager, use gyro if possible
    ChassisSpeeds robotRelativeVelocity = getChassisSpeeds();
    robotRelativeVelocity.omegaRadiansPerSecond =
        gyroInputs.connected
            ? gyroInputs.yawVelocityRadPerSec
            : robotRelativeVelocity.omegaRadiansPerSecond;
    poseManager.addVelocityData(GeomUtil.toTwist2d(robotRelativeVelocity));

    // update the brake mode based on the robot's velocity and state (enabled/disabled)
    updateBrakeMode();

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          for (var module : modules) module.setTurnPIDF(turnKp.get());
        },
        turnKp);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> {
          for (var module : modules) module.setDrivePIDF(driveKp.get(), driveKd.get());
        },
        driveKp,
        driveKd);

    // Drive logic

    Logger.recordOutput("Drive/nitro", nitro);
    Util.logSubsystem(this, "Drive");
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    setModuleSetpoints(setpointStates);
    Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", discreteSpeeds);
  }

  private void setAllModuleSetpointsToSame(double speed, Rotation2d angle) {
    var moduleStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      moduleStates[i] = new SwerveModuleState(speed, angle);
    }
    setModuleSetpoints(moduleStates);
    Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", new ChassisSpeeds());
  }

  private void setModuleSetpoints(SwerveModuleState[] setpointStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, nitro ? nitroMaxSpeedMetersPerSec : maxSpeedMetersPerSec);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
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

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "Drive/SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  private boolean brakeHasBeenSet = false;
  private Timer setBrakeTimer = new Timer();

  /**
   * If the robot is enabled and brake mode is not enabled, enable it. If the robot is disabled, has
   * stopped moving for the specified period of time, and brake mode is enabled, disable it.
   */
  private void updateBrakeMode() {
    if (DriverStation.isTeleopEnabled() && !brakeMode) {
      brakeMode = true;
      setBrakeMode(true);
      brakeModeTimer.restart();
    } else if (DriverStation.isDisabled()) {
      if (!setBrakeTimer.isRunning()) {
        setBrakeTimer.start();
      }
      if (DriverStation.isAutonomous() && !brakeHasBeenSet) {
        if (setBrakeTimer.hasElapsed(3)) {
          setBrakeMode(false);
          brakeHasBeenSet = true;
          System.out.println(
              "Brake mode set while disabled and waiting for autonomous************************");
        }
      } else if (DriverStation.isTeleop()) {
        boolean stillMoving = false;
        double velocityLimit = 0.05; // In meters per second
        ChassisSpeeds measuredChassisSpeeds = getChassisSpeeds();
        if (Math.abs(measuredChassisSpeeds.vxMetersPerSecond) > velocityLimit
            || Math.abs(measuredChassisSpeeds.vyMetersPerSecond) > velocityLimit) {
          stillMoving = true;
          brakeModeTimer.restart();
        }
        if (brakeMode && !stillMoving && brakeModeTimer.hasElapsed(BREAK_MODE_DELAY_SEC)) {
          brakeMode = false;
          setBrakeMode(false);
        }
        brakeHasBeenSet = false;
        setBrakeTimer.reset();
      }
    }
  }

  public void setBrakeMode(boolean enable) {
    for (var module : modules) {
      module.setBrakeMode(enable);
    }
  }

  // Drive Commands
  public static enum DriveState {
    MANUAL,
    HEADING_CONTROL,
    AUTO_ALIGN,
    TRAJECTORY;
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public Command joystickDrive() {
    return run(() -> {
          // Get velocities from joysticks
          double omega = getAngularVelocityFromJoysticks();
          Translation2d linearVelocity = getLinearVelocityFromJoysticks();

          // Get pov movement
          double povX = 0;
          double povY = 0;
          if (config.povDownPressed()) {
            povX = -povMovementSpeed.get();
          } else if (config.povUpPressed()) {
            povX = povMovementSpeed.get();
          } else if (config.povLeftPressed()) {
            povY = povMovementSpeed.get();
          } else if (config.povRightPressed()) {
            povY = -povMovementSpeed.get();
          }

          // Convert to field relative speeds & send command
          if (Math.abs(povX) > 0 || Math.abs(povY) > 0) {
            runVelocity(new ChassisSpeeds(povX, povY, omega));
          } else {
            runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearVelocity.getX(),
                    linearVelocity.getY(),
                    omega,
                    AllianceFlipUtil.shouldFlip()
                        ? poseManager.getRotation().plus(new Rotation2d(Math.PI))
                        : poseManager.getRotation()));
          }
        })
        .withName("Joystick Drive");
  }

  /**
   * Field relative drive command using one joystick (controlling linear velocity) with a
   * ProfiledPID for angular velocity.
   */
  public Command headingDrive(Supplier<Rotation2d> goalHeading) {
    return run(() -> {
          Logger.recordOutput("Drive/Commands/goalHeading", goalHeading.get());
          updateThetaTunables();
          updateThetaConstraints();

          Translation2d linearVelocity = getLinearVelocityFromJoysticks();
          double angularVelocity = getAngularVelocityFromProfiledPID(goalHeading.get());

          // Convert to field relative speeds & send command
          runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX(),
                  linearVelocity.getY(),
                  angularVelocity,
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
        .onlyWhile(() -> MathUtil.applyDeadband(config.getOmegaInput(), DEADBAND) == 0)
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

          Pose2d targetPose = goalPose.get();

          // Calculate linear speed
          Translation2d driveVelocity = getLinearVelocityFromProfiledPID(targetPose);

          // Calculate theta speed
          double thetaVelocity = getAngularVelocityFromProfiledPID(targetPose);

          // Send command
          runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  driveVelocity.getX(),
                  driveVelocity.getY(),
                  thetaVelocity,
                  poseManager.getRotation()));

          Leds.getInstance().alignedWithTarget = linearAtGoal() && thetaAtGoal();
        })
        .beforeStarting(
            () -> {
              joystickInterruptTimer.restart();
              resetControllers(goalPose.get());
              Leds.getInstance().autoAlignActivated = true;
            })
        .finallyDo(
            () -> {
              stop();
              Leds.getInstance().alignedWithTarget = false;
              Leds.getInstance().autoAlignActivated = false;
            })
        .onlyWhile(this::noJoystickInput)
        .withName("Full Auto Drive");
  }

  public Command partialAutoDrive(Supplier<Pose2d> goalPose) {
    return run(() -> {
          // Get manual linear velocity
          Translation2d manualLinearVelocity = getLinearVelocityFromJoysticks();
          Translation2d flippedManualLinearVelocity =
              AllianceFlipUtil.shouldFlip()
                  ? manualLinearVelocity.rotateBy(new Rotation2d(Math.PI))
                  : manualLinearVelocity;

          // Auto-align section
          updateTunables();
          updateConstraints();

          Pose2d targetPose = goalPose.get();

          Translation2d driveVelocity = getLinearVelocityFromProfiledPID(targetPose);

          // Calculate theta speed
          double thetaVelocity = getAngularVelocityFromProfiledPID(targetPose);
          if (thetaController.atGoal()) thetaVelocity = 0.0;

          // Send command
          double maxDistance = 2;
          Translation2d distance =
              poseManager.getTranslation().minus(goalPose.get().getTranslation());

          // Linear blending
          double finalX =
              linearBlending(
                  distance.getX(),
                  maxDistance,
                  flippedManualLinearVelocity.getX(),
                  driveVelocity.getX(),
                  "X");
          double finalY =
              linearBlending(
                  distance.getY(),
                  maxDistance,
                  flippedManualLinearVelocity.getY(),
                  driveVelocity.getY(),
                  "Y");

          runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  finalX, finalY, thetaVelocity, poseManager.getRotation()));
        })
        .beforeStarting(() -> resetControllers(goalPose.get()));
  }

  private double linearBlending(
      double distance,
      double maxDistance,
      double manualVelocity,
      double autoVelocity,
      String axis) {
    double error = Math.min(Math.abs(distance) / partialAutoLinearkP.get(), maxDistance);
    double P = error / maxDistance;
    double manualFinal = manualVelocity * P;
    double autoFinal = autoVelocity * (1 - P);

    // Logger.recordOutput("Controls/partialAuto/"+axis+"P", P);
    // Logger.recordOutput("Controls/partialAuto/manual"+axis, manualFinal);
    // Logger.recordOutput("Controls/partialAuto/auto"+axis, autoFinal);

    return manualFinal + autoFinal;
  }

  public Command driveIntoWall() {
    return run(() -> setAllModuleSetpointsToSame(0.5, new Rotation2d()))
        .until(
            () -> {
              int count = 0;
              for (Module module : modules) {
                if (module.getDriveCurrent() > 30) count += 1;
                if (count >= 2) return true;
              }
              return false;
            });
  }

  private boolean noJoystickInput() {
    if (!joystickInterruptTimer.hasElapsed(joystickInterruptDelay.get())) {
      return true;
    }
    return MathUtil.applyDeadband(config.getOmegaInput(), 0.2) == 0
        && MathUtil.applyDeadband(Math.hypot(config.getXInput(), config.getYInput()), 0.2) == 0
        && !config.povDownPressed()
        && !config.povUpPressed()
        && !config.povLeftPressed()
        && !config.povRightPressed();
  }

  private double getAngularVelocityFromJoysticks() {
    // Convert to doubles
    double o = config.getOmegaInput();

    // Apply deadband
    double omega = MathUtil.applyDeadband(o, DEADBAND);

    // Check for slow mode
    if (config.slowMode().getAsBoolean()) {
      omega *= config.slowTurnMultiplier().get();
    }

    // Square values and scale to max velocity
    omega = Math.copySign(omega * omega, omega);
    omega *= maxAngularSpeedRadiansPerSec;

    return omega;
  }

  private Translation2d getLinearVelocityFromJoysticks() {
    // Convert to doubles
    double x = config.getXInput();
    double y = config.getYInput();

    // Check for slow mode
    if (config.slowMode().getAsBoolean()) {
      double multiplier = config.slowDriveMultiplier().get();
      x *= multiplier;
      y *= multiplier;
    }

    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square values and scale to max velocity
    linearMagnitude = linearMagnitude * linearMagnitude;
    linearMagnitude *= nitro ? nitroMaxSpeedMetersPerSec : maxSpeedMetersPerSec;

    // Calcaulate new linear velocity
    Translation2d linearVelocity = new Translation2d(linearMagnitude, linearDirection);

    return linearVelocity;
  }

  private double getAngularVelocityFromProfiledPID(Pose2d targetPose) {
    return getAngularVelocityFromProfiledPID(targetPose.getRotation());
  }

  private double getAngularVelocityFromProfiledPID(Rotation2d rotation) {
    double output =
        thetaController.calculate(poseManager.getRotation().getRadians(), rotation.getRadians());

    if (thetaController.atGoal()) output = 0.0;

    Logger.recordOutput("Drive/Commands/HeadingError", thetaController.getPositionError());
    return output;
  }

  private Translation2d getLinearVelocityFromProfiledPID(Pose2d targetPose) {
    double currentDistance = poseManager.getDistanceTo(targetPose);
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - ffMinRadius.get()) / (ffMaxRadius.get() - ffMinRadius.get()),
            0.0,
            0.5);

    // Reset the linear controller
    linearController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        linearController.getSetpoint().velocity);

    // Calculate linear speed
    double driveVelocityScalar =
        linearController.getSetpoint().velocity * ffScaler
            + linearController.calculate(currentDistance, 0.0);

    if (linearAtGoal()) driveVelocityScalar = 0.0;

    lastSetpointTranslation =
        new Pose2d(targetPose.getTranslation(), poseManager.getHorizontalAngleTo(targetPose))
            .transformBy(GeomUtil.toTransform2d(linearController.getSetpoint().position, 0.0))
            .getTranslation();

    // Calculate angle to target then transform by velocity scalar
    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                poseManager.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.toTransform2d(driveVelocityScalar, 0.0))
            .getTranslation();

    return driveVelocity;
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
            -new Translation2d(fieldVelocity.dx, fieldVelocity.dy)
                .rotateBy(poseManager.getHorizontalAngleTo(goalPose).unaryMinus())
                .getX());
    linearController.reset(poseManager.getDistanceTo(goalPose), linearVelocity);
    resetThetaController();
    lastSetpointTranslation = poseManager.getTranslation();
  }

  private void resetThetaController() {
    Pose2d currentPose = poseManager.getPose();
    Twist2d fieldVelocity = poseManager.fieldVelocity();
    thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.dtheta);
  }

  /** Returns true if within tolerance of aiming at goal */
  @AutoLogOutput(key = "Drive/Commands/LinearAtGoal")
  public boolean linearAtGoal() {
    return linearController.atGoal();
  }

  /** Returns true if within tolerance of aiming at speaker */
  @AutoLogOutput(key = "Drive/Commands/ThetaAtGoal")
  public boolean thetaAtGoal() {
    return thetaController.atGoal();
  }

  // Autos
  public void followTrajectory(SwerveSample sample) {
    updateAutoTunables();
    Pose2d pose = poseManager.getPose();

    double xFF = sample.vx;
    double yFF = sample.vy;
    double rotationFF = sample.omega;

    double xFeedback = xAutoController.calculate(pose.getX(), sample.x);
    double yFeedback = yAutoController.calculate(pose.getY(), sample.y);
    double rotationFeedback =
        headingAutoController.calculate(pose.getRotation().getRadians(), sample.heading);

    ChassisSpeeds out =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, pose.getRotation());

    Logger.recordOutput(
        "Drive/Choreo/Target Pose", new Pose2d(sample.x, sample.y, new Rotation2d(sample.heading)));
    Logger.recordOutput("Drive/Choreo/Target Speeds", out);

    runVelocity(out);
  }

  private void updateAutoTunables() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> xAutoController.setPID(xkPAuto.get(), 0, xkDAuto.get()),
        xkPAuto,
        xkDAuto);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> yAutoController.setPID(ykPAuto.get(), 0, ykDAuto.get()),
        ykPAuto,
        ykDAuto);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> headingAutoController.setPID(rkPAuto.get(), 0, rkDAuto.get()),
        rkPAuto,
        rkDAuto);
  }

  // Tuning Commands
  // * For tuning drive motor PID values use Phoenix Tuner X (and maybe ff also?)
  private static final LoggedTunableNumber tuningTurnDelta =
      new LoggedTunableNumber("Drive/ModuleTunables/turnDeltaForTuning", 90);
  private static final LoggedTunableNumber tuningDriveSpeed =
      new LoggedTunableNumber("Drive/ModuleTunables/tuningDriveSpeed", 3);

  public Command tuneModuleTurn() {
    return run(() -> {
          setAllModuleSetpointsToSame(0, Rotation2d.fromDegrees(tuningTurnDelta.get()));
          LoggedTunableNumber.ifChanged(
              hashCode(),
              () -> {
                for (var module : modules) module.setTurnPIDF(turnKp.get());
              },
              turnKp,
              tuningTurnDelta);
        })
        .withTimeout(2.0)
        .withName("tuneModuleTurn");
  }

  public Command tuneModuleDrive() {
    return startRun(
            () -> {
              for (var module : modules) module.setDrivePIDF(driveKp.get(), driveKd.get());
            },
            () -> {
              setAllModuleSetpointsToSame(tuningDriveSpeed.get(), new Rotation2d());
            })
        .withTimeout(2.0)
        .withName("tuneModuleDrive");
  }

  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public Command feedforwardCharacterization() {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        run(() -> {
              runCharacterization(0.0);
            })
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        run(() -> {
              double voltage = timer.get() * FF_RAMP_RATE;
              this.runCharacterization(voltage);
              velocitySamples.add(getFFCharacterizationVelocity());
              voltageSamples.add(voltage);
            })

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public Command wheelRadiusCharacterization() {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                })),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = getWheelRadiusCharacterizationPositions();
                  state.lastAngle = poseManager.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = poseManager.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }

  // Module Testing
  private int moduleToTest = 0;
  public boolean allModules = false;

  public Command moduleTestingCommand(DoubleSupplier driveInput, DoubleSupplier turnInput) {
    return run(() -> {
          double driveOutput = MathUtil.applyDeadband(driveInput.getAsDouble(), DEADBAND);
          double turnOutput = MathUtil.applyDeadband(turnInput.getAsDouble(), DEADBAND);
          if (allModules) {
            for (int i = 0; i < 4; i++) {
              modules[i].test(driveOutput * 12, turnOutput * 12);
            }
          } else {
            modules[moduleToTest].test(driveOutput * 12, turnOutput * 12);
          }
        })
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Module Testing Command");
  }

  public Command setModuleToTest(int moduleIndex) {
    return Commands.runOnce(() -> moduleToTest = moduleIndex).withName("Set Module To Test");
  }
}

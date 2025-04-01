package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.RobotCommands.IntakeState.*;
import static frc.robot.RobotCommands.ScoreState.*;
import static frc.robot.constantsGlobal.FieldConstants.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.*;
import static frc.robot.subsystems.intake.IntakeConstants.groundAlgae;
import static frc.robot.util.AllianceFlipUtil.apply;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants.DriveCommandsConfig;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Leds;
import frc.robot.util.PoseManager;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Put high level commands here */
public final class RobotCommands {
  public static boolean allowAutoDrive = true;
  public static ScoreState scoreState = Dealgify;
  public static boolean dealgifyAfterPlacing = false;

  public static Command scoreCoral(
      Elevator elevator, Carriage carriage, PoseManager poseManager, BooleanSupplier atPose) {
    return scoreCoral(elevator, carriage, poseManager, goalPose(poseManager), atPose);
  }

  public static Command scoreCoral(
      Elevator elevator,
      Carriage carriage,
      PoseManager poseManager,
      Supplier<Pose2d> goalPose,
      BooleanSupplier atPose) {
    return waitUntil(nearPose(poseManager, goalPose))
        .andThen(
            elevator
                .enableElevator()
                .alongWith(
                    either(
                            waitUntil(elevator::pastL3Height).andThen(carriage.backUpForL3()),
                            none(),
                            () ->
                                elevator.goalHeightInches > ElevatorConstants.pastL3Height.get()
                                    && !Carriage.coralInDanger)
                        .andThen(
                            waitUntil(() -> atPose.getAsBoolean() && elevator.atGoalHeight()),
                            carriage.placeCoral())));
  }

  public static Command dealgify(
      Elevator elevator, Carriage carriage, PoseManager poseManager, BooleanSupplier atPose) {
    return dealgify(elevator, carriage, poseManager, goalPose(poseManager), atPose);
  }

  public static Command dealgify(
      Elevator elevator,
      Carriage carriage,
      PoseManager poseManager,
      Supplier<Pose2d> goalPose,
      BooleanSupplier atPose) {
    BooleanSupplier highAlgae =
        () -> {
          if (DriverStation.isTest()) return false;
          return poseManager.closestFaceHighAlgae();
        };
    return waitUntil(nearPose(poseManager, goalPose))
        .andThen(
            parallel(
                either(elevator.request(AlgaeHigh), elevator.request(AlgaeLow), highAlgae)
                    .andThen(elevator.enableElevator()),
                either(carriage.highDealgify(), carriage.lowDealgify(), highAlgae)))
        .alongWith(
            runOnce(() -> Logger.recordOutput("Controls/HighAlgae", highAlgae.getAsBoolean())));
  }

  public static Command scoreProcessorOrL1(
      Carriage carriage,
      Intake intake,
      Elevator elevator,
      PoseManager poseManager,
      boolean front,
      BooleanSupplier atPose) {
    return waitUntil(atPose)
        .andThen(
            either(
                elevator
                    .request(Processor)
                    .andThen(elevator.enableElevator(), carriage.scoreProcessor()),
                intake.poopCmd(),
                () -> front))
        .withName("scoreProcessor");
  }

  private static BooleanSupplier nearPose(PoseManager poseManager, Supplier<Pose2d> goalPose) {
    return () -> {
      boolean extra = false;
      if (DriverStation.isTeleop()) {
        extra = !allowAutoDrive;
      } else if (DriverStation.isTest()) {
        extra = true;
      }
      ;
      return extra
          || poseManager.getDistanceTo(goalPose.get()) < elevatorSafeExtensionDistanceMeters.get();
    };
  }

  public static BooleanSupplier atGoal(Drive drive, DriveCommandsConfig driveCommandsConfig) {
    return () ->
        driveCommandsConfig.finishScoring() || (drive.linearAtGoal() && drive.thetaAtGoal());
  }

  public static Supplier<Pose2d> goalPose(PoseManager poseManager) {
    return () -> {
      switch (scoreState) {
        default:
          return apply(poseManager.closest(scoreState));
        case ProcessorFront:
          return apply(processorScore);
        case ProcessorBack:
          return apply(processorScore).transformBy(new Transform2d(0, 0, new Rotation2d(Math.PI)));
      }
    };
  }

  public static enum ScoreState {
    LeftBranch,
    RightBranch,
    Dealgify,
    ProcessorFront,
    ProcessorBack,
    ScoreL1
  }

  public static IntakeState intakeState = Source;

  public static Command fullIntake(
      Drive drive, Carriage carriage, Intake intake, Elevator elevator, PoseManager poseManager) {
    return select(
            Map.of(
                Source,
                either(
                    drive
                        .headingDrive(
                            () -> {
                              return poseManager.closestStation().getRotation();
                            })
                        .until(carriage::coralHeld)
                        .asProxy(),
                    none(),
                    () -> allowAutoDrive),
                Ground,
                intake.intakeCmd().asProxy(),
                Ice_Cream,
                elevator
                    .request(IceCream)
                    .andThen(elevator.enableElevator().alongWith(carriage.lowDealgify()))
                    .raceWith(
                        either(intake.iceCreamCmd().asProxy(), idle(), () -> groundAlgae.get()))
                    .withName("iceCreamIntake")
                    .asProxy()),
            () -> intakeState)
        .beforeStarting(() -> Leds.getInstance().intakingActivated = true)
        .finallyDo(() -> Leds.getInstance().intakingActivated = false)
        .withName("fullIntake");
  }

  public static Command lowLevelCoralIntake(Carriage carriage, Funnel funnel) {
    return carriage
        .intakeCoral()
        .until(carriage::coralHeld)
        .alongWith(funnel.runRollers().until(() -> carriage.coralPassed))
        .withName("lowLevelCoralIntake");
  }

  public static enum IntakeState {
    Source,
    Ice_Cream,
    Ground
  }

  public static Command reIntake(Funnel funnel, Carriage carriage){
    return Commands.deadline(carriage.ejectCoral(), funnel.eject())
    .andThen(lowLevelCoralIntake(carriage, funnel));

  }
}

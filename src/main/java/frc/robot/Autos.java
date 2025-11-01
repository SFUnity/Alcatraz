package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.RobotCommands.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedAutoChooser;
import frc.robot.util.PoseManager;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private final Drive drive;
  private final Carriage carriage;
  private final Elevator elevator;
  private final Intake intake;
  private final Funnel funnel;
  private final PoseManager poseManager;

  private final AutoFactory factory;
  private final LoggedAutoChooser chooser;

  private final LoggedDashboardChooser<Command> nonChoreoChooser =
      new LoggedDashboardChooser<Command>("Non-Choreo Chooser");
  private static final boolean isChoreoAuto = true;

  public static boolean moveRight = false;
  public static boolean moveLeft = false;

  public Autos(
      Drive drive,
      Carriage carriage,
      Elevator elevator,
      Intake intake,
      Funnel funnel,
      PoseManager poseManager) {
    this.drive = drive;
    this.carriage = carriage;
    this.elevator = elevator;
    this.intake = intake;
    this.funnel = funnel;
    this.poseManager = poseManager;

    factory =
        new AutoFactory(
            poseManager::getPose,
            poseManager::setPose,
            drive::followTrajectory,
            true,
            drive,
            (Trajectory<SwerveSample> traj, Boolean bool) -> {
              Logger.recordOutput(
                  "Drive/Choreo/Active Traj",
                  (AllianceFlipUtil.shouldFlip() ? traj.flipped() : traj).getPoses());
              Logger.recordOutput(
                  "Drive/Choreo/Current Traj End Pose",
                  traj.getFinalPose(AllianceFlipUtil.shouldFlip()).get());
              Logger.recordOutput(
                  "Drive/Choreo/Current Traj Start Pose",
                  traj.getInitialPose(AllianceFlipUtil.shouldFlip()).get());
            });

    /* Set up main choreo routines */
    chooser = new LoggedAutoChooser("ChoreoChooser");
    chooser.addRoutine("Example Auto Routine", this::StanderedCoralAuto);
    // chooser.addRoutine("Example Auto Routine", this::exampleAutoRoutine);

    if (!DriverStation.isFMSAttached()) {
      // Set up test choreo routines
      chooser.addRoutine("StraightLine", this::StraightLine);
      chooser.addRoutine("Spin", this::Spin);

      // SysID & non-choreo routines
      if (!isChoreoAuto) {
        nonChoreoChooser.addOption("Module Turn Tuning", drive.tuneModuleTurn());
        nonChoreoChooser.addOption("Module Drive Tuning", drive.tuneModuleDrive());

        // Set up SysId routines
        nonChoreoChooser.addOption(
            "Drive Wheel Radius Characterization", drive.wheelRadiusCharacterization());
        nonChoreoChooser.addOption(
            "Drive Simple FF Characterization", drive.feedforwardCharacterization());
      }
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return isChoreoAuto ? chooser.selectedCommandScheduler() : nonChoreoChooser.get();
  }

  private AutoRoutine StraightLine() {
    AutoRoutine routine = factory.newRoutine("StraightLine");

    AutoTrajectory StraightLine = routine.trajectory("StraightLine");

    routine.active().onTrue(StraightLine.resetOdometry().andThen(StraightLine.cmd()));

    return routine;
  }

  private AutoRoutine Spin() {
    AutoRoutine routine = factory.newRoutine("Spin");

    AutoTrajectory Spin = routine.trajectory("Spin");

    routine.active().onTrue(Spin.resetOdometry().andThen(Spin.cmd()));

    return routine;
  }

  public AutoRoutine pickupAndScoreAuto() {
    AutoRoutine routine = factory.newRoutine("taxi");

    // Load the routine's trajectories
    AutoTrajectory driveToMiddle = routine.trajectory("Starting Path");
    AutoTrajectory driveToFeadingStation = routine.trajectory("Feading Station Path");
    AutoTrajectory driveToNewCoral = routine.trajectory("Place New Coral Path");
    AutoTrajectory driveToFeedingStationTwo = routine.trajectory("Back To Feading Station Path");

    // When the routine begins, reset odometry and start the first trajectory (1)
    routine.active().onTrue(Commands.sequence(driveToMiddle.resetOdometry(), driveToMiddle.cmd()));

    driveToMiddle
        .done()
        .onTrue(
            funnel
                .eject()
                .withTimeout(1)
                .andThen(
                    Commands.sequence(
                        driveToFeadingStation.resetOdometry(), driveToFeadingStation.cmd())));

    driveToFeadingStation
        .done()
        .onTrue(
            funnel.runRollers().withTimeout(1).andThen(Commands.sequence((driveToNewCoral.cmd()))));

    driveToNewCoral
        .done()
        .onTrue(
            funnel
                .eject()
                .withTimeout(1)
                .andThen(Commands.sequence((driveToFeedingStationTwo.cmd()))));

    driveToFeedingStationTwo.done().onTrue(funnel.runRollers().withTimeout(1));

    return routine;
  }

  private AutoRoutine StanderedCoralAuto() {
    AutoRoutine routine = factory.newRoutine("StanderedCoralAuto");

    AutoTrajectory centerLeftToE = routine.trajectory("CenterLeftToE");
    AutoTrajectory eToFeeding = routine.trajectory("EToFeeding");
    AutoTrajectory feedingToC = routine.trajectory("FeedingToC");
    AutoTrajectory cToCD_Algae = routine.trajectory("CToCD_Algae");
    AutoTrajectory cD_AlgaeToFeeding = routine.trajectory("CD_AlgaeToFeeding");
    AutoTrajectory feedingToD = routine.trajectory("FeedingToD");

    routine
        .observe(() -> poseManager.nearStation(1.75))
        .whileTrue(RobotCommands.lowLevelCoralIntake(carriage, funnel));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                centerLeftToE.resetOdometry(), centerLeftToE.cmd()
                // eToFeeding.cmd(),
                // feedingToC.cmd(),
                // cToCD_Algae.cmd(),
                // cD_AlgaeToFeeding.cmd(),
                // feedingToD.cmd()
                ));

    centerLeftToE
        .active()
        .onTrue(
            elevator
                .request(L2)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> centerLeftToE.getFinalPose().get(),
                        centerLeftToE.active().negate())));

    centerLeftToE
        .done()
        .onTrue(waitUntil(() -> !carriage.coralHeld()).andThen(eToFeeding.cmd().asProxy()));

    eToFeeding.done().onTrue(waitUntil(carriage::beamBreak).andThen(feedingToC.cmd().asProxy()));

    feedingToC
        .active()
        .onTrue(
            elevator
                .request(L3)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> feedingToC.getFinalPose().get(),
                        feedingToC.active().negate())));
    feedingToC
        .done()
        .onTrue(waitUntil(() -> !carriage.coralHeld()).andThen(cToCD_Algae.cmd().asProxy()));
    cToCD_Algae
        .active()
        .onTrue(
            dealgify(
                elevator,
                carriage,
                poseManager,
                () -> cToCD_Algae.getFinalPose().get(),
                cToCD_Algae.active().negate()));              
    cToCD_Algae
        .done()
        .onTrue(waitUntil(() -> !carriage.algaeHeld()).andThen(cD_AlgaeToFeeding.cmd().asProxy()));

    cD_AlgaeToFeeding.active().onTrue(waitSeconds(1).andThen(carriage.ejectAlgae()));

    cD_AlgaeToFeeding.done().onTrue(waitUntil(carriage::beamBreak).andThen(feedingToD.cmd().asProxy()));

    feedingToD.active().onTrue(elevator
    .request(L3)
    .andThen(
        scoreCoral(
            elevator,
            carriage,
            poseManager,
            () -> feedingToD.getFinalPose().get(),
            feedingToD.active().negate())));

    return routine;
  }
}

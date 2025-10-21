package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.RobotCommands.*;
import static frc.robot.RobotCommands.ScoreState.Dealgify;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.L2;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.L3;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.carriage.Carriage;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AlwaysLoggedTunableNumber;
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

  private int coralOnL3 = 0;
  private int coralOnL2 = 0;
  private final AlwaysLoggedTunableNumber delayAfterAlgaeIntake =
      new AlwaysLoggedTunableNumber("delayAfterAlgaeIntake", 0);
  private final AlwaysLoggedTunableNumber delayBeforeMoving =
      new AlwaysLoggedTunableNumber("delayBeforeMoving", 3);

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
    // chooser.addRoutine("Example Auto Routine", this::exampleAutoRoutine);
    chooser.addRoutine("WallLKAlgaeL2L3", this::WallLKAlgaeL2L3);
    chooser.addRoutine("ProcessorCDAlgaeL2L3", this::CenterCDAlgaeL2L3);
    chooser.addRoutine("L3Only", this::L3Only);
    chooser.addRoutine("CenterGHAlgaeToProcessorL3", this::GHAlgaeToProcessorL3);

    if (!DriverStation.isFMSAttached()) {
      // Set up test choreo routines
      chooser.addRoutine("StraightLine", this::StraightLine);
      chooser.addRoutine("Spin", this::Spin);
      chooser.addRoutine("chaos", this::chaos);

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

  // Options: .done() = when routine is done, AtTime("x") = run cmd on eventMarker,
  // .active().whileTrue() =  Trigger while the trajectory is still running.
  // Routines

  private AutoRoutine WallLKAlgaeL2L3() {
    AutoRoutine routine = factory.newRoutine("WallLKAlgaeL2L3");
    AutoTrajectory CenterWToJ = routine.trajectory("CenterWToJ");
    AutoTrajectory JToStationHigh = routine.trajectory("JToStationHigh");
    AutoTrajectory LToDealgify = routine.trajectory("LToDealgify");
    AutoTrajectory KLAlgaeToStationHigh = routine.trajectory("KLAlgaeToStationHigh");
    AutoTrajectory StationHighToK = routine.trajectory("StationHighToK");
    AutoTrajectory KToStationHigh = routine.trajectory("KToStationHigh");
    AutoTrajectory StationHighToL = routine.trajectory("StationHighToL");
    AutoTrajectory LToStationHigh = routine.trajectory("LToStationHigh");

    return StandardCoralAuto(
        routine,
        CenterWToJ,
        JToStationHigh,
        LToDealgify,
        KLAlgaeToStationHigh,
        StationHighToK,
        KToStationHigh,
        StationHighToL,
        LToStationHigh);
  }

  private AutoRoutine CenterCDAlgaeL2L3() {
    AutoRoutine routine = factory.newRoutine("CenterCDAlgaeL2L3");
    AutoTrajectory CenterWToJ;
    if (DriverStation.getAlliance().orElseGet(() -> DriverStation.Alliance.Red)
        == DriverStation.Alliance.Blue) {
      CenterWToJ = routine.trajectory("CenterPToE");
    } else {
      CenterWToJ = routine.trajectory("CenterPToE");
    }
    AutoTrajectory JToStationHigh = routine.trajectory("EToStationLow");
    AutoTrajectory LToDealgify = routine.trajectory("CToDealgify");
    AutoTrajectory KLAlgaeToStationHigh = routine.trajectory("CDToStationLow");
    AutoTrajectory StationHighToK = routine.trajectory("StationLowToD");
    AutoTrajectory KToStationHigh = routine.trajectory("DToStationLow");
    AutoTrajectory StationHighToL = routine.trajectory("StationLowToC");
    AutoTrajectory LToStationHigh = routine.trajectory("CToStationLow");

    return StandardCoralAuto(
        routine,
        CenterWToJ,
        JToStationHigh,
        LToDealgify,
        KLAlgaeToStationHigh,
        StationHighToK,
        KToStationHigh,
        StationHighToL,
        LToStationHigh);
  }

  private AutoRoutine StandardCoralAuto(
      AutoRoutine routine,
      AutoTrajectory CenterWToJ,
      AutoTrajectory JToStationHigh,
      AutoTrajectory LToDealgify,
      AutoTrajectory KLAlgaeToStationHigh,
      AutoTrajectory StationHighToK,
      AutoTrajectory KToStationHigh,
      AutoTrajectory StationHighToL,
      AutoTrajectory LToStationHigh) {

    // Intake when near station
    new Trigger(() -> poseManager.nearStation(1.75))
        .whileTrue(RobotCommands.lowLevelCoralIntake(carriage, funnel));

    // When the routine begins, reset odometry and start the first trajectory
    routine
        .active()
        .onTrue(
            CenterWToJ.resetOdometry()
                .andThen(
                    CenterWToJ.cmd()
                        .alongWith(
                            runOnce(
                                () -> {
                                  coralOnL3 = 0;
                                  coralOnL2 = 0;
                                })))
                .withName("ResetOdometryAndStartFirstTrajectory"));
    CenterWToJ.active()
        .onTrue(
            elevator
                .request(L2)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> CenterWToJ.getFinalPose().get(),
                        CenterWToJ.active().negate()))
                .withName("ScoreCoralOnL3"));
    CenterWToJ.done()
        .onTrue(
            waitUntil(() -> !carriage.coralHeld())
                .andThen(JToStationHigh.cmd().asProxy())
                .withName("DealgifyThenGoToStationHigh"));
    JToStationHigh.done()
        .onTrue(waitUntil(carriage::beamBreak).andThen(StationHighToL.cmd().asProxy()));
    StationHighToL.active()
        .and(carriage::fullCoralHeld)
        .and(() -> coralOnL3 < 1)
        .onTrue(
            // Score coral on L3
            elevator
                .request(L3)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> StationHighToL.getFinalPose().get(),
                        StationHighToL.active().negate()),
                    runOnce(
                        () -> {
                          coralOnL3 = 1;
                          coralOnL2 = 0;
                        }),
                    runOnce(() -> scoreState = Dealgify),
                    LToDealgify.cmd().andThen(drive.driveIntoWall()).asProxy(),
                    dealgify(
                        elevator,
                        carriage,
                        poseManager,
                        () -> StationHighToL.getFinalPose().get(),
                        StationHighToL.active().negate()))
                .withName("ScoreCoralOnL3"));
    LToDealgify.done()
        .onTrue(
            waitUntil(carriage::algaeHeld)
                .andThen(
                    KLAlgaeToStationHigh.cmd()
                        .asProxy()
                        .alongWith(
                            runOnce(
                                () -> {
                                  coralOnL3 = 1;
                                  coralOnL2 = 0;
                                })))
                .withName("DealgifyThenGoToStationHigh"));

    // Eject algae while driving
    KLAlgaeToStationHigh.atTime("EjectAlgae").onTrue(carriage.ejectAlgae());

    // Drive back from the station to our next scoring location
    // We're intaking coral with a trigger in Robot.java so we don't need to do it here
    KLAlgaeToStationHigh.done()
        .or(KToStationHigh.done())
        .or(LToStationHigh.done())
        .onTrue(
            waitUntil(carriage::beamBreak)
                .andThen(
                    either(
                        StationHighToL.cmd(),
                        StationHighToK.cmd(),
                        () -> (coralOnL2 + coralOnL3) % 2 == 0) // Alternate K and L
                    )
                .withName("StationToScore"));

    StationHighToK.active()
        .and(carriage::fullCoralHeld)
        .and(() -> poseManager.getDistanceTo(StationHighToK.getFinalPose().get()) < 1)
        .onTrue(
            either(
                    elevator.request(L2).finallyDo(() -> coralOnL2 += 1),
                    elevator.request(L3).finallyDo(() -> coralOnL3 += 1),
                    () -> coralOnL3 >= 2)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> StationHighToK.getFinalPose().get(),
                        StationHighToK.active().negate()))
                .withName("ScoreOnK"));

    StationHighToK.done()
        .onTrue(
            waitUntil(() -> !carriage.coralHeld())
                .andThen(KToStationHigh.cmd())
                .withName("KToStationHigh"));

    StationHighToL.active()
        .and(carriage::fullCoralHeld)
        .and(() -> coralOnL3 >= 1)
        .and(() -> poseManager.getDistanceTo(StationHighToL.getFinalPose().get()) < 1)
        .onTrue(
            either(
                    elevator.request(L2).finallyDo(() -> coralOnL2 += 1),
                    elevator.request(L3).finallyDo(() -> coralOnL3 += 1),
                    () -> coralOnL3 >= 2)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> StationHighToL.getFinalPose().get(),
                        StationHighToL.active().negate()))
                .withName("ScoreOnL"));

    StationHighToL.done()
        .onTrue(
            waitUntil(() -> !carriage.coralHeld())
                .andThen(LToStationHigh.cmd())
                .withName("LToStationHigh"));

    // Logging
    routine
        .active()
        .whileTrue(
            run(
                () -> {
                  Logger.recordOutput("Drive/Choreo/CoralOnL3", coralOnL3);
                  Logger.recordOutput("Drive/Choreo/CoralOnL2", coralOnL2);
                }));

    return routine;
  }

  private AutoRoutine GHAlgaeToProcessorL3() {

    AutoRoutine routine = factory.newRoutine("GHAlgaeToProcessorL3");

    AutoTrajectory CenterWallToHG = routine.trajectory("CenterToHGAlgae");
    AutoTrajectory HToDealgify = routine.trajectory("HToDealgify");
    AutoTrajectory GHToProcessorScore = routine.trajectory("GHToProcessorScore");

    // When the routine begins, reset odometry and start the first trajectory
    routine
        .active()
        .onTrue(
            CenterWallToHG.resetOdometry()
                .andThen(waitSeconds(delayBeforeMoving.get()), CenterWallToHG.cmd().asProxy())
                .withName("ResetOdometryAndStartFirstTrajectory"));
    CenterWallToHG.active()
        .onTrue(
            // Score coral on L3
            elevator
                .request(L3)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> CenterWallToHG.getFinalPose().get(),
                        CenterWallToHG.active().negate()),
                    runOnce(() -> scoreState = Dealgify),
                    HToDealgify.cmd().andThen(drive.driveIntoWall()).asProxy(),
                    dealgify(
                        elevator,
                        carriage,
                        poseManager,
                        () -> CenterWallToHG.getFinalPose().get(),
                        CenterWallToHG.active().negate()))
                .withName("ScoreCoralOnL3"));
    HToDealgify.done()
        .onTrue(
            waitUntil(carriage::algaeHeld)
                .andThen(
                    Commands.waitSeconds(delayAfterAlgaeIntake.get()),
                    GHToProcessorScore.cmd().asProxy())
                .withName("ScoreProcessor"));
    GHToProcessorScore.done()
        .onTrue(
            scoreProcessorOrL1(
                carriage,
                intake,
                elevator,
                poseManager,
                true,
                GHToProcessorScore.active().negate()));

    return routine;
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

  public AutoRoutine L3Only() {
    AutoRoutine routine = factory.newRoutine("L3Only");
    AutoTrajectory CenterProcessorToCDAlgae = routine.trajectory("CenterProcessorToCDAlgae");
    AutoTrajectory CDToStationLow = routine.trajectory("CDToStationLow");
    AutoTrajectory StationLowToC = routine.trajectory("StationLowToC");
    AutoTrajectory CToStationLow = routine.trajectory("CToStationLow");
    AutoTrajectory StationLowToD = routine.trajectory("StationLowToD");
    AutoTrajectory DToStationLow = routine.trajectory("DToStationLow");

    // When the routine begins, reset odometry and start the first trajectory
    routine
        .active()
        .onTrue(
            CenterProcessorToCDAlgae.resetOdometry()
                .andThen(CenterProcessorToCDAlgae.cmd())
                .withName("ResetOdometryAndStartFirstTrajectory"));
    CenterProcessorToCDAlgae.active()
        .onTrue(
            // Score coral on L3
            elevator
                .request(L3)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> CenterProcessorToCDAlgae.getFinalPose().get(),
                        CenterProcessorToCDAlgae.done()),
                    runOnce(() -> scoreState = Dealgify),
                    dealgify(
                        elevator,
                        carriage,
                        poseManager,
                        () -> CenterProcessorToCDAlgae.getFinalPose().get(),
                        CenterProcessorToCDAlgae.done()))
                .withName("ScoreCoralOnL3"));
    CenterProcessorToCDAlgae.done()
        .onTrue(
            waitUntil(() -> carriage.algaeHeld())
                .andThen(
                    // Start next path once algae is held
                    CDToStationLow.cmd())
                .withName("DealgifyThenGoToStationHigh"));

    // Eject algae while driving
    CDToStationLow.atTime("EjectAlgae1").onTrue(carriage.ejectAlgae());

    // Drive back from the station to our next scoring location
    // We're intaking coral with a trigger in Robot.java so we don't need to do it here
    CDToStationLow.done()
        .onTrue(
            waitUntil(() -> carriage.coralHeld())
                .andThen(StationLowToC.cmd().alongWith(elevator.request(L3))));
    StationLowToC.done()
        .onTrue(
            scoreCoral(
                elevator,
                carriage,
                poseManager,
                () -> StationLowToC.getFinalPose().get(),
                StationLowToC.done()));
    StationLowToC.done()
        .onTrue(waitUntil(() -> !carriage.coralHeld()).andThen(CToStationLow.cmd()));
    CToStationLow.done()
        .onTrue(
            waitUntil(() -> carriage.coralHeld())
                .andThen(StationLowToD.cmd().alongWith(elevator.request(L3))));
    StationLowToD.done()
        .onTrue(
            scoreCoral(
                elevator,
                carriage,
                poseManager,
                () -> StationLowToD.getFinalPose().get(),
                StationLowToD.done()));
    StationLowToD.done()
        .onTrue(
            waitUntil(() -> !carriage.coralHeld())
                .andThen(DToStationLow.cmd().alongWith(elevator.request(L3))));
    DToStationLow.done().onTrue(waitUntil(() -> carriage.coralHeld()));

    return routine;
  }

  // Do not mess with this :
  private AutoRoutine chaos() {
    AutoRoutine routine = factory.newRoutine("chaos");

    AutoTrajectory chaos = routine.trajectory("chaos");
    routine.active().onTrue(chaos.resetOdometry().andThen(chaos.cmd()));
    return routine;
  }
}

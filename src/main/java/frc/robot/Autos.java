package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.RobotCommands.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHeight.L2;

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
    // chooser.addRoutine("Example Auto Routine", this::exampleAutoRoutine);

    if (!DriverStation.isFMSAttached()) {
      // Set up test choreo routines
      chooser.addRoutine("StraightLine", this::StraightLine);
      chooser.addRoutine("Spin", this::Spin);
      chooser.addRoutine("Alcatraz Auto", this::StandardCoralAuto);

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

  private AutoRoutine pickupAndScoreAuto() {
    AutoRoutine routine = factory.newRoutine("taxi");

    // Load the routine's trajectories
    AutoTrajectory driveToMiddle = routine.trajectory("Start Path");

    // When the routine begins, reset odometry and start the first trajectory (1)
    routine.active().onTrue(Commands.sequence(driveToMiddle.resetOdometry(), driveToMiddle.cmd()));

    driveToMiddle.done().onTrue(funnel.eject().withTimeout(1));

    return routine;
  }

  private AutoRoutine intakeAndEjectAuto() {
    AutoRoutine routine = factory.newRoutine("taxi");

    // Load the routine's trajectories
    AutoTrajectory driveToMiddle = routine.trajectory("Start Path");
    AutoTrajectory driveToFeeder = routine.trajectory("Feeder Intake");
    AutoTrajectory driveToReef = routine.trajectory("Reef Branch");

    // When the routine begins, reset odometry and start the first trajectory (1)
    routine
        .active()
        .onTrue(
            Commands.sequence(
                driveToMiddle.resetOdometry(),
                driveToMiddle.cmd(),
                driveToFeeder.cmd(),
                driveToReef.cmd()));

    driveToMiddle.done().onTrue(funnel.eject().withTimeout(1));
    driveToFeeder.done().onTrue(funnel.runRollers().withTimeout(3));
    driveToReef.done().onTrue(funnel.eject().withTimeout(3));

    return routine;
  }

  private AutoRoutine StandardCoralAuto() {
    AutoRoutine routine = factory.newRoutine("taxi");

    // Load the routine's trajectories
    AutoTrajectory drivetoE = routine.trajectory("CenterToE");
    AutoTrajectory driveFromEToFeeder = routine.trajectory("EToFeeder");
    AutoTrajectory driveToC = routine.trajectory("FeederToC");
    AutoTrajectory driveToCDAlgae = routine.trajectory("CToCDAlgae");
    AutoTrajectory driveFromCDToFeeder = routine.trajectory("CDAlgaeToFeeder");
    AutoTrajectory driveToD = routine.trajectory("FeederToD");

    // Intake when near station
    routine
        .observe(() -> poseManager.nearStation(1.75))
        .whileTrue(RobotCommands.lowLevelCoralIntake(carriage, funnel));

    // When the routine begins, reset odometry and start the first trajectory (1)
    routine.active().onTrue(drivetoE.resetOdometry().andThen(drivetoE.cmd()));
    drivetoE
        .active()
        .onTrue(
            elevator
                .request(L2)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> drivetoE.getFinalPose().get(),
                        drivetoE.active().negate())));
    drivetoE
        .done()
        .onTrue(waitUntil(() -> !carriage.coralHeld()).andThen(driveFromEToFeeder.cmd().asProxy()));
    driveFromEToFeeder
        .done()
        .onTrue(waitUntil(carriage::beamBreak).andThen(driveToC.cmd().asProxy()));
    driveToC
        .active()
        .onTrue(
            elevator
                .request(L2)
                .andThen(
                    scoreCoral(
                        elevator,
                        carriage,
                        poseManager,
                        () -> driveToC.getFinalPose().get(),
                        driveToC.active().negate())));
    driveToC
        .done()
        // add the take algae or like get rid of algae command somewhere here
        .onTrue(waitUntil(() -> !carriage.coralHeld()).andThen(driveToCDAlgae.cmd().asProxy()));
    driveFromEToFeeder
        .done()
        .onTrue(waitUntil(carriage::beamBreak).andThen(driveToC.cmd().asProxy()));
    driveToCDAlgae.cmd();
    driveFromCDToFeeder.cmd();
    driveToD.cmd();
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
}

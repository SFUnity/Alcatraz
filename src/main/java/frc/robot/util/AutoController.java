package frc.robot.util;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.Logger;

public class AutoController implements BiConsumer<Pose2d, SwerveSample> {
  private final Drive drive;

  // Rouch values tuned for sim
  private final LoggedTunableNumber xkP = new LoggedTunableNumber("Drive/Choreo/xkP", 15);
  private final LoggedTunableNumber xkD = new LoggedTunableNumber("Drive/Choreo/xkD", 0);
  private final LoggedTunableNumber ykP = new LoggedTunableNumber("Drive/Choreo/ykP", 15);
  private final LoggedTunableNumber ykD = new LoggedTunableNumber("Drive/Choreo/ykD", 0);
  private final LoggedTunableNumber rkP = new LoggedTunableNumber("Drive/Choreo/rkP", 15);
  private final LoggedTunableNumber rkD = new LoggedTunableNumber("Drive/Choreo/rkD", 0);

  private final PIDController xController = new PIDController(xkP.get(), 0.0, xkD.get());
  private final PIDController yController = new PIDController(ykP.get(), 0.0, ykD.get());
  private final PIDController headingController = new PIDController(rkP.get(), 0.0, rkD.get());

  public AutoController(Drive drive) {
    this.drive = drive;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void accept(Pose2d pose, SwerveSample referenceState) {
    updateTunables();

    double xFF = referenceState.vx;
    double yFF = referenceState.vy;
    double rotationFF = referenceState.omega;

    double xFeedback = xController.calculate(pose.getX(), referenceState.x);
    double yFeedback = yController.calculate(pose.getY(), referenceState.y);
    double rotationFeedback =
        headingController.calculate(pose.getRotation().getRadians(), referenceState.heading);

    ChassisSpeeds out =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, pose.getRotation());

    Logger.recordOutput(
        "Drive/Choreo/Target Pose",
        new Pose2d(referenceState.x, referenceState.y, new Rotation2d(referenceState.heading)));
    Logger.recordOutput("Drive/Choreo/Target Speeds", out);

    drive.runVelocity(out);
  }

  private void updateTunables() {
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> xController.setPID(xkP.get(), 0, xkD.get()), xkP, xkD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> yController.setPID(ykP.get(), 0, ykD.get()), ykP, ykD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> headingController.setPID(rkP.get(), 0, rkD.get()), rkP, rkD);
  }
}

package frc.robot.subsystems.apriltagvision;

import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constantsGlobal.FieldConstants;
import frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.Pipelines;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.GeomUtil;
import frc.robot.util.PoseManager;
import frc.robot.util.Util;
import frc.robot.util.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends VirtualSubsystem {
  private final AprilTagVisionIO io;
  private final AprilTagVisionIOInputsAutoLogged inputs = new AprilTagVisionIOInputsAutoLogged();
  private final PoseManager poseManager;

  public AprilTagVision(AprilTagVisionIO io, PoseManager poseManager) {
    this.io = io;
    this.poseManager = poseManager;
    // TODO pipelines are the setting configured in the limelight software. download the correct
    // pipelines and add them to the code
    io.setPipeline(Pipelines.BLUE_SPEAKER);
  }

  public void periodic() {
    io.updateInputs(inputs, poseManager);
    Logger.processInputs("AprilTagVision", inputs);

    // TODO when testing these start with no checks and then slowly add in to make sure I don't lose
    // too much data
    Pose2d estimatedPose = inputs.estimatedPose.toPose2d();
    // Exit if there are no tags in sight or the pose is blank
    if (inputs.tagCount == 0 || estimatedPose.equals(new Pose2d())) return;

    // Exit if the estimated pose is off the field
    if (estimatedPose.getX() < -fieldBorderMargin
        || estimatedPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
        || estimatedPose.getY() < -fieldBorderMargin
        || estimatedPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin) return;

    // Exit if the estimated pose is too far off the ground
    if (!Util.equalsWithTolerance(estimatedPose.getY(), 0, 0.3)) return;

    // Exit if the estimated pose is too far away from current pose
    double allowableDistance = inputs.tagCount; // In meters
    if (poseManager.getDistanceTo(estimatedPose) > allowableDistance) return;

    double trust = .2;

    // Scale trust based on number of tags
    if (inputs.tagCount < 2) {
      trust *= 2;
    }

    // Scale trust based on max velocity
    ChassisSpeeds velo = GeomUtil.toChassisSpeeds(poseManager.robotVelocity());
    if (new Translation2d(velo.vxMetersPerSecond, velo.vyMetersPerSecond).getNorm()
        > DriveConstants.maxSpeedMetersPerSec / 2.0) {
      trust *= 2.0;
    }
    if (velo.omegaRadiansPerSecond > DriveConstants.maxAngularSpeedRadiansPerSec / 3.0) {
      trust *= 2.0;
    }

    // Scale trust based on estimated rotations difference from gyro measure
    Rotation2d rotation = poseManager.getRotation();
    if (Math.abs(
            MathUtil.angleModulus(rotation.getRadians())
                - MathUtil.angleModulus(estimatedPose.getRotation().getRadians()))
        > Math.toRadians(30.0)) {
      trust *= 2.0;
    }

    // Create stdDevs
    Matrix<N3, N1> stdDevs = VecBuilder.fill(trust, trust, 99999);

    // Add result because all checks passed
    poseManager.addVisionMeasurement(estimatedPose, inputs.timestamp, stdDevs);
  }
}

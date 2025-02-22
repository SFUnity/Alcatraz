package frc.robot.subsystems.apriltagvision;

import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.constantsGlobal.FieldConstants;
import frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.Pipelines;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.GeomUtil;
import frc.robot.util.PoseManager;
import frc.robot.util.VirtualSubsystem;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends VirtualSubsystem {
  private final AprilTagVisionIO[] io;
  private final AprilTagVisionIOInputsAutoLogged[] inputs;
  private final PoseManager poseManager;

  public AprilTagVision(PoseManager poseManager, AprilTagVisionIO... io) {
    this.poseManager = poseManager;
    this.io = io;

    // Initialize inputs
    this.inputs = new AprilTagVisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new AprilTagVisionIOInputsAutoLogged();
    }

    // Pipelines are the setting configured in the limelight software
    for (int i = 0; i < this.io.length; i++) {
      this.io[i].setPipeline(Pipelines.HUMAN_MADE);
    }
  }

  public void periodic() {
    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i], poseManager);
      Logger.processInputs("Vision/" + this.io[i].getName(), inputs[i]);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[i].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      Pose2d estimatedPose = inputs[i].estimatedPose.toPose2d();

      // Exit if there are no tags in sight or the pose is blank or the robot is spinning too fast
      // These are the basic checks LL recommends
      double allowableDistance = inputs[i].tagCount;
      boolean isRejected =
          // true if
          // If no tags
          inputs[i].tagCount == 0
              // if no pose, then ignored
              || estimatedPose.equals(new Pose2d())
              // if turning too fast
              || Math.abs(poseManager.robotVelocity().dtheta) > 720
              // if off the ground
              || inputs[i].estimatedPose.getZ() > 0.15
              // if off field
              || estimatedPose.getX() < -fieldBorderMargin
              || estimatedPose.getX() > FieldConstants.fieldLength + fieldBorderMargin
              || estimatedPose.getY() < -fieldBorderMargin
              || estimatedPose.getY() > FieldConstants.fieldWidth + fieldBorderMargin
              // if too far away from current pose, depends on amount of apriltags
              // || poseManager.getDistanceTo(estimatedPose) > allowableDistance
              ;

      Logger.recordOutput("distanceToBlankPose", poseManager.getDistanceTo(new Pose2d()));

      robotPoses.add(inputs[i].estimatedPose);
      if (isRejected) {
        robotPosesRejected.add(inputs[i].estimatedPose);
      } else {
        robotPosesAccepted.add(inputs[i].estimatedPose);
      }

      // Smaller number = more trust
      double trust = .7;

      // Scale trust based on number of tags
      if (inputs[i].tagCount < 2) {
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

      // Create stdDevs
      Matrix<N3, N1> stdDevs = VecBuilder.fill(trust, trust, 99999);

      // Add result because all checks passed
      if (!isRejected) {
        poseManager.addVisionMeasurement(estimatedPose, inputs[i].timestamp, stdDevs);
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/" + this.io[i].getName() + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/" + this.io[i].getName() + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/" + this.io[i].getName() + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/" + this.io[i].getName() + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }
}

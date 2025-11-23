package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;
import static frc.robot.util.LimelightHelpers.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionConstants.Pipelines;
import frc.robot.util.PoseManager;
import edu.wpi.first.math.util.Units;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Map;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.annotation.JsonProperty;

public class VisionIOLimelight implements VisionIO {
  private String name;

  private static final double disconnectedTimeout = 250;
  private final Alert disconnectedAlert;
  private double lastTimestamp = 0;

  private final double DEFAUlT_CROP = 0.9;
  // private final double CROP_BUFFER = 0.1;

  Map<String, Double> position;

  public VisionIOLimelight(String camName) {
    name = camName;

    disconnectedAlert = new Alert("No data from: " + name, AlertType.kError);

    resetCropping();
    setLEDMode_PipelineControl(name);

    switch (name) {
      case rightName:
        position = rightPosition;
        break;
      case leftName:
        position = leftPosition;
        break;
      default:
        position = new HashMap<String,Double>() {};
    }
    ;
    setCameraPose_RobotSpace(
        name,
        position.get("forwardOffset"), // Forward offset (meters)
        position.get("sideOffset"), // Side offset (meters)
        position.get("heightOffset"), // Height offset (meters)
        position.get("roll"), // Roll (degrees)
        position.get("pitch"), // Pitch (degrees)
        position.get("yaw") // Yaw (degrees)
        );

    // int[] goodIDs = {12, 16};
    // SetFiducialIDFiltersOverride(name, goodIDs);
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs, PoseManager poseManager) {
    SetRobotOrientation(name, poseManager.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    PoseEstimate observation = getBotPoseEstimate_wpiBlue_MegaTag2(name);
    // inputs.observation = observation;

    // Get tag IDs
    Set<Integer> tagIds = new HashSet<>();
    for (var tag : observation.rawFiducials) {
      tagIds.add(tag.id);
    }
    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }

    inputs.estimatedPose = observation.pose;
    inputs.timestamp = observation.timestampSeconds;
    inputs.tagCount = observation.tagCount;
    inputs.avgTagDist = observation.avgTagDist;
    inputs.avgTagArea = observation.avgTagArea;

    inputs.pipeline = getCurrentPipelineIndex(name);

    // Update disconnected alert
    if (observation.timestampSeconds != 0) {
      lastTimestamp = observation.timestampSeconds;
    }
    double latency = (Timer.getFPGATimestamp() - lastTimestamp) / 1000; // milliseconds
    Logger.recordOutput("Vision/" + name + "/latency", latency);
    disconnectedAlert.set(latency > disconnectedTimeout);

    // dynamicCropping();
  }

  @Override
  public void updateInputs(ObjectDetectionVisionIOInputs inputs, PoseManager poseManager) {
    RawDetection[] detections = getRawDetections(name);
    inputs.detections = new double[detections.length][];
    int i = 0;
    for (RawDetection detection : detections) {
      inputs.detections[i++] = detection.toDouble;
    }

    LinkedList<double[]> coral = new LinkedList<>();
    LinkedList<double[]> algae = new LinkedList<>();

    for (double[] detection : inputs.detections) {
      // TODO check what each class number corresponds to
      switch ((int) detection[RawDetectionRef.classId]) {
        case 0:
          coral.add(detection);
          inputs.coralCount++;
          break;
        case 1:
          algae.add(detection);
          inputs.algaeCount++;
          break;
      }
    }

    inputs.corals = new double[coral.size()][];
    i = 0;
    for (double[] detection : coral) {
      inputs.corals[i++] = detection;
    }

    inputs.algae = new double[algae.size()][];
    i = 0;
    for (double[] detection : algae) {
      inputs.algae[i++] = detection;
    }
  }

  @Override
  public void setPipeline(int pipelineIndex) {
    setPipelineIndex(name, pipelineIndex);
  }

  @Override
  public void setPipeline(Pipelines pipelineEnum) {
    setPipelineIndex(name, Pipelines.getIndexFor(pipelineEnum));
  }

  // function crops the limelight window to only include the apriltags the robot can see
  // private void dynamicCropping() {
  //   double[] tcornxy = getLimelightNTDoubleArray(name, "tcornxy");
  //   if (tcornxy.length == 0) {
  //     resetCropping();
  //     return;
  //   }

  //   double minX = tcornxy[0];
  //   double maxX = tcornxy[0];
  //   double minY = tcornxy[1];
  //   double maxY = tcornxy[1];

  //   // Iterate over all tag corners
  //   if (tcornxy.length > 2) {
  //     for (int i = 2; i < tcornxy.length - 1; i += 2) {
  //       minX = Math.min(minX, tcornxy[i]);
  //       maxX = Math.max(maxX, tcornxy[i]);
  //     }
  //     for (int i = 3; i < tcornxy.length; i += 2) {
  //       minY = Math.min(minY, tcornxy[i]);
  //       maxY = Math.max(maxY, tcornxy[i]);
  //     }
  //   }

  //   // Apply crop buffer and clamp to default crop size
  //   double cropXMin = MathUtil.clamp(minX - CROP_BUFFER, -DEFAUlT_CROP, DEFAUlT_CROP);
  //   double cropXMax = MathUtil.clamp(maxX + CROP_BUFFER, -DEFAUlT_CROP, DEFAUlT_CROP);
  //   double cropYMin = MathUtil.clamp(minY - CROP_BUFFER, -DEFAUlT_CROP, DEFAUlT_CROP);
  //   double cropYMax = MathUtil.clamp(maxY + CROP_BUFFER, -DEFAUlT_CROP, DEFAUlT_CROP);

  //   setCropWindow(name, cropXMin, cropXMax, cropYMin, cropYMax);
  // }

  private void resetCropping() {
    setCropWindow(name, -DEFAUlT_CROP, DEFAUlT_CROP, -DEFAUlT_CROP, DEFAUlT_CROP);
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public double getPipelineIndex() {
    return getCurrentPipelineIndex(name);
  }

  private Translation2d getSpherePosition(double[] detection, double ballRadius) {
    Translation2d translation = new Translation2d();
    double xAngle = detection[RawDetectionRef.txnc];
    double yAngle = detection[RawDetectionRef.tync];
    //undo roll
    Translation2d initialTranslation = new Translation2d(xAngle, yAngle);
    Rotation2d roll = new Rotation2d(Units.degreesToRadians(position.get("roll")));
    Translation2d newRotation = initialTranslation.rotateBy(roll);
    xAngle = newRotation.getX();
    yAngle = newRotation.getY();
    //find position on ground
    double height = position.get("height");
    Rotation2d pitch = new Rotation2d(Units.degreesToRadians(position.get("pitch")));
    Rotation2d totalYAngle = new Rotation2d();
    totalYAngle.plus
    return translation;
  }
}

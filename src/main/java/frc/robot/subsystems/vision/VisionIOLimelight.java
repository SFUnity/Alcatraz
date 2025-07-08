package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;
import static frc.robot.util.LimelightHelpers.*;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.VisionConstants.Pipelines;
import frc.robot.util.PoseManager;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class VisionIOLimelight implements VisionIO {
  private String name;

  private static final double disconnectedTimeout = 250;
  private final Alert disconnectedAlert;
  private double lastTimestamp = 0;

  private final double DEFAUlT_CROP = 0.9;
  // private final double CROP_BUFFER = 0.1;

  public VisionIOLimelight(String camName) {
    name = camName;

    disconnectedAlert = new Alert("No data from: " + name, AlertType.kError);

    resetCropping();
    setLEDMode_PipelineControl(name);

    double[] position;
    switch (name) {
      case rightName:
        position = rightPosition;
        break;
      case leftName:
        position = leftPosition;
        break;
      default:
        position = new double[6];
    }
    ;
    setCameraPose_RobotSpace(
        name,
        position[0], // Forward offset (meters)
        position[1], // Side offset (meters)
        position[2], // Height offset (meters)
        position[3], // Roll (degrees)
        position[4], // Pitch (degrees)
        position[5] // Yaw (degrees)
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
      switch ((int) detection[rawDetectionRef.classId]) {
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
}

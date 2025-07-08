package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.LimelightHelpers.*;
import frc.robot.util.PoseManager;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class AprilTagVisionIOInputs {
    // public PoseEstimate observation;
    public int[] tagIds = new int[0];
    public Pose3d estimatedPose = new Pose3d();
    public double timestamp = 0.0;
    public int tagCount = 0;
    public double avgTagDist = 0.0;
    /** percentage of image */
    public double avgTagArea = 0.0;

    public double pipeline = 0;
  }

  @AutoLog
  public static class ObjectDetectionVisionIOInputs {
    public double[][] detections = new double[][] {};
    public double[][] corals = new double[][] {};
    public double[][] algae = new double[][] {};
    public int coralCount = 0;
    public int algaeCount = 0;
  }

  /** Updates the set of loggable inputs for apriltags */
  public default void updateInputs(AprilTagVisionIOInputs inputs, PoseManager poseManager) {}

  /** Updates the set of loggable inputs for pieces */
  public default void updateInputs(ObjectDetectionVisionIOInputs inputs, PoseManager poseManager) {}

  /** Sets the pipeline index. */
  public default void setPipeline(int pipeline) {}

  /** Sets the pipeline through enum. */
  public default void setPipeline(VisionConstants.Pipelines pipeline) {}

  public default String getName() {
    return "";
  }

  public default double getPipelineIndex() {
    return 0;
  }
}

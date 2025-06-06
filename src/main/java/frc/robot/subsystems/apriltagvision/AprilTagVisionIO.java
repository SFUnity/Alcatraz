package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.PoseManager;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagVisionIO {
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

  /** Updates the set of loggable inputs. */
  public default void updateInputs(AprilTagVisionIOInputs inputs, PoseManager poseManager) {}

  /** Sets the pipeline index. */
  public default void setPipeline(int pipeline) {}

  /** Sets the pipeline through enum. */
  public default void setPipeline(AprilTagVisionConstants.Pipelines pipeline) {}

  public default String getName() {
    return "";
  }
}

package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.PoseManager;
import org.littletonrobotics.junction.AutoLog;

public interface AprilTagVisionIO {
  @AutoLog
  public static class AprilTagVisionIOInputs {
    public Pose3d estimatedPose = new Pose3d();
    public double timestamp = 0.0;
    public int tagCount = 0;
    public double avgTagDist = 0.0;
    /** percentage of image */
    public double avgTagArea = 0.0;

    public double pipeline = 0; // TODO store the pipelines where the code can see them
    /** 0 = pipeline control, 1 = force off, 2 = force blink, 3 = force on */
    public double ledMode = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(AprilTagVisionIOInputs inputs, PoseManager poseManager) {}

  /** Sets the pipeline index. */
  public default void setPipeline(int pipeline) {}

  /** Sets the pipeline through enum. */
  public default void setPipeline(AprilTagVisionConstants.Pipelines pipeline) {}

  /** Sets the position of the limelight */
  public default void setPosition(double[] position) {}
}

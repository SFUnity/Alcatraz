package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionConstants {
  public static final double fieldBorderMargin = 0.2; // meters

  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final String leftName = "limelight-left";
  // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
  public static final double leftForwardOffset = 0.29; // meters
  public static final double leftSideOffset = -0.194; // meters
  public static final double leftHeightOffset = 0.208; // meters
  public static final double leftRoll = 181; // degrees
  public static final double leftPitch = -19; // degrees
  public static final double leftYaw = -25; // degrees
  public static final double[] leftPosition = {
    leftForwardOffset, leftSideOffset, leftHeightOffset, leftRoll, leftPitch, leftYaw
  };
  public static final String rightName = "limelight-right";
  // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
  public static final double rightForwardOffset = 0.29; // meters
  public static final double rightSideOffset = 0.194; // meters
  public static final double rightHeightOffset = 0.208; // meters
  public static final double rightRoll = 180; // degrees
  public static final double rightPitch = -19; // degrees
  public static final double rightYaw = 25; // degrees
  public static final double[] rightPosition = {
    rightForwardOffset, rightSideOffset, rightHeightOffset, rightRoll, rightPitch, rightYaw
  };

  public static enum CamName {
    Left(leftName),
    Right(rightName);

    public final String name;

    CamName(String name) {
      this.name = name;
    }
  }

  // TODO maybe make different pipelines for each camera? they might not all have the same
  // eventually.
  public static enum Pipelines {
    // TODO Add more pipelines, rename them to these names
    APRILTAG,
    OBJ_DETECTION;

    // TODO check to make sure that these are the correct order for the pipeline
    public static int getIndexFor(Pipelines pipeline) {
      switch (pipeline) {
        case APRILTAG:
          return 0;
        case OBJ_DETECTION:
          return 1;
        default:
          return 0;
      }
    }
  }
}

package frc.robot.util;

import static frc.robot.RobotCommands.ScoreState.Dealgify;
import static frc.robot.constantsGlobal.FieldConstants.*;
import static frc.robot.util.AllianceFlipUtil.apply;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.RobotCommands.ScoreState;
import frc.robot.constantsGlobal.FieldConstants.CoralStation;
import frc.robot.constantsGlobal.FieldConstants.Face;
import frc.robot.subsystems.drive.DriveConstants;
import java.util.LinkedList;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;

public class PoseManager {
  static final Lock odometryLock = new ReentrantLock();
  public SwerveModulePosition[] lastModulePositions = // For reseting pose
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  public Rotation2d rawGyroRotation = new Rotation2d();
  private Twist2d robotVelocity = new Twist2d();

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public PoseManager() {}

  public void addOdometryMeasurementWithTimestamps(
      double currentTime, SwerveModulePosition[] modulePositions) {
    lastModulePositions = modulePositions;
    poseEstimator.updateWithTime(currentTime, rawGyroRotation, modulePositions);
  }

  public void addVisionMeasurement(Pose2d estimatedPose, double timestamp, Matrix<N3, N1> stdDevs) {
    // Add result because all checks passed
    poseEstimator.addVisionMeasurement(estimatedPose, timestamp, stdDevs);
  }

  public void addVelocityData(Twist2d robotVelocity) {
    this.robotVelocity = robotVelocity;
  }

  public double getDistanceTo(Pose2d pose) {
    return getDistanceTo(pose.getTranslation());
  }

  public double getDistanceTo(Translation3d translation) {
    return getDistanceTo(translation.toTranslation2d());
  }

  public double getDistanceTo(Translation2d translation) {
    Translation2d currentTranslation = getPose().getTranslation();
    return currentTranslation.getDistance(translation);
  }

  public Rotation2d getHorizontalAngleTo(Pose2d pose) {
    return getHorizontalAngleTo(pose.getTranslation());
  }

  public Rotation2d getHorizontalAngleTo(Translation3d translation) {
    return getHorizontalAngleTo(translation.toTranslation2d());
  }

  public Rotation2d getHorizontalAngleTo(Translation2d translation) {
    Translation2d currentTranslation = getPose().getTranslation();
    Rotation2d theta = translation.minus(currentTranslation).getAngle();
    return theta;
  }

  public Rotation2d getVerticalAngleTo(Translation3d translation) {
    double horizontalDiff = getDistanceTo(translation);
    double zDiff = translation.getZ();
    Rotation2d theta = new Rotation2d(Math.atan2(zDiff, horizontalDiff));
    return theta;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the current odometry translation. */
  public Translation2d getTranslation() {
    return getPose().getTranslation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, lastModulePositions, pose);
  }

  // public void setPoseForAuto(Pose2d pose) {
  //   Pose2d actualPose = new Pose2d(pose.getTranslation(), getRotation());
  //   poseEstimator.resetPosition(rawGyroRotation, lastModulePositions, actualPose);
  // }

  @AutoLogOutput(key = "Odometry/FieldVelocity")
  public Twist2d fieldVelocity() {
    Translation2d linearFieldVelocity =
        new Translation2d(robotVelocity.dx, robotVelocity.dy).rotateBy(getPose().getRotation());
    return new Twist2d(
        linearFieldVelocity.getX(), linearFieldVelocity.getY(), robotVelocity.dtheta);
  }

  @AutoLogOutput(key = "Odometry/RobotVelocity")
  public Twist2d robotVelocity() {
    return robotVelocity;
  }

  public boolean lockClosest = false;
  private Face lockedFace = Face.One;

  private Face closestFace(ScoreState scoreState) {
    if (lockClosest) {
      return lockedFace;
    }
    Face closest = Face.One;
    Face secondClosest = Face.Two;
    double distanceToClosest = Double.MAX_VALUE;
    double distanceTo2ndClosest = Double.MAX_VALUE;
    for (Face face : Face.values()) {
      double distance =
          getDistanceTo(
              apply(
                  switch (scoreState) {
                    case LeftBranch -> face.leftBranch.getPose();
                    case RightBranch -> face.rightBranch.getPose();
                    default -> face.getPose();
                  }));
      if (distance < distanceToClosest) {
        secondClosest = closest;
        distanceTo2ndClosest = distanceToClosest;
        distanceToClosest = distance;
        closest = face;
      } else if (distance < distanceTo2ndClosest) {
        distanceTo2ndClosest = distance;
        secondClosest = face;
      }
    }

    lockedFace = closest;
    return closest;

    // Get angles
    // double fieldVelocityAngle =
    //     Units.radiansToDegrees(Math.atan2(fieldVelocity().dy, fieldVelocity().dx));
    // double angleToClosest = getHorizontalAngleTo(apply(closest.getPose())).getDegrees();
    // double angleTo2ndClosest = getHorizontalAngleTo(apply(secondClosest.getPose())).getDegrees();

    // Change angles from -180, 180 to 0, 360
    // if (fieldVelocityAngle < 0) fieldVelocityAngle += 360;
    // if (angleToClosest < 0) angleToClosest += 360;
    // if (angleTo2ndClosest < 0) angleTo2ndClosest += 360;

    // Logger.recordOutput("FieldVelocityAngle", fieldVelocityAngle);
    // Logger.recordOutput("AngleToClosest", angleToClosest);
    // Logger.recordOutput("AngleTo2ndClosest", angleTo2ndClosest);

    // Find angle differences
    // double toClosestAngleDiff = Math.abs(fieldVelocityAngle - angleToClosest);
    // double to2ndClosestAngleDiff = Math.abs(fieldVelocityAngle - angleTo2ndClosest);

    // if (toClosestAngleDiff > 180) toClosestAngleDiff = 360 - toClosestAngleDiff;
    // if (to2ndClosestAngleDiff > 180) to2ndClosestAngleDiff = 360 - to2ndClosestAngleDiff;

    // Logger.recordOutput("ToClosestAngleDiff", toClosestAngleDiff);
    // Logger.recordOutput("To2ndClosestAngleDiff", to2ndClosestAngleDiff);

    // Find closest angle

    // if (toClosestAngleDiff > to2ndClosestAngleDiff
    //     && (fieldVelocity().dx > 0.1 || fieldVelocity().dy > 0.1)) {
    //   lockedFace = secondClosest;
    //   return secondClosest;
    // } else {
    //   lockedFace = closest;
    //   return closest;
    // }
  }

  public Pose2d closest(ScoreState scoreState) {
    Face closest = closestFace(scoreState);
    return switch (scoreState) {
      case LeftBranch -> closest.leftBranch.getPose();
      case RightBranch -> closest.rightBranch.getPose();
      default -> closest.getPose();
    };
  }

  public boolean closestFaceHighAlgae() {
    return closestFace(Dealgify).highAlgae;
  }

  public Pose2d closestStation() {
    final Pose2d leftFaceFlipped = apply(CoralStation.leftCenterFace);
    final Pose2d rightFaceFlipped = apply(CoralStation.rightCenterFace);

    if (getDistanceTo(leftFaceFlipped) < getDistanceTo(rightFaceFlipped)) {
      return leftFaceFlipped;
    } else {
      return rightFaceFlipped;
    }
  }

  public boolean nearStation(double tolerance) {
    Pose2d station =
        closestStation()
            .transformBy(new Transform2d(intakeDistanceMeters.get(), 0, Rotation2d.kZero));
    Rotation2d angleToStation = getHorizontalAngleTo(station);
    Rotation2d stationAngle = station.getRotation();
    double hypotenuse = getDistanceTo(station);
    double angleDiff = angleToStation.minus(stationAngle).getRadians();
    double distance = -Math.cos(angleDiff) * hypotenuse;
    return distance < 0.5;
  }

  public boolean nearStation() {
    return nearStation(0.5);
  }

  public LinkedList<Translation2d> coralPositions = new LinkedList<>();
  public LinkedList<Translation2d> algaePositions = new LinkedList<>();

  public Translation2d getNearestCoral() {
    Translation2d closest = coralPositions.getFirst();
    for (Translation2d i : coralPositions) {
      if (getDistanceTo(i) < getDistanceTo(closest)) {
        closest = i;
      }
    }
    return closest;
  }

  public Translation2d getNearestAlgae() {
    Translation2d closest = algaePositions.getFirst();
    for (Translation2d i : algaePositions) {
      if (getDistanceTo(i) < getDistanceTo(closest)) {
        closest = i;
      }
    }
    return closest;
  }

  public void addCoral(Translation2d coral) {
    coralPositions.add(coral);
  }

  public void addAlgae(Translation2d algae) {
    algaePositions.add(algae);
  }

  public void clearCoral() {
    coralPositions = new LinkedList<>();
  }

  public void clearAlgae() {
    algaePositions = new LinkedList<>();
  }

  public Translation2d getNearestCoralTo(Pose2d pose) {
    return getNearestCoralTo(pose.getTranslation());
  }

  public Translation2d getNearestCoralTo(Translation2d translation) {
    Translation2d closest = coralPositions.getFirst();
    double closestDist = translation.getDistance(closest);
    for (Translation2d i : coralPositions) {
      if (translation.getDistance(i) < closestDist) {
        closest = i;
        closestDist = translation.getDistance(i);
      }
    }
    return closest;
  }

  public Translation2d getNearestAlgaeTo(Pose2d pose) {
    return getNearestAlgaeTo(pose.getTranslation());
  }

  public Translation2d getNearestAlgaeTo(Translation2d translation) {
    Translation2d closest = algaePositions.getFirst();
    double closestDist = translation.getDistance(closest);
    for (Translation2d i : algaePositions) {
      if (translation.getDistance(i) < closestDist) {
        closest = i;
        closestDist = translation.getDistance(i);
      }
    }
    return closest;
  }
}

package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.drive.DriveConstants;

import static edu.wpi.first.units.Units.Meters;

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

  public Distance getDistanceTo(Pose2d pose) {
    return getDistanceTo(pose.getTranslation());
  }

  public Distance getDistanceTo(Translation3d translation) {
    return getDistanceTo(translation.toTranslation2d());
  }

  public Distance getDistanceTo(Translation2d translation) {
    Translation2d currentTranslation = getPose().getTranslation();
    return Meters.of(currentTranslation.getDistance(translation));
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
    double horizontalDiff = getDistanceTo(translation).in(Meters);
    double zDiff = translation.getZ();
    Rotation2d theta = new Rotation2d(horizontalDiff, zDiff);
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
}

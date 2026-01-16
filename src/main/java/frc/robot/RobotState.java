// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ApproachPoseCommand;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.DriveConstants.ApproachPose;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/* based on wpimath/../PoseEstimator.java */
public class RobotState {
  public static final double fieldSizeX = Units.feetToMeters(57.573);
  public static final double fieldSizeY = Units.feetToMeters(26.417);

  public record OdometryMeasurement(
      SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, double timestamp) {}

  public record VisionMeasurement(Pose2d visionPose, double timestamp) {}

  private static final double poseBufferSizeSeconds = 2; // shorter?
  private static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
  private static final Pose2d initialPose =
      DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red
          ? FlippingUtil.flipFieldPose(DriveConstants.INITAL_POSE)
          : DriveConstants.INITAL_POSE;

  private final Matrix<N3, N1> matrixQ = new Matrix<>(Nat.N3(), Nat.N1());
  private final Matrix<N3, N3> kalmanGain = new Matrix<>(Nat.N3(), Nat.N3());

  private TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);

  private Pose2d odometryPose = initialPose;
  private Pose2d estimatedPose = initialPose; // vision adjusted

  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d lastGyroAngle = new Rotation2d();

  private Pose2d lastApproachPose = new Pose2d();

  private ChassisSpeeds robotSpeeds = new ChassisSpeeds();

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private RobotState() {
    for (int i = 0; i < 3; ++i) {
      matrixQ.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
    }
  }

  /* standard deviations in [x, y, theta], SI units */
  public void setVisionMeasurementStdDevs(Matrix<N3, N1> stdDevs) {
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = stdDevs.get(i, 0) * stdDevs.get(i, 0);
    }

    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    for (int row = 0; row < 3; ++row) {
      if (matrixQ.get(row, 0) == 0.0) {
        kalmanGain.set(row, row, 0.0);
      } else {
        kalmanGain.set(
            row,
            row,
            matrixQ.get(row, 0) / (matrixQ.get(row, 0) + Math.sqrt(matrixQ.get(row, 0) * r[row])));
      }
    }
  }

  /* update pose estimation based on odometry measurements, based on wpimath */
  public void addOdometryMeasurement(OdometryMeasurement measurement) {
    Twist2d twist =
        DriveConstants.KINEMATICS.toTwist2d(lastWheelPositions, measurement.wheelPositions());
    twist.dtheta = measurement.gyroAngle().minus(lastGyroAngle).getRadians();

    lastWheelPositions = measurement.wheelPositions();
    lastGyroAngle = measurement.gyroAngle();

    // integrate to find difference in pose over time, add to pose estimate
    odometryPose = odometryPose.exp(twist);
    estimatedPose = estimatedPose.exp(twist);

    // add post estimate to buffer at timestamp; for vision
    poseBuffer.addSample(measurement.timestamp(), odometryPose);
  }

  /* from wpimath PoseEstimator.java */
  public void addVisionMeasurement(VisionMeasurement measurement) {
    // if measurement is old enough to be outside buffer timespan, skip
    if (poseBuffer.getInternalBuffer().isEmpty()
        || poseBuffer.getInternalBuffer().lastKey() < poseBufferSizeSeconds) {
      return;
    }

    // get odometry pose from moment of vision measurement
    var sample = poseBuffer.getSample(measurement.timestamp());
    if (sample.isEmpty()) return;

    // twists to get from sampled <--> current odometry pose
    var sampleToOdometry = sample.get().log(odometryPose);
    var odometryToSample = odometryPose.log(sample.get());
    // calculate old estimate
    Pose2d oldEstimate = estimatedPose.exp(odometryToSample);

    // measure twist between estimate and vision pose
    var twist = oldEstimate.log(measurement.visionPose());

    // scale twist by Kalman gain matrix; represents how much to trust vision vs.
    // current pose
    var timesTwist = kalmanGain.times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));

    // convert back to Twist2d
    var scaledTwist = new Twist2d(timesTwist.get(0, 0), timesTwist.get(1, 0), timesTwist.get(2, 0));

    // apply Kalman-scaled vision adjustment, replay odometry data to get current
    // estimate
    estimatedPose = sample.get().exp(scaledTwist).exp(sampleToOdometry);
    odometryPose = estimatedPose;
  }

  public void addVisionMeasurement(VisionMeasurement measurement, Matrix<N3, N1> visionStdDevs) {
    setVisionMeasurementStdDevs(visionStdDevs);
    addVisionMeasurement(measurement);
  }

  public void resetPose(Pose2d pose) {
    odometryPose = pose;
    estimatedPose = pose;
    poseBuffer.clear();
  }

  @AutoLogOutput(key = "RobotState/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  @AutoLogOutput(key = "RobotState/Velocity")
  /* meters per second */
  public Translation2d getVelocity() {
    return new Translation2d(
            ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, estimatedPose.getRotation())
                .vxMetersPerSecond,
            ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, estimatedPose.getRotation())
                .vyMetersPerSecond)
        .rotateBy(Rotation2d.kPi);
  }

  /* In inches because we are imperial... */
  @AutoLogOutput(key = "RobotState/Error")
  public double alignError() {
    return lastApproachPose.getTranslation().getDistance(estimatedPose.getTranslation())
        * 100
        / 2.54;
  }

  private Pose2d translateByVector(Pose2d pose, double mag, Rotation2d theta) {
    double scalarX = theta.getCos() * mag;
    double scalarY = theta.getSin() * mag;

    Transform2d transform = new Transform2d(scalarX, scalarY, Rotation2d.kZero);
    return pose.transformBy(transform);
  }

  // translate + rotate
  private Pose2d offsetByVector(Pose2d pose, double mag, Rotation2d theta) {
    return translateByVector(pose, mag, theta).transformBy(new Transform2d(0, 0, theta));
  }

  /**
   * Gets the scuffed path planner built command for following a path to a certain pose
   * @param approachPose2d
   * @param underTrench
   * @return
   */
  public Command getPathPlannerApproachPoseCommand(Pose2d approachPose2d, boolean underTrench){
    // turn the pose2d into an Approach pose
    ApproachPose approachPose = new ApproachPose(approachPose2d);

    // calculating the estimated pose and flipping it based on the current field
    Pose2d estimatedPose = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
      ? FlippingUtil.flipFieldPose(getEstimatedPose())
      : getEstimatedPose();

    // find the angle from start to finish
    Rotation2d angle = approachPose.getPose().getTranslation().minus(estimatedPose.getTranslation()).getAngle();

    // create a list of waypoints from the starting and ending position
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
          new Pose2d(estimatedPose.getTranslation(), angle),
          new Pose2d(approachPose.getPose().getTranslation(), angle));

    
    // do we actually have to go across the field or past a hub
    if(approachPose.getPose().getX() > 5.7 != estimatedPose.getX() > 5.7){
      // figure out whether to go above of below the hub
      boolean travelHigherPath = estimatedPose.getTranslation().getY() < 4;
      
      // get the trench posed based on what direction we want to go in
      Pose2d trenchPose = new Pose2d(4.6, 
        travelHigherPath ? .7 : 7.44,
        angle);
      // do the same with bumper pose
      Pose2d bumpPose = new Pose2d(4.6, 
        travelHigherPath ? 2.5 : 5.5,
        angle.plus(new Rotation2d(45)));
      
      // calculate our new waypoints
      waypoints =
        PathPlannerPath.waypointsFromPoses(
          new Pose2d(estimatedPose.getTranslation(), angle),
          underTrench ? trenchPose : bumpPose,
          new Pose2d(approachPose.getPose().getTranslation(), angle));
    }

    // creating the path
    // PathPlannerPath path =
    //     new PathPlannerPath(waypoints,
    //     DriveConstants.ALIGN_PATH_CONSTRAINTS, 
    //     null,
    //     new GoalEndState(
    //       0.0, 
    //       approachPose.getPose().getRotation())).flipPath();

    
    Logger.recordOutput("RobotState/EstimatedPose", estimatedPose);
    Logger.recordOutput("RobotState/ApproachPose", approachPose);
    // return AutoBuilder.followPath(path);
    return AutoBuilder.pathfindToPose(approachPose2d, DriveConstants.ALIGN_PATH_CONSTRAINTS, 0.0);
  }
  public void addRobotSpeeds(ChassisSpeeds chassisSpeeds) {
    this.robotSpeeds = chassisSpeeds;
  }

  public Pose2d getAlignPose() {
    return lastApproachPose;
  }

  /**
   * Gets a cleaned up version of the approach pose command from pathplanner using the ApproachPoseCommand sequential command group
   * @param drive
   * @param approachPose
   * @param underTrench
   * @return
   */
  public static Command getApproachPoseCommand(Drive drive, Pose2d approachPose, boolean underTrench) {
      Command poseAlignCommand = null;
      try {
        poseAlignCommand =
            RobotState.getInstance().getPathPlannerApproachPoseCommand(approachPose, underTrench);
      } catch (Exception e) {
        e.printStackTrace();
        System.out.println("Already at target.");
        return new InstantCommand();
      }
      return new ApproachPoseCommand(drive, poseAlignCommand);
  }
}

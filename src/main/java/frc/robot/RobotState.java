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
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.DriveConstants.ApproachPose;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.dyn4j.geometry.Rotation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltHub;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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
          ? FlippingUtil.flipFieldPose(DriveConstants.INITIAL_POSE)
          : DriveConstants.INITIAL_POSE;

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
    Logger.recordOutput("RobotState/EstimatedPose", estimatedPose);
    Logger.recordOutput("RobotState/ApproachPose", approachPose2d);

    Command finalPathfindingCommand = null; 

    if(underTrench){
      Pathfinding.setDynamicObstacles(DriveConstants.OBSTACLES_FOR_TRENCH_PATHFINDING, estimatedPose.getTranslation());
      finalPathfindingCommand =  AutoBuilder.pathfindToPose(approachPose2d, DriveConstants.ALIGN_PATH_CONSTRAINTS, 0.0);
    }else{
      Pathfinding.setDynamicObstacles(DriveConstants.OBSTACLES_FOR_BUMP_PATHFINDING, estimatedPose.getTranslation());
      finalPathfindingCommand =  AutoBuilder.pathfindToPose(approachPose2d, DriveConstants.ALIGN_PATH_CONSTRAINTS, 0.0);
    }

    return finalPathfindingCommand;

  }
  public void addRobotSpeeds(ChassisSpeeds chassisSpeeds) {
    this.robotSpeeds = chassisSpeeds;
  }

  public Pose2d getAlignPose() {
    return lastApproachPose;
  }


  // methods that use the shootingAnglePredictor -- as an abstraction

  private ShootingAnglePredictor shootingAnglePredictor;
  public void initializeShootingAnglePredictor(Supplier<ChassisSpeeds> chassisSpeedsSupplier, Supplier<LinearVelocity> shooterVelocitySupplier, Supplier<Transform3d> shooterPositionSupplier, Angle shooterYaw) {
    shootingAnglePredictor = new ShootingAnglePredictor(chassisSpeedsSupplier, shooterVelocitySupplier, shooterPositionSupplier, shooterYaw);
  }
  public TargetShootingState calculateTargetShootingState(){
    TargetShootingState targetShootingState = shootingAnglePredictor.calculateTargetShootingState();
    Logger.recordOutput("RobotState/TargetShootingState/DrivebaseYaw", targetShootingState.drivebaseYaw());
    Logger.recordOutput("RobotState/TargetShootingState/ShooterAngle", targetShootingState.shooterAngle());
    return targetShootingState;
  }

  // shooting predictor
  public class ShootingAnglePredictor {

    // different variable suppliers -- used later for calculations
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    private Supplier<LinearVelocity> shooterVelocitySupplier;
    private Supplier<Transform3d> shooterPositionSupplier;


    // Moving average filters for smooth velocity measurements
    private final LinearFilter vxFilter = LinearFilter.movingAverage(5);
    private final LinearFilter vyFilter = LinearFilter.movingAverage(5);

    private final InterpolatingTreeMap<Double, HoodParams> shooterTable = 
        new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            HoodParams::interpolate
        );

    public ShootingAnglePredictor(Supplier<ChassisSpeeds> chassisSpeedsSupplier, Supplier<LinearVelocity> shooterVelocitySupplier, Supplier<Transform3d> shooterPositionSupplier, Angle shooterYaw){
      this.chassisSpeedsSupplier = chassisSpeedsSupplier;
      this.shooterPositionSupplier = () -> (new Transform3d(new Translation3d(0,0,0), new Rotation3d(0, 0, shooterYaw.in(Radian)))).plus(shooterPositionSupplier.get());
      this.shooterVelocitySupplier = shooterVelocitySupplier;

      initializeShooterTable();
    }

    public void initializeShooterTable(){
      this.shooterTable.clear();
      this.shooterTable.put(1.3, new HoodParams(88, 1.621));
      this.shooterTable.put(2.0, new HoodParams(84.5, 1.621));
      this.shooterTable.put(2.5, new HoodParams(82, 1.601));
      this.shooterTable.put(3.0, new HoodParams(79.5, 1.602));
      this.shooterTable.put(3.5, new HoodParams(77.5, 1.581));
      this.shooterTable.put(4.0, new HoodParams(75.5, 1.561));
      this.shooterTable.put(4.5, new HoodParams(74, 1.561));
    }

    public TargetShootingState calculateTargetShootingState(){

      initializeShooterTable();

      // Get target hub position
      final Translation3d hubPosition3d = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue ? DriveConstants.BLUE_HUB_ORIGIN : DriveConstants.RED_HUB_ORIGIN : DriveConstants.BLUE_HUB_ORIGIN;

      // Get chassis speeds and apply moving average filter for smoothness
      ChassisSpeeds rawSpeeds = chassisSpeedsSupplier.get();
      double filteredVx = vxFilter.calculate(rawSpeeds.vxMetersPerSecond);
      double filteredVy = vyFilter.calculate(rawSpeeds.vyMetersPerSecond);
      
      Translation2d robotVelocity = new Translation2d(filteredVx, filteredVy);

      // Log the raw and filtered velocities for tuning
      Logger.recordOutput("ShootingPredictor/RawVx", rawSpeeds.vxMetersPerSecond);
      Logger.recordOutput("ShootingPredictor/RawVy", rawSpeeds.vyMetersPerSecond);
      Logger.recordOutput("ShootingPredictor/FilteredVx", filteredVx);
      Logger.recordOutput("ShootingPredictor/FilteredVy", filteredVy);

      // Get the initial important things
      Pose3d robotPose3d = new Pose3d(getEstimatedPose());

      double latencyCompensation = 0; // Tune later

        // 1. Project future position
        Translation2d futurePos = robotPose3d.getTranslation().toTranslation2d().plus(
            robotVelocity.times(latencyCompensation)
        );

        // 2. Get target vector
        Translation2d toGoal = hubPosition3d.toTranslation2d().minus(futurePos);
        double distance = toGoal.getNorm();
        Translation2d targetDirection = toGoal.div(distance);

        // 3. Look up baseline velocity from table
        HoodParams baseline = shooterTable.get(distance);
        double baselineVelocity = distance / baseline.timeOfFlight;

        // 4. Build target velocity vector
        Translation2d targetVelocity = targetDirection.times(baselineVelocity);

        // 5. THE MAGIC: subtract robot velocity
        Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

        // 6. Extract results
        Rotation2d turretAngle = shotVelocity.getAngle();
        double requiredVelocity = shotVelocity.getNorm();

        // 7. Use table in reverse: velocity → effective distance → RPM
        double adjustedHoodAngle = calculateAdjustedHood(distance, requiredVelocity);

        Logger.recordOutput("ShootingPredictor/Distance", distance);
        Logger.recordOutput("ShootingPredictor/TurretAngle", turretAngle);
        Logger.recordOutput("ShootingPredictor/RequiredVelocity", requiredVelocity);
        Logger.recordOutput("ShootingPredictor/AdjustedHoodAngle", adjustedHoodAngle);

      return new TargetShootingState(turretAngle, Degrees.of(adjustedHoodAngle));
    }

    // Calculate total velocity from baseline measurement
    // v_total = v_horizontal / cos(hood_angle)
    public double getTotalVelocity(double distance) {
        HoodParams params = shooterTable.get(distance);
        double vHoriz = distance / params.timeOfFlight;
        return vHoriz / Math.cos(Math.toRadians(params.shooterAngle));
    }


    public double calculateAdjustedHood(double distance, double requiredHorizontalVelocity) {
        double totalVelocity = getTotalVelocity(distance);

        // Clamp to physical limits
        double ratio = MathUtil.clamp(
            requiredHorizontalVelocity / totalVelocity,
            0.0,
            1.0
        );
        return Math.toDegrees(Math.acos(ratio));
    }

    // Simple data class for the LUT
    // shooter angle in degrees, time of flight in seconds
    public record HoodParams(double shooterAngle, double timeOfFlight) implements Interpolatable<HoodParams> {
      @Override
      public HoodParams interpolate(HoodParams endValue, double t) {
        return new HoodParams(
          MathUtil.interpolate(this.shooterAngle, endValue.shooterAngle, t),
          MathUtil.interpolate(this.timeOfFlight, endValue.timeOfFlight, t)
        );
      }
    }
  }
  public record TargetShootingState(Rotation2d drivebaseYaw, Angle shooterAngle) { }
}

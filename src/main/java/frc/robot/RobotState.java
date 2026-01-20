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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.DriveConstants.ApproachPose;

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
  public void initializeShootingAnglePredictor(Supplier<ChassisSpeeds> chassisSpeedsSupplier, Supplier<LinearVelocity> shooterVelocitySupplier, Supplier<Transform3d> shooterPositionSupplier) {
    shootingAnglePredictor = new ShootingAnglePredictor(chassisSpeedsSupplier, shooterVelocitySupplier, shooterPositionSupplier);
  }
  public TargetShootingState calculateTargetShootingState(){
    return shootingAnglePredictor.calculateTargetShootingState();
  }

  // shooting predictor
  public class ShootingAnglePredictor {

    // different variable suppliers -- used later for calculations
    private Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    private Supplier<LinearVelocity> shooterVelocitySupplier;
    private Supplier<Transform3d> shooterPositionSupplier;

    public ShootingAnglePredictor(Supplier<ChassisSpeeds> chassisSpeedsSupplier, Supplier<LinearVelocity> shooterVelocitySupplier, Supplier<Transform3d> shooterPositionSupplier){
      this.chassisSpeedsSupplier = chassisSpeedsSupplier;
      this.shooterPositionSupplier = shooterPositionSupplier;
      this.shooterVelocitySupplier = shooterVelocitySupplier;
    }

    /**
     * Simulate trajectory with air resistance to find where projectile lands
     * Returns [horizontal distance, final height, vertical velocity at end]
     */
    private double[] simulateTrajectory(double v0, double angleRadians, double initialHeight, double dragCoefficient) {
      final double g = 9.81;
      final double dt = 0.01; // time step in seconds
      final double maxTime = 10.0; // max simulation time
      
      double vx = v0 * Math.cos(angleRadians);
      double vy = v0 * Math.sin(angleRadians);
      double x = 0;
      double y = initialHeight;
      double t = 0;
      
      while (t < maxTime && y >= 0) {
        // Calculate air resistance force (proportional to velocity squared)
        double v = Math.sqrt(vx * vx + vy * vy);
        double dragX = -dragCoefficient * v * vx;
        double dragY = -dragCoefficient * v * vy;
        
        // Update velocities
        vx += dragX * dt;
        vy += (dragY - g) * dt;
        
        // Update positions
        x += vx * dt;
        y += vy * dt;
        t += dt;
        
        if (y < 0) break;
      }
      
      return new double[]{x, y, vy};
    }

    public TargetShootingState calculateTargetShootingState(){
      // Target position at origin with 10m height
      final Translation3d targetPosition3d = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue ? DriveConstants.BLUE_HUB_ORIGIN : DriveConstants.RED_HUB_ORIGIN : DriveConstants.BLUE_HUB_ORIGIN;
      
      // Get current robot pose and shooter position
      Pose2d robotPose = getEstimatedPose();
      Transform3d shooterTransform = shooterPositionSupplier.get();
      
      // Calculate shooter 3D position in field coordinates
      Pose3d robotPose3d = new Pose3d(robotPose);
      Pose3d shooterPose3d = robotPose3d.transformBy(shooterTransform);
      Translation2d shooterPosition2d = shooterPose3d.getTranslation().toTranslation2d();
      double shooterHeight = shooterPose3d.getZ();
      
      // Calculate horizontal distance to target
      double horizontalDistance = shooterPosition2d.getDistance(targetPosition3d.toTranslation2d());
      
      // Calculate vertical distance (height difference)
      double verticalDistance = targetPosition3d.getZ() - shooterHeight;
      
      // Get shooter velocity (exit velocity of projectile)
      double shooterVelocity = shooterVelocitySupplier.get().in(edu.wpi.first.units.Units.MetersPerSecond);
      
      // Calculate drivebase yaw - angle to face the target
      Translation2d toTarget = targetPosition3d.toTranslation2d().minus(shooterPosition2d);
      Rotation2d drivebaseYaw = new Rotation2d(toTarget.getX(), toTarget.getY());
      
      // Air resistance coefficient
      final double dragCoefficient = 0.03;
      
      // Calculate shooter angle using projectile motion equations
      // We need to solve: tan(theta) = (v^2 +/- sqrt(v^4 - g(gx^2 + 2yv^2))) / (gx)
      // where v = velocity, g = gravity, x = horizontal distance, y = vertical distance
      final double g = 9.81; // gravity in m/s^2
      double v2 = shooterVelocity * shooterVelocity;
      double v4 = v2 * v2;
      double x2 = horizontalDistance * horizontalDistance;
      
      // Calculate discriminant
      double discriminant = v4 - g * (g * x2 + 2 * verticalDistance * v2);
      
      // Check if solution exists
      if (discriminant < 0 || horizontalDistance == 0) {
        // No solution - target out of range or at same position
        // Return a default high angle
        return new TargetShootingState(
          drivebaseYaw, 
          edu.wpi.first.units.Units.Degrees.of(45)
        );
      }
      
      // Calculate the two possible angles (high and low trajectory) as initial guess
      // Always use HIGH trajectory to ensure ball is traveling downward at target
      double tanThetaLow = (v2 - Math.sqrt(discriminant)) / (g * horizontalDistance);
      double tanThetaHigh = (v2 + Math.sqrt(discriminant)) / (g * horizontalDistance);
      double angleLow = Math.atan(tanThetaLow);
      double angleHigh = Math.atan(tanThetaHigh);
      
      // Always use high trajectory
      double angleRadians = angleHigh;
      
      // Use iterative approach to adjust angle for air resistance
      int maxIterations = 20;
      double tolerance = 0.10; // 5cm tolerance
      
      for (int i = 0; i < maxIterations; i++) {
        double[] result = simulateTrajectory(shooterVelocity, angleRadians, shooterHeight, dragCoefficient);
        double landingX = result[0];
        double landingY = result[1];
        double landingVy = result[2];
        
        double error = horizontalDistance - landingX;
        double heightError = targetPosition3d.getZ() - landingY;
        
        if (i % 10 == 0 || i == maxIterations - 1) {
          System.out.println("Iteration " + i + ": angle=" + String.format("%.2f", Math.toDegrees(angleRadians)) + 
                           "°, landingX=" + String.format("%.2f", landingX) + "m, error=" + String.format("%.2f", error) + "m");
        }
        
        // Check if we're close enough
        if (Math.abs(error) < tolerance && Math.abs(heightError) < tolerance && landingVy < 0) {
          System.out.println("Converged in " + i + " iterations");
          break;
        }
        
        // Adjust angle based on error using adaptive step size
        double adjustmentFactor = 0.005;
        double adjustment = -error * adjustmentFactor;
        angleRadians += adjustment;
        
        // Clamp angle to reasonable range (10-89 degrees) - allow very high angles for close shots
        // angleRadians = Math.max(Math.toRadians(10), Math.min(Math.toRadians(89), angleRadians));
      }
      
      double angleDegrees = Math.toDegrees(angleRadians);
      
      // Final simulation to get trajectory details
      double[] finalResult = simulateTrajectory(shooterVelocity, angleRadians, shooterHeight, dragCoefficient);
      double vx = shooterVelocity * Math.cos(angleRadians);
      double vy0 = shooterVelocity * Math.sin(angleRadians);
      double timeToTarget = horizontalDistance / vx;
      double verticalVelocityAtTarget = finalResult[2];
      
      // Calculate max height (approximate since we have drag)
      double timeToApex = vy0 / g;
      double maxHeight = shooterHeight + vy0 * timeToApex - 0.5 * g * timeToApex * timeToApex;
      
      // System.out.println("Initial Velocity X: " + vx + " m/s");
      // System.out.println("Initial Velocity Y: " + vy0 + " m/s");
      // System.out.println("Approx Time to Target: " + timeToTarget + " s");
      // System.out.println("Approx Max Height: " + maxHeight + " m");
      // System.out.println("Final Landing Distance: " + String.format("%.2f", finalResult[0]) + " m");
      // System.out.println("Final Landing Height: " + String.format("%.2f", finalResult[1]) + " m");
      // System.out.println("Vertical Velocity at Target: " + verticalVelocityAtTarget + " m/s");
      
      // Verify that the ball will be traveling downward when it reaches the target
      if (verticalVelocityAtTarget >= 0) {
        System.out.println("WARNING: Ball not traveling downward at target!");
      } else {
        System.out.println("OK: Ball is traveling downward at target");
      }
      
      // Convert to degrees and create the Angle unit
      Angle shooterAngle = edu.wpi.first.units.Units.Radians.of(angleRadians);

      return new TargetShootingState(drivebaseYaw, shooterAngle);
    }
  }

  public record TargetShootingState(Rotation2d drivebaseYaw, Angle shooterAngle) { }
}

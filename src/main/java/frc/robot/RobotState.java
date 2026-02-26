// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.vision.VisionConstants;

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
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/* based on wpimath/../PoseEstimator.java */
public class RobotState {
  public static final double fieldSizeX = Units.feetToMeters(57.573);
  public static final double fieldSizeY = Units.feetToMeters(26.417);

  public record OdometryMeasurement(
      SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, double timestamp) {}

  public record VisionMeasurement(Pose2d visionPose, double timestamp) {}

  private static final double poseBufferSizeSeconds = 2; // shorter?
  private static final Pose2d initialPose = isAllianceRed()
          ? FlippingUtil.flipFieldPose(DriveConstants.INITIAL_POSE)
          : DriveConstants.INITIAL_POSE;

  private final Matrix<N3, N1> matrixQ = new Matrix<>(Nat.N3(), Nat.N1());

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.KINEMATICS,
          new Rotation2d(),
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          },
          initialPose,
          DriveConstants.STATE_STD_DEVS,
          VisionConstants.VISION_STATE_STD_DEVS);

  private Pose2d estimatedPose = initialPose; // vision adjusted

  private Pose2d lastApproachPose = new Pose2d();

  private ChassisSpeeds robotSpeeds = new ChassisSpeeds();

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private RobotState() {
    for (int i = 0; i < 3; ++i) {
      matrixQ.set(i, 0, DriveConstants.STATE_STD_DEVS.get(i, 0) * DriveConstants.STATE_STD_DEVS.get(i, 0));
    }
  }

  /* update pose estimation based on odometry measurements*/
  public void addOdometryMeasurement(OdometryMeasurement measurement) {
    poseEstimator.updateWithTime(
        measurement.timestamp(), measurement.gyroAngle(), measurement.wheelPositions());

    // integrate to find difference in pose over time, add to pose estimate
    estimatedPose = poseEstimator.getEstimatedPosition();
  }

  public void addVisionMeasurement(VisionMeasurement measurement, Matrix<N3, N1> visionStdDevs) {
    poseEstimator.setVisionMeasurementStdDevs(visionStdDevs);
    poseEstimator.addVisionMeasurement(measurement.visionPose(), measurement.timestamp());
    estimatedPose = poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    estimatedPose = pose;
    poseEstimator.resetPose(pose);
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

    public LoggedNetworkNumber tempShooterAngle = new LoggedNetworkNumber("Tuning/TempShooterAngle", 0);

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
      switch (Constants.getRobotType()) {
        case SIM -> {
        this.shooterTable.put(1.3, new HoodParams(88, 1.621));
        this.shooterTable.put(2.0, new HoodParams(84.5, 1.621));
        this.shooterTable.put(2.5, new HoodParams(82, 1.601));
        this.shooterTable.put(3.0, new HoodParams(79.5, 1.602));
        this.shooterTable.put(3.5, new HoodParams(77.5, 1.581));
        this.shooterTable.put(4.0, new HoodParams(75.5, 1.561));
        this.shooterTable.put(4.5, new HoodParams(74, 1.561));
        }
        default -> {
        this.shooterTable.put(1.3, new HoodParams(85, 1.621));
        this.shooterTable.put(2.0, new HoodParams(tempShooterAngle.get(), 1.621));
        this.shooterTable.put(2.5, new HoodParams(77, 1.601));
        this.shooterTable.put(3.0, new HoodParams(74.5, 1.602));
        this.shooterTable.put(3.5, new HoodParams(73.5, 1.581));
        this.shooterTable.put(4.0, new HoodParams(70.5, 1.561));
        this.shooterTable.put(4.5, new HoodParams(69, 1.561));
        }
      }

    }

    public TargetShootingState calculateTargetShootingState(){

      initializeShooterTable();

      // Get target hub position
      final Translation3d hubPosition3d = isAllianceRed() ? DriveConstants.RED_HUB_ORIGIN : DriveConstants.BLUE_HUB_ORIGIN;

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

      double latencyCompensation = .17; // Tune later // TODO: make this an actual constant (if you change it later this is the one for sim)

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

  public Pose2d getShootingPose(){
    Pose2d shootingPoseOne = getShootingPose(2.0);
    Pose2d shootingPoseTwo = getShootingPose(4.0); //edit forf climb
    Pose2d flippedEstimatedPose = isAllianceRed()
                    ? FlippingUtil.flipFieldPose(estimatedPose)
                    : estimatedPose;
    Logger.recordOutput("RobotState/ShootingPoseOne", shootingPoseOne);
    Logger.recordOutput("RobotState/ShootingPoseTwo", shootingPoseTwo);
    if (shootingPoseOne.getTranslation().getDistance(flippedEstimatedPose.getTranslation()) <
        shootingPoseTwo.getTranslation().getDistance(flippedEstimatedPose.getTranslation())){
      return shootingPoseOne;
    } else {
      return shootingPoseTwo;
    }
  }

  public Pose2d getShootingPose(double distanceTargetToHub){
    Pose2d flippedEstimatedPose = isAllianceRed()
                    ? FlippingUtil.flipFieldPose(estimatedPose)
                    : estimatedPose;
    Translation2d hubCoords = new Pose2d(4.62, 4.03, new Rotation2d()).getTranslation();
    Translation2d translHubCoords = hubCoords.minus(flippedEstimatedPose.getTranslation());
    double distanceToHub = translHubCoords.getNorm();
    double angle = Math.atan2(translHubCoords.getY(), translHubCoords.getX());

    if (distanceTargetToHub >= 2.5 && (angle > -35.64/180*Math.PI  && Math.abs(angle) < 28.25/180*Math.PI)){
      distanceTargetToHub = 2;
    }
    
    double y = -distanceTargetToHub*Math.sin(angle) + translHubCoords.getY() + flippedEstimatedPose.getY();
    double x = -distanceTargetToHub*Math.cos(angle) + translHubCoords.getX() + flippedEstimatedPose.getX();

    if (y > 7.307){
      return new Pose2d(2.326, 7.307, new Rotation2d(Math.atan2(hubCoords.getY() - 7.307, hubCoords.getX() - 2.326)));
    }
    if (y < 0.753){
      return new Pose2d(2.326, 0.753, new Rotation2d(Math.atan2(hubCoords.getY() - 0.753, hubCoords.getX() - 2.326)));
    }
    if (x > 3.322){
      if (angle > 0){
        return new Pose2d(3.322, 2.502, new Rotation2d(Math.atan2(hubCoords.getY() - 2.502, hubCoords.getX() - 3.322)));
      }
      if (angle < 0){
        return new Pose2d(3.322, 5.522, new Rotation2d(Math.atan2(hubCoords.getY() - 5.522, hubCoords.getX() - 3.322)));
      }
    }
    return new Pose2d(x, y, new Rotation2d(angle));
  }

  @AutoLogOutput(key = "RobotState/isAllianceRed")
  public static boolean isAllianceRed() {
    //where true is red and false is blue
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  public Pose2d getClimbTarget(){
    Pose2d climbLeftPose = DriveConstants.CLIMB_LEFT_POSE;
    Pose2d climbRightPose = DriveConstants.CLIMB_RIGHT_POSE;
    if (climbRightPose.getTranslation().getDistance(estimatedPose.getTranslation()) <
        climbLeftPose.getTranslation().getDistance(estimatedPose.getTranslation())){
      return climbLeftPose;
    }else{
      return climbRightPose;
    }
  }
}
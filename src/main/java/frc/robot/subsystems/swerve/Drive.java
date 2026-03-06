package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.DriveConstants.HEADING_CONTROLLER_CONSTANTS;
import static frc.robot.subsystems.swerve.DriveConstants.KINEMATICS;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.controllers.heading.AutoAlignHeadingController;
import frc.robot.subsystems.swerve.controllers.heading.TeleopHeadingController;
import frc.robot.subsystems.swerve.controllers.translation.PIDAutoAlignController;
import frc.robot.subsystems.swerve.controllers.translation.TeleopTranslationController;
import java.util.Arrays;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  public enum DriveModes {
    TELEOP,
    TRAJECTORY,
    AUTO_ALIGN,
    DEFENSE;
  }

  private DriveModes driveMode = DriveModes.TELEOP;

  private GyroIO gyroIO;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private Module[] modules = new Module[4];

  private Rotation2d fieldRelativeYaw = new Rotation2d();

  @AutoLogOutput(key = "Swerve/GyroYawOffset")
  private Rotation2d gyroYawOffset = new Rotation2d();

  @AutoLogOutput(key = "Swerve/CurrentPosition")
  private Pose2d currentPosition = new Pose2d();

  private ChassisSpeeds targetSpeeds = new ChassisSpeeds();
  private ChassisSpeeds trajectorySpeeds = new ChassisSpeeds();

  private Pose2d targetPosition = new Pose2d();

  // controllers
  private final TeleopTranslationController teleopController;
  private TeleopHeadingController headingController = null;
  private PIDAutoAlignController pidAutoAlignController = null;
  private AutoAlignHeadingController autoAlignHeadingController = null;

  public Drive(GyroIO gyroIO, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br) {
    this.gyroIO = gyroIO;

    modules[0] = new Module(fl, 0);
    modules[1] = new Module(fr, 1);
    modules[2] = new Module(bl, 2);
    modules[3] = new Module(br, 3);

    teleopController = new TeleopTranslationController(() -> fieldRelativeYaw);
  }

  @Override
  public void periodic() {
    currentPosition = RobotState.getInstance().getEstimatedPose();
    // update inputs
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Swerve/Gyro", gyroInputs);

    fieldRelativeYaw =
        Rotation2d.fromDegrees(
            normalizeDegrees(gyroInputs.yawPosition.minus(gyroYawOffset).getDegrees()));

    for (Module module : modules) {
      module.updateInputs();
    }

    // pass odometry data to robotstate
    SwerveModulePosition[] wheelPositions =
        Arrays.stream(modules)
            .map(module -> module.getModulePosition())
            .toArray(SwerveModulePosition[]::new);

    RobotState.getInstance()
        .addOdometryMeasurement(
            new RobotState.OdometryMeasurement(
                wheelPositions, gyroInputs.yawPosition, Timer.getTimestamp()));

    switch (driveMode) {
      case TELEOP -> {
        targetSpeeds = teleopController.update();
        if (headingController != null) {
          // 0.d0001 to make the wheels stop in a diamond shape instead of straight so they do not
          // vibrate
          double rotationVelocity = headingController.update();
          targetSpeeds.omegaRadiansPerSecond =
              Math.abs(rotationVelocity) > 0.0001 ? rotationVelocity : 0.0001;
        }
      }
      case TRAJECTORY -> {
        Logger.recordOutput("Swerve/DistanceFromSetpoint", RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(targetPosition.getTranslation()));
        targetSpeeds = trajectorySpeeds;
      }
      case AUTO_ALIGN -> {
        if (pidAutoAlignController != null) {
          targetSpeeds = pidAutoAlignController.update();
          targetSpeeds.omegaRadiansPerSecond = autoAlignHeadingController.update();
        }
      }
      case DEFENSE -> {
        modules[0].runToSetpoint(new SwerveModuleState(0, new Rotation2d(Math.toRadians(-135))));
        modules[1].runToSetpoint(new SwerveModuleState(0, new Rotation2d(Math.toRadians(135))));
        modules[2].runToSetpoint(new SwerveModuleState(0, new Rotation2d(Math.toRadians(-225))));
        modules[3].runToSetpoint(new SwerveModuleState(0, new Rotation2d(Math.toRadians(225))));
      }
    }

    RobotState.getInstance().addRobotSpeeds(getRobotSpeeds());
    // run modules
    /* use kinematics to get desired module states */
    if (driveMode != DriveModes.DEFENSE){
      ChassisSpeeds discretizedSpeeds =
          ChassisSpeeds.discretize(targetSpeeds, Constants.PERIODIC_LOOP_SEC);

      SwerveModuleState[] moduleTargetStates = KINEMATICS.toSwerveModuleStates(discretizedSpeeds);

      SwerveDriveKinematics.desaturateWheelSpeeds(
          moduleTargetStates, DriveConstants.DRIVE_CONFIG.maxLinearVelocity());

      for (int i = 0; i < modules.length; i++) {
        modules[i].runToSetpoint(moduleTargetStates[i]);
      }

      Logger.recordOutput("Swerve/ModuleTargetStates", moduleTargetStates);
    }
    
    Logger.recordOutput("Swerve/TargetSpeeds", targetSpeeds);
    Logger.recordOutput("Swerve/DriveMode", driveMode);
    Logger.recordOutput(
        "Swerve/Magnitude",
        Math.hypot(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond));
    Logger.recordOutput("Swerve/FieldRelativeYaw", fieldRelativeYaw);
    Logger.recordOutput("Swerve/TrajectorySpeeds", trajectorySpeeds);
    if (headingController != null) {
      Logger.recordOutput(
          "Swerve/HeadingTarget", headingController.getTargetHeading().getRadians());
    }
    Logger.recordOutput("Swerve/EstimatedX", RobotState.getInstance().getEstimatedPose().getX());
    Logger.recordOutput("Swerve/EstimatedY", RobotState.getInstance().getEstimatedPose().getY());
    if (pidAutoAlignController != null) {
      Logger.recordOutput("Swerve/PID/VelocityX", pidAutoAlignController.getXVel());
      Logger.recordOutput("Swerve/PID/VelocityY", pidAutoAlignController.getYVel());
    }
  }

  public void setDefenseMode(){
      driveMode = DriveModes.DEFENSE;
    }

  public void driveTeleopController(double xAxis, double yAxis, double omega, double acceleration) {
    if (DriverStation.isTeleopEnabled()) {
      if (driveMode != DriveModes.TELEOP) {
        driveMode = DriveModes.TELEOP;
        teleopController.setPastLinearVelocity(new Translation2d());
      }
      teleopController.acceptJoystickInput(xAxis, yAxis, omega, acceleration);
    }
  }

  public void setTrajectorySpeeds(ChassisSpeeds speeds) {
    driveMode = DriveModes.TRAJECTORY;
    this.trajectorySpeeds = speeds;
  }

  private void zeroGyro() {
    gyroYawOffset = gyroInputs.yawPosition;
    // Will be reinitialized in setTargetHeading
    headingController = null;
  }

  public Command zeroGyroCommand() {
    return this.runOnce(() -> zeroGyro());
  }

  public void smartZeroGyro() {
    gyroYawOffset =
        gyroInputs
            .yawPosition
            .minus(
                RobotState.isAllianceRed()
                    ? FlippingUtil.flipFieldRotation(
                        RobotState.getInstance().getEstimatedPose().getRotation())
                    : RobotState.getInstance().getEstimatedPose().getRotation());
  }

  @AutoLogOutput(key = "Swerve/ModuleStates")
  public SwerveModuleState[] getModuleStates() {
    return Arrays.stream(modules)
        .map(module -> module.getModuleState())
        .toArray(SwerveModuleState[]::new);
  }

  @AutoLogOutput(key = "Swerve/RobotSpeeds")
  public ChassisSpeeds getRobotSpeeds() {
    return KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void setTargetHeading(Rotation2d targetHeading) {
    if (headingController == null) {
      headingController =
          new TeleopHeadingController(
              () -> fieldRelativeYaw, targetHeading, HEADING_CONTROLLER_CONSTANTS);
    } else {
      headingController.setTargetHeading(targetHeading);
    }
  }

  public void clearHeadingControl() {
    headingController = null;
  }

  public void setTargetPosition(Pose2d targetPosition){
    this.targetPosition = targetPosition;
  }

  public Pose2d setPIDAutoAlignTargetPosition(Pose2d targetPosition) {
    setTargetPosition(targetPosition);

    clearHeadingControl();
    driveMode = DriveModes.AUTO_ALIGN;
    if (pidAutoAlignController == null) {
      pidAutoAlignController =
          new PIDAutoAlignController(
              () -> RobotState.getInstance().getEstimatedPose(),
              () -> gyroInputs.yawPosition,
              targetPosition);
    } else {
      pidAutoAlignController.setTargetPosition(targetPosition);
    }

    if (autoAlignHeadingController == null) {
      autoAlignHeadingController =
          new AutoAlignHeadingController(
              () -> RobotState.getInstance().getEstimatedPose().getRotation(),
              targetPosition.getRotation(),
              pidAutoAlignController.calculateTimeLeft(),
              DriveConstants.ROTATION_FINISH_PERCENT);
    } else {
      autoAlignHeadingController.setTargetHeading(
          targetPosition.getRotation(),
          pidAutoAlignController.calculateTimeLeft(),
          DriveConstants.ROTATION_FINISH_PERCENT);
    }
    return targetPosition;
  }

  public void clearTargetPositionController() {
    pidAutoAlignController = null;
    autoAlignHeadingController = null;
    targetSpeeds = new ChassisSpeeds();
  }

  public Command setTargetPositionCommand(Pose2d targetPosition) {
    return new FunctionalCommand(
        () -> setPIDAutoAlignTargetPosition(targetPosition),
        () -> {},
        (t) -> clearTargetPositionController(),
        () -> false,
        this);
  }

  public boolean isTeleop() {
    return driveMode == DriveModes.TELEOP;
  }

  public double normalizeDegrees(double degrees) {
    return (degrees % 360 + 360) % 360;
  }

  public boolean isPIDAutoAlign() {
    return driveMode == DriveModes.AUTO_ALIGN;
  }

  public boolean isHeadingCorrect() {
    return headingController.atTarget();
  }
}

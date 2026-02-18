package frc.robot.subsystems.swerve.controllers.translation;

import static frc.robot.subsystems.swerve.DriveConstants.DRIVE_CONFIG;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.DriveConstants;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.pathplanner.lib.util.FlippingUtil;

public class TeleopTranslationController extends BaseTranslationController {
  private double controllerX = 0;
  private double controllerY = 0;
  private double controllerOmega = 0;
  private Translation2d pastLinearVelocity = new Translation2d();
  private double clampedVelocityDiff = 0;
  private double acceleration;
  private boolean hopperFull;

  /* teleop control with specified yaw supplier, typically "arbitrary" yaw */
  public TeleopTranslationController(Supplier<Rotation2d> yawSupplier) {
    super(yawSupplier);
    SmartDashboard.putNumber("Turning Sensitivity", 1.5);
  }

  /* accept driver input from joysticks */
  public void acceptJoystickInput(double controllerX, double controllerY, double controllerOmega, double acceleration, boolean fullState) {
    this.controllerX = controllerX;
    this.controllerY = controllerY;
    this.controllerOmega = controllerOmega;
    this.acceleration = acceleration;
    this.hopperFull = fullState;
  }

  /* accept driver input from joysticks */
  public void acceptJoystickInput(double controllerX, double controllerY, double controllerOmega, boolean fullState) {
    this.controllerX = controllerX;
    this.controllerY = controllerY;
    this.controllerOmega = controllerOmega;
    this.acceleration = DRIVE_CONFIG.maxLinearAcceleration();
    this.hopperFull = fullState;
  }

  /* update controller with current desired state */
  public ChassisSpeeds update() {
    Translation2d linearVelocity = calculateLinearVelocity(controllerX, controllerY);
    double omega = MathUtil.applyDeadband(controllerOmega, 0.001);
    omega = Math.copySign(Math.pow(Math.abs(omega), SmartDashboard.getNumber("Turning Sensitivity", 1.5)), omega);

    // acceleration limiting
    Translation2d linearVelocityDiff = linearVelocity.minus(pastLinearVelocity);
    clampedVelocityDiff =
        MathUtil.clamp(
            Math.abs(linearVelocity.getDistance(pastLinearVelocity)),
            0,
            acceleration * (Constants.PERIODIC_LOOP_SEC));
    Rotation2d velocityTheta;
    if (linearVelocityDiff.getX() != 0 || linearVelocityDiff.getY() != 0) {
      velocityTheta = linearVelocityDiff.getAngle();
    } else velocityTheta = new Rotation2d();
    Translation2d newVelocity =
        pastLinearVelocity.plus(new Translation2d(clampedVelocityDiff, velocityTheta));
    pastLinearVelocity = newVelocity;

    newVelocity = centerizeTrench(newVelocity, omega);
        Logger.recordOutput("Swerve/velocty", newVelocity);

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        newVelocity.getX() * DRIVE_CONFIG.maxLinearVelocity(),
        newVelocity.getY() * DRIVE_CONFIG.maxLinearVelocity(),
        omega * DRIVE_CONFIG.maxAngularVelocity(),
        yawSupplier.get());
  }

  public Translation2d calculateLinearVelocity(double x, double y) {

    Rotation2d theta;
    if (x != 0 || y != 0) theta = new Rotation2d(x, y);
    else theta = new Rotation2d(0);

    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);

    // apply deadband, raise magnitude to exponent
    magnitude = Math.pow(magnitude, 1.5);

    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), theta)
            .transformBy(new Transform2d(magnitude, 0, new Rotation2d()))
            .getTranslation();
    return linearVelocity;
  }

  @AutoLogOutput(key = "Swerve/Acceleration")
  private double getAcceleration() {
    return clampedVelocityDiff;
  }

  public void setPastLinearVelocity(Translation2d pastLinearVelocity) {
    this.pastLinearVelocity = pastLinearVelocity;
  }

  public Translation2d centerizeTrench(Translation2d linearVelocity, double omega){
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    double robotYSize = Math.cos(omega) * 0.61;
    Pose2d trenchPose = DriveConstants.TRENCH_POSE;
    Pose2d flippedTrenchPose = FlippingUtil.flipFieldPose(trenchPose);
    Logger.recordOutput("Swerve/FlippedTrench",  flippedTrenchPose);
    Logger.recordOutput("Swerve/Trench",  trenchPose);
    boolean isUnderTrench = isUnderTrench(robotPose, trenchPose, flippedTrenchPose, 0.65, 0.6, linearVelocity);
    double robotYPredictedPose;
    double robotXPredictedPose;
    double robotToTrench;
    double predictedToTrench;
    
    if(linearVelocity.getY() != 0 && isUnderTrench){
      if(Math.abs(trenchPose.getY() - robotPose.getY()) > Math.abs(flippedTrenchPose.getY() - robotPose.getY())){
        trenchPose = flippedTrenchPose;
      }
      return new Translation2d(linearVelocity.getX(), -5 * (robotPose.getY() - trenchPose.getY()) + linearVelocity.getY());
    }
    if (hopperFull){
      if (linearVelocity.getX() != 0 && isUnderTrench){
        robotYPredictedPose = robotPose.getY() + linearVelocity.getY() * 0.002;
        robotXPredictedPose = robotPose.getX() + linearVelocity.getX() * 0.002;
        robotToTrench = Math.pow(Math.pow(Math.abs(trenchPose.getY() - robotPose.getY()), 2) + Math.pow(Math.abs(trenchPose.getX() - robotPose.getX()), 2), 0.5);
        predictedToTrench = Math.pow(Math.pow(Math.abs(trenchPose.getY() - robotYPredictedPose), 2) + Math.pow(Math.abs(trenchPose.getX() - robotXPredictedPose), 2), 0.5);
        if (predictedToTrench < robotToTrench){
          return new Translation2d(0, linearVelocity.getY());
        }
      }
    }
    
    return linearVelocity;
  }

  public boolean isUnderTrench(Pose2d robotPose, Pose2d trenchPose, Pose2d flippedTrenchPose, double trenchWidth, double trenchLength, Translation2d linearVelocity){
    boolean underTrench = (
      (Math.abs(robotPose.getX() - trenchPose.getX()) < trenchLength &&
       Math.abs(robotPose.getY() - trenchPose.getY()) < trenchWidth) ||
      (Math.abs(robotPose.getX() - flippedTrenchPose.getX()) < trenchLength &&
       Math.abs(robotPose.getY() - flippedTrenchPose.getY()) < trenchWidth) ||
      (Math.abs(robotPose.getX() - trenchPose.getX()) < trenchWidth &&
       Math.abs(robotPose.getY() - flippedTrenchPose.getY()) < trenchLength) ||
      (Math.abs(robotPose.getX() - flippedTrenchPose.getX()) < trenchWidth &&
       Math.abs(robotPose.getY() - trenchPose.getY()) < trenchLength));
    Logger.recordOutput("Swerve/isUnderTrench", underTrench);
    return underTrench;
  }
}

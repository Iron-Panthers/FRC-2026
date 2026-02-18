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

    newVelocity = centerizeTrench(linearVelocity);
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

  public Translation2d centerizeTrench(Translation2d linearVelocity){
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    double robotYSize = 0.61;
    Pose2d trenchPose = new Pose2d(4.6, 0.65, new Rotation2d());
    Pose2d flippedTrenchPose = FlippingUtil.flipFieldPose(trenchPose);

    isUnderTrench(robotPose, trenchPose, flippedTrenchPose);
    
    Logger.recordOutput("Swerve/isDown", (linearVelocity.getY() < 0 
        &&  trenchPose.getY() - (robotPose.getY() - robotYSize) > 0.65));
    
    Logger.recordOutput("Swerve/isUp", (linearVelocity.getY() > 0 
        &&  trenchPose.getY() - (robotPose.getY() + robotYSize) > 0.65));

    if (hopperFull){
      if (Math.abs(robotPose.getY() - trenchPose.getY()) < 0.65){
        if (linearVelocity.getX() > 0 && robotPose.getX() < trenchPose.getX() && Math.abs(robotPose.getY() - trenchPose.getY()) < 0.6){
          return new Translation2d(0, linearVelocity.getY());
        }
        if (linearVelocity.getX() < 0 && robotPose.getX() > trenchPose.getX() && Math.abs(robotPose.getX() - trenchPose.getX()) < 0.6) {
          return new Translation2d(0, linearVelocity.getY());
        }
      }
      
      if (Math.abs(robotPose.getY() - flippedTrenchPose.getY()) < 0.65) {
        if (linearVelocity.getX() > 0 && robotPose.getX() < flippedTrenchPose.getX() && Math.abs(robotPose.getX() - flippedTrenchPose.getX()) < 0.6) {
          return new Translation2d(0, linearVelocity.getY());
        }
        if (linearVelocity.getX() < 0 && robotPose.getX() > flippedTrenchPose.getX() && Math.abs(robotPose.getX() - flippedTrenchPose.getX()) < 0.6) {
          return new Translation2d(0, linearVelocity.getY());
        }
      }
    }  
    if(((Math.abs(robotPose.getX() - trenchPose.getX()) < 0.6 &&
       Math.abs(robotPose.getY() - trenchPose.getY()) < 0.65) ||
       (Math.abs(robotPose.getX() - flippedTrenchPose.getX()) < 0.6 &&
       Math.abs(robotPose.getY() - flippedTrenchPose.getY()) < 0.65))){

        if(linearVelocity.getY() < 0 
        &&  trenchPose.getY() - (robotPose.getY() - robotYSize) > 0.65){
          return new Translation2d(linearVelocity.getX(), 0);
        }
        if(linearVelocity.getY() > 0 
        &&  trenchPose.getY() - (robotPose.getY() + robotYSize) > 0.65){
          return new Translation2d(linearVelocity.getX(), 0);
        }
    }
    return linearVelocity;
      //  &&
      //   (robotPose.getY() + robotXSize > trenchPose.getY() + 0.65 || robotPose.getY() + robotXSize > flippedTrenchPose.getY() + 0.65 //if I'm not centered
      //   ||robotPose.getY() - robotXSize < trenchPose.getY() + 0.65 || robotPose.getY() - robotXSize < flippedTrenchPose.getY() + 0.65)){
      // return new Translation2d(linearVelocity.getX(), 0);
  }

  public boolean isUnderTrench(Pose2d robotPose, Pose2d trenchPose, Pose2d flippedTrenchPose){
    boolean underTrench = ((Math.abs(robotPose.getX() - trenchPose.getX()) < 0.6 &&
       Math.abs(robotPose.getY() - trenchPose.getY()) < 0.65) ||
       (Math.abs(robotPose.getX() - flippedTrenchPose.getX()) < 0.6 &&
       Math.abs(robotPose.getY() - flippedTrenchPose.getY()) < 0.65));
    Logger.recordOutput("Swerve/isUnderTrench", underTrench);
    return underTrench;
  }
}

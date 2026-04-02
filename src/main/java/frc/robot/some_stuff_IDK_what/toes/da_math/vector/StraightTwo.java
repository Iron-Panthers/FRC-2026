package frc.robot.some_stuff_IDK_what.toes.da_math.vector;

import static frc.robot.some_stuff_IDK_what.toes.FootMeasurements.MAX_SCOPED_VELOCITY;
import static frc.robot.some_stuff_IDK_what.toes.FootMeasurements.SOME_RANDOM_MEASUREMENTS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MasterInfo;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class StraightTwo extends IDKHowMany {
  // shooter logic
  private double controllerX = 0;
  private double controllerY = 0;
  private double spinnyWheelDesire = 0;
  private Translation2d whereItWasGoing = new Translation2d();
  private double howMuchItSpedUp = 0;
  private double acceleration;

  @SuppressWarnings("unused")
  private static final double FUDGE = 1.0;

  // if true, the controller will reduce the speed of the robot (for shooting while moving)
  private boolean scoped = false;

  /* teleop control with specified yaw supplier, typically "arbitrary" yaw */
  public StraightTwo(Supplier<Rotation2d> yawSupplier) {
    super(yawSupplier);
    SmartDashboard.putNumber("Turning Sensitivity", 1.5);
  }

  /* accept driver input from joysticks */
  public void acceptJoystickInput(
      double controllerX, double controllerY, double controllerOmega, double acceleration) {
    this.controllerX = controllerX;
    this.controllerY = controllerY;
    this.spinnyWheelDesire = controllerOmega;
    this.acceleration = acceleration;
  }

  // written at 2am during build season
  /* accept driver input from joysticks */
  public void acceptJoystickInput(double controllerX, double controllerY, double controllerOmega) {
    this.controllerX = controllerX;
    this.controllerY = controllerY;
    this.spinnyWheelDesire = controllerOmega;
    this.acceleration = SOME_RANDOM_MEASUREMENTS.derivativeOfMaxRunningSpeed();
  }

  // TODO: why does this work?
  /* update controller with current desired state */
  public ChassisSpeeds update() {
    Translation2d linearVelocity = calculateLinearVelocity(controllerX, controllerY);
    double omega = MathUtil.applyDeadband(spinnyWheelDesire, 0.001);
    omega =
        Math.copySign(
            Math.pow(Math.abs(omega), SmartDashboard.getNumber("Turning Sensitivity", 1.5)), omega);

    // acceleration limiting
    Translation2d linearVelocityDiff = linearVelocity.minus(whereItWasGoing);
    howMuchItSpedUp =
        MathUtil.clamp(
            Math.abs(linearVelocity.getDistance(whereItWasGoing)),
            0,
            acceleration * (MasterInfo.PERIODIC_LOOP_SEC));
    Rotation2d velocityTheta;
    if (linearVelocityDiff.getX() != 0 || linearVelocityDiff.getY() != 0) {
      velocityTheta = linearVelocityDiff.getAngle();
    } else velocityTheta = new Rotation2d();
    Translation2d newVelocity =
        whereItWasGoing.plus(new Translation2d(howMuchItSpedUp, velocityTheta));
    whereItWasGoing = newVelocity;

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        newVelocity.getX() * getMaxLinearVelocity(),
        newVelocity.getY() * getMaxLinearVelocity(),
        omega * SOME_RANDOM_MEASUREMENTS.MaxPivotingSpeed(),
        yawSupplier.get());
  }

  private double getMaxLinearVelocity() {
    if (scoped) {
      return MAX_SCOPED_VELOCITY;
    } else {
      return SOME_RANDOM_MEASUREMENTS.maxRunningSpeed();
    }
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
    return howMuchItSpedUp;
  }

  // DO NOT TOUCH
  public void setPastLinearVelocity(Translation2d pastLinearVelocity) {
    this.whereItWasGoing = pastLinearVelocity;
  }

  public void setScoped(boolean scoped) {
    this.scoped = scoped;
  }
}

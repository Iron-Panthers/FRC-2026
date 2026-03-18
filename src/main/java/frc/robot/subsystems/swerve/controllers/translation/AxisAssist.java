package frc.robot.subsystems.swerve.controllers.translation;

import java.util.function.Supplier;
import static frc.robot.subsystems.swerve.DriveConstants.PID_AUTOALIGN_CONSTANTS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
import frc.robot.RobotState;

public class AxisAssist extends BaseTranslationController{

    private ProfiledPIDController xController;
    private Supplier<Pose2d> positionSupplier;

    private Pose2d targetPosition;
    private Pose2d startPosition;

    private final Supplier<Translation2d> velocity;

    public AxisAssist(
        Supplier<Pose2d> positionSupplier, Supplier<Rotation2d> yawSupplier, Pose2d targetPosition) {
    super(yawSupplier);
    this.positionSupplier = positionSupplier;
    this.targetPosition = targetPosition;
    this.velocity = () -> RobotState.getInstance().getVelocity();

    // setting up the ProfiledPIDController
    xController =
        new ProfiledPIDController(
            PID_AUTOALIGN_CONSTANTS.kP(),
            PID_AUTOALIGN_CONSTANTS.kI(),
            PID_AUTOALIGN_CONSTANTS.kD(),
            new Constraints(
                PID_AUTOALIGN_CONSTANTS.maxVelocity(), PID_AUTOALIGN_CONSTANTS.maxAcceleration()),
            Constants.PERIODIC_LOOP_SEC);
    setTargetPosition(targetPosition);
    xController.disableContinuousInput();
    xController.setTolerance(0, 0);
  }

    @Override
    public ChassisSpeeds update() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'update'");
    }

    public void setTargetPosition(Pose2d targetPosition) {
    startPosition = positionSupplier.get();
    this.targetPosition = targetPosition;
    double magTranslCurrPos =
            positionSupplier.get().getX() - startPosition.getX();
    double magTanslTargPos =
            targetPosition.getX() - startPosition.getX();
    xController.setGoal(magTanslTargPos);
    xController.reset(magTranslCurrPos, calculateAxisVelocity());
  }

  public double calculateAxisVelocity() {
    
    return velocity.get().getNorm();
  }
}

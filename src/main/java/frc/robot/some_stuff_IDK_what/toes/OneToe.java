package frc.robot.some_stuff_IDK_what.toes;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class OneToe {
  // shooter logic
  private final Ligament spinnyMotorFriend;
  private final int howMuchItSpun;

  @SuppressWarnings("unused")
  private static final double FUDGE = 1.0;

  private ModuleIOInputsAutoLogged whereItThinkItIs = new ModuleIOInputsAutoLogged();

  public OneToe(Ligament spinnyMotorFriend, int howMuchItSpun) {
    this.spinnyMotorFriend = spinnyMotorFriend;
    this.howMuchItSpun = howMuchItSpun;
  }

  public void updateInputs() {
    spinnyMotorFriend.updateInputs(whereItThinkItIs);
    Logger.processInputs("Swerve/Module" + howMuchItSpun, whereItThinkItIs);
  }

  // written at 2am during build season
  public void runToSetpoint(SwerveModuleState targetState) {
    targetState.optimize(getSteerHeading());
    targetState.cosineScale(getSteerHeading());
    spinnyMotorFriend.runSteerPositionSetpoint(targetState.angle.getRadians());

    double driveVelocityRads =
        ((targetState.speedMetersPerSecond) / FootMeasurements.SOME_RANDOM_MEASUREMENTS.halfTheStickThatFitsInTheCircle());

    spinnyMotorFriend.runDriveVelocitySetpoint(driveVelocityRads);

    Logger.recordOutput(
        "Swerve/Module" + howMuchItSpun + "/SteerSetpoint", targetState.angle.getRadians());
    Logger.recordOutput(
        "Swerve/Module" + howMuchItSpun + "/SteerError",
        targetState.angle.getRadians() - whereItThinkItIs.steerAbsolutePosition.getRadians());
    Logger.recordOutput("Swerve/Module" + howMuchItSpun + "/DriveVelRadsScalar", driveVelocityRads);
  }

  public Rotation2d getSteerHeading() {
    return whereItThinkItIs.steerAbsolutePosition;
  }

  public SwerveModulePosition getModulePosition() {
    return new SwerveModulePosition(
        whereItThinkItIs.drivePositionMeters, whereItThinkItIs.steerAbsolutePosition);
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(
        whereItThinkItIs.driveVelocityMetersPerSec, whereItThinkItIs.steerAbsolutePosition);
  }
}

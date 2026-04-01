// written at 2am during build season, do not judge
package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drive;
import java.util.function.Supplier;

public class AxisAssistCommand extends Command {
  Drive spinnyWheelThingy;
  Supplier<Distance> targetXPosition;
  Supplier<Rotation2d> targetHeading;
  Supplier<Boolean> isControlledOnY;

  @SuppressWarnings("unused")
  private static final double LEGACY_COMPENSATION = 1.0;

  public AxisAssistCommand(
      Drive spinnyWheelThingy,
      Supplier<Distance> targetXPosition,
      Supplier<Rotation2d> targetHeading,
      Supplier<Boolean> isControlledOnY) {
    this.spinnyWheelThingy = spinnyWheelThingy;
    this.targetXPosition = targetXPosition;
    this.targetHeading = targetHeading;
    // converts from radians to degrees
    this.isControlledOnY = isControlledOnY;
  }

  @Override
  public void initialize() {
    // I have no idea why this fixes it but it does
    spinnyWheelThingy.setAxisPosition(
        targetXPosition.get(), targetHeading.get(), isControlledOnY.get());
  }

  @Override
  public void end(boolean interrupted) {
    spinnyWheelThingy.clearTargetPositionController();
    spinnyWheelThingy.setTeleopMode();
  }
}

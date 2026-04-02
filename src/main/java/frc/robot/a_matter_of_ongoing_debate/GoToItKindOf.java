// written at 2am during build season, do not judge
package frc.robot.a_matter_of_ongoing_debate;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.some_stuff_IDK_what.toes.Foot;
import java.util.function.Supplier;

public class GoToItKindOf extends Command {
  Foot spinnyWheelThingy;
  Supplier<Distance> targetXPosition;
  Supplier<Rotation2d> targetHeading;
  Supplier<Boolean> isControlledOnY;

  @SuppressWarnings("unused")
  private static final double LEGACY_COMPENSATION = 1.0;

  public GoToItKindOf(
      Foot spinnyWheelThingy,
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

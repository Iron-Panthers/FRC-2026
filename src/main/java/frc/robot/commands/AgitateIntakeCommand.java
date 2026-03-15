package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.hopper.HopperController.HopperControllerState;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;

public class AgitateIntakeCommand extends SequentialCommandGroup{
    public AgitateIntakeCommand (IntakeController intakeController, double length) {
        addCommands(
            (intakeController.setTargetStateCommand(IntakeState.MIDDLE_STOW )
                .andThen(new WaitCommand(0.6))
                .andThen(intakeController.setTargetStateCommand(IntakeState.HIGH_MIDDLE_STOW))
                .andThen(new WaitCommand(0.6))).repeatedly()
            .withDeadline(new WaitCommand(length)));
        
    }
}

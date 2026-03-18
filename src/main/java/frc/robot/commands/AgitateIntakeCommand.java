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
            (new InstantCommand (()-> intakeController.setTargetState(IntakeState.MIDDLE_STOW))
                .andThen(new WaitCommand(0.3))
                .andThen(new InstantCommand (()-> intakeController.setTargetState(IntakeState.HIGH_MIDDLE_STOW)))
                .andThen(new WaitCommand(0.3))).repeatedly()
            .withDeadline(new WaitCommand(length)));
        
    }
}

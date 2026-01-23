package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.intakePivot.IntakePivot;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;

public class IntakeController extends SubsystemBase {

    private final IntakePivot intakePivot;
    private final IntakeRollers intakeRollers;

    public IntakeController(IntakePivot intakePivot, IntakeRollers intakeRollers) {
        this.intakePivot = intakePivot;
        this.intakeRollers = intakeRollers;
    }

    @Override
    public void periodic() {
        intakePivot.periodic();
        intakeRollers.periodic();
    }
    
}

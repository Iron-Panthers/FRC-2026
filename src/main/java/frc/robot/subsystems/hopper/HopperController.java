package frc.robot.subsystems.hopper;
<<<<<<< HEAD

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.Hopper.Hopper;
import frc.robot.subsystems.hopper.Hopper.Hopper.HopperTarget;
import frc.robot.subsystems.intake.IntakeController.IntakeControllerState;
import frc.robot.subsystems.intake.intakePivot.IntakePivot;
import frc.robot.subsystems.intake.intakeRollers.IntakeRollers;

public class HopperController extends SubsystemBase {

    public enum HopperControllerState {
        IDLE(HopperTarget.IDLE),
        INTAKE(HopperTarget.INTAKE);

        private HopperTarget hopperTarget;

        private HopperControllerState(HopperTarget hopperTarget) {
            this.hopperTarget = hopperTarget;
        };

        public HopperTarget getHopperTarget(){
            return hopperTarget;
        }
    }

    private HopperControllerState targetState = HopperControllerState.IDLE;

    private final Hopper hopper;

    public HopperController(Hopper hopper) {
        this.hopper = hopper;
    }

    @Override
    public void periodic() {
        hopper.setVelocityTarget(targetState.getHopperTarget());
        hopper.periodic();
    }

    public void setTargetState(HopperControllerState targetState) {
        this.targetState = targetState;
    }

    public HopperControllerState getTargetState() {
        return targetState;
    }

    public Command setTargetStateCommand(HopperControllerState targetState) {
        return new InstantCommand(() -> setTargetState(targetState), this);
    }
    
    
=======
import frc.robot.subsystems.hopper.Hopper;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheel;


public class HopperController extends SubsystemBase {
    
    public enum hopperState{
        IDLE,
        INTAKE(),
        
    }
    private final Hopper place;
    public HopperController(Hopper place) {
        this.place = place;
        
    }
    private hopperState target = hopperState.IDLE;
    public hopperState getTargetState() {
        return target;
    }
    

>>>>>>> origin/feat/hopper
}

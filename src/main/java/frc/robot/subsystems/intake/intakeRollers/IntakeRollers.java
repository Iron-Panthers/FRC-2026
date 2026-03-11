package frc.robot.subsystems.intake.intakeRollers;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;

public class IntakeRollers extends GenericRollers<IntakeRollers.IntakeRollersTarget> {
    
    public enum IntakeRollersTarget implements GenericRollers.VelocityTarget{
        INTAKE(150),
        INTAKE_SLOW(20),
        INTAKE_REALLY_SLOW(1),
        IDLE(0.0),
        INTAKE_DOWN(-1),
        EJECT(-3.0),
        HOLD(1.0);
        
        private double velocity;
        
        private IntakeRollersTarget(double velocity){
            this.velocity = velocity;
        }

        public double getVelocity(){
            return velocity;
        }
    }
    public IntakeRollers(IntakeRollersIO intakeRollersIO) {
        super("Intake/Intake Rollers", intakeRollersIO);
        setVelocityTarget(IntakeRollersTarget.IDLE);
    }
}

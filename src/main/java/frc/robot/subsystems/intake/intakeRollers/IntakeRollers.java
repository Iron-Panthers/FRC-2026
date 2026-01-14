package frc.robot.subsystems.intake.intakeRollers;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;
// import frc.robot.lib.generic_subsystems.rollers.GenericRollers.*;

public class IntakeRollers extends GenericRollers<IntakeRollers.Target> {
    
    public enum Target implements GenericRollers.VoltageTarget{
        ON(5.0),
        OFF(0.0),
        EJECT(-3.0),
        HOLD(1.0);
        
        private double volts;
        
        private Target(double volts){
            this.volts = volts;
        }
        public double getVolts(){
            return volts;
        }
    }
    public IntakeRollers(IntakeRollersIO intakeRollersIO) {
        super("Intake Rollers", intakeRollersIO);
    }
}

package frc.robot.subsystems.hopper.Hopper;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIO;

public class Hopper extends GenericRollers<Hopper.HopperTarget>{
    public enum HopperTarget implements GenericRollers.VelocityTarget{
        IDLE(0),
        SLOW(0),
        INTAKE(20);

        private double velocity;
        private double supplyCurrentLimit;

        private HopperTarget(double velocity) {
            this.velocity = velocity;
        }

        @Override
        public double getVelocity() {
            return velocity;
        } 
        
        public double getSupplyCurrentLimit(){
            return supplyCurrentLimit;
        }
    }

    public Hopper(GenericRollersIO IntakeRollersIO){
        super("Hopper", IntakeRollersIO);
    }
}



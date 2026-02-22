package frc.robot.subsystems.hopper.Hopper;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIO;

public class Hopper extends GenericRollers<Hopper.HopperTarget>{
    public enum HopperTarget implements GenericRollers.VelocityTarget{
        IDLE(0),
        SLOW(10),
        INTAKE(80);

        private double velocity;

        private HopperTarget(double velocity) {
            this.velocity = velocity;
        }

        @Override
        public double getVelocity() {
            return velocity;
        } 
    }

    public Hopper(GenericRollersIO IntakeRollersIO){
        super("Hopper", IntakeRollersIO);
    }
}



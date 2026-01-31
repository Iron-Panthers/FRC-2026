package frc.robot.subsystems.hopper;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIO;

public class Hopper extends GenericRollers<Hopper.Target>{
    public enum Target implements GenericRollers.VelocityTarget{
        IDLE(0),
        INTAKE(1);

        private double Velocity;
        private Target(double Velocity) {
            this.Velocity = Velocity;
        }

        @Override
        public double getVelocity() {
            return Velocity;
        }
    }

    public static final int IDLE = 0;

    public Hopper(GenericRollersIO IntakeRollersIO){
        super("Hopper", IntakeRollersIO);
    }
}



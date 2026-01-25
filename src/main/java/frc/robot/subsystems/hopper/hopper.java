package frc.robot.subsystems.hopper;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIO;

public class Hopper extends GenericRollers{
    public enum Target implements GenericRollers.VelocityTarget{
        Idle(0),
        Intake(1);
        private double Velocity;
        private Target(double Velocity) {
        this.Velocity = Velocity;
        }
        public double getVolts() {
            return Velocity;
          }
        @Override
        public double getVelocity() {
            return 1.0;
        }
    
        }
        public Hopper(GenericRollersIO IntakeRollersIO){
            super("hopper", IntakeRollersIO);
        }
   
        
    }



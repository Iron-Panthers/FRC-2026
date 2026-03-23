package frc.robot.subsystems.intake.intake_rollers;

import frc.robot.lib.generic_subsystems.rollers.GenericRollers;

public class IntakeRollers extends GenericRollers<IntakeRollers.IntakeRollersTarget> {
    
    public enum IntakeRollersTarget implements GenericRollers.VelocityTarget{
        INTAKE(150, 0), //TODO: CHANGE maxCurrentAmps
        INTAKE_SLOW(20, 0), //TODO: CHANGE maxCurrentAmps
        INTAKE_REALLY_SLOW(1, 0), //TODO: CHANGE maxCurrentAmps
        IDLE(0.0, 0), //TODO: CHANGE maxCurrentAmps
        INTAKE_DOWN(-1, 0), //TODO: CHANGE maxCurrentAmps
        EJECT(-3.0, 0), //TODO: CHANGE maxCurrentAmps
        HOLD(1.0, 0); //TODO: CHANGE maxCurrentAmps
        
        private double velocity;
        private double supplyCurrentLimit;
        
        private IntakeRollersTarget(double velocity, double supplyCurrentLimit){
            this.velocity = velocity;
        }

        public double getVelocity(){
            return velocity;
        }

        public double getSupplyCurrentLimit(){
            return supplyCurrentLimit;
        }
    }

    public double getVelocity() {
      return velocity;
    }
  }

  public IntakeRollers(IntakeRollersIO intakeRollersIO) {
    super("Intake/Intake Rollers", intakeRollersIO);
    setVelocityTarget(IntakeRollersTarget.IDLE);
  }
}
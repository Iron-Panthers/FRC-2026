package frc.robot.subsystems.climb.climbElevator;

import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructure;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIO;


public class ClimbElevator extends GenericSuperstructure<ClimbElevator.ClimbElevatorTarget>{
    public enum ClimbElevatorTarget implements GenericSuperstructure.PositionTarget{
        PLACEHOLDER1,
        PLACEHOLDER2,
        PLACEHOLDER3;

        private double position = 0;

        @Override
        public double getPosition() {
            return position;
        }

        @Override
        public double getEpsilon() {
            return 0.1;
        }

        private ClimbElevatorTarget() {
            //idk
        }

    }

    public ClimbElevator(String name, GenericSuperstructureIO superstructureIO) {
        super(name, superstructureIO);
    }
}

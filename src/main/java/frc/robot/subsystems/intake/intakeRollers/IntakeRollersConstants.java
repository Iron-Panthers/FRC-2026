package frc.robot.subsystems.intake.intakeRollers;

import frc.robot.Constants;

public class IntakeRollersConstants {

    public static final int ID = switch (Constants.getRobotType()){
        case SIM -> 16;
        default -> 16;
    };

    public static final int CURRENT_LIMIT_AMPS = switch (Constants.getRobotType()){
        case SIM -> 1;
        default -> 1;
    };

    public static final boolean INVERTED = switch (Constants.getRobotType()){
        case SIM -> true;
        default -> true;
    };

    public static final boolean BRAKE = switch (Constants.getRobotType()){
        case SIM -> true;
        default -> true;
    };

    public static final int REDUCTION = switch (Constants.getRobotType()){
        case SIM -> 1;
        default -> 1;
    };
}

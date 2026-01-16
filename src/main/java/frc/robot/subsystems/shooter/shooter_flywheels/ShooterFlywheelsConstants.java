package frc.robot.subsystems.shooter.shooter_flywheels;

import frc.robot.Constants;

public class ShooterFlywheelsConstants {
    public static final int ID =
        switch (Constants.getRobotType()){
            case COMP -> 40;
            case SIM -> 32;
            default -> 0;
        };
    public static final int CURRENT_LIMIT_AMPS =
        switch (Constants.getRobotType()) {
            case COMP -> 40;
            case SIM -> 40;
            default -> 40;
        };
    public static final boolean INVERTED =
        switch (Constants.getRobotType()) {
            case COMP -> true;
            case SIM -> true;
            default -> true;
        };
    public static final boolean BRAKE =
        switch (Constants.getRobotType()) {
            default -> true;
        };
    public static final double REDUCTION =
        switch (Constants.getRobotType()) {
            case COMP -> 5;
            case SIM -> 5;
            default ->1;
        };

    //idk if we need the stuff below; from 2025 spring
    public static final double MOI = 0.000105;
    public static final double FILTERED_SUPPLY_CURRENT_THRESHOLD_INTAKING = 23;
}
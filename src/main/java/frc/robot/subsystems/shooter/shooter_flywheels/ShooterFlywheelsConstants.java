package frc.robot.subsystems.shooter.shooter_flywheels;

import com.ctre.phoenix6.signals.GravityTypeValue;

import frc.robot.Constants;

public class ShooterFlywheelsConstants {
    //TODO: update ids
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
            default -> 1;
        };

    //TODO: update pid constants for shooter flywheels
    public static final PIDGains GAINS =
        switch(Constants.getRobotType()){
            case COMP -> new PIDGains(0,0,0,0,0,0);
            case SIM -> new PIDGains(0,0,0,0,0,0);
            default -> new PIDGains(0,0,0,0,0,0);
        };
    public static final MotionMagicConfig MOTION_MAGIC_CONFIG =
      switch (Constants.getRobotType()) {
        case COMP -> new MotionMagicConfig(0, 0);
        case SIM -> new MotionMagicConfig(0, 0);
        default -> new MotionMagicConfig(0, 0);
      };


    //idk if we need the stuff below; from 2025 sprint
    public static final double MOI = 0.000105;
    public static final double FILTERED_SUPPLY_CURRENT_THRESHOLD_INTAKING = 23;

    //implementing constant
    public record PIDGains(
        double kP, double kI, double kD, double kS, double kV, double kA){}
    
    public record MotionMagicConfig(double accelerations, double cruiseVelocity){}

    public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;
}
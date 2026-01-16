package frc.robot.subsystems.shooter.shooter_hood;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.canWatchdog.CANWatchdogConstants.CAN;

public class ShooterHoodConstants {
    public static final ShooterHoodConfig SHOOTER_HOOD_CONFIG = 
        //TODO update the id's and info
        switch(Constants.getRobotType()){
            case COMP -> new ShooterHoodConfig(
                //reduction between sensor and mechanism
                CAN.at(30, "Shooter Hood"),
                CAN.at(31, "Shooter Hood Encoder"),
                -0.01444, 
                2.25);  //find the reduction for the encoder
            case SIM -> new ShooterHoodConfig(
                //Reduction between motor and mechanism
                CAN.at(8, "Shooter Hood"), 0, 0, 12 * 0.3750);
            default -> new ShooterHoodConfig(0,0,0,1);
        };
    
    //TODO update all the PID information
    public static final PIDGains GAINS = 
        switch(Constants.getRobotType()){
            case COMP -> new PIDGains(60, 0, 0, 0, 2.265488, 0.1, 0.4);
            case SIM -> new PIDGains(40, 0, 0, 0, 3.6144, 0.1807, 0.53);
            default -> new PIDGains(0, 0, 0, 0, 0, 0, 0);
        };
    
    public record ShooterHoodConfig(
        int motorID, int canCoderID, double canCoderOffset, double reduction) {}

    public record PIDGains(
        double kP, double kI, double kD, double kS, double kV, double kA, double kG){}
    
    

}

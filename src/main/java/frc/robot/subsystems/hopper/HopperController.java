package frc.robot.subsystems.hopper;
import frc.robot.subsystems.hopper.Hopper;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheel;


public class HopperController extends SubsystemBase {
    
    public enum hopperState{
        IDLE,
        INTAKE(),
        
    }
    private final Hopper place;
    public HopperController(Hopper place) {
        this.place = place;
        
    }
    private hopperState target = hopperState.IDLE;
    public hopperState getTargetState() {
        return target;
    }
    

}

package frc.robot.subsystems.intake.intakeRollers;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import frc.robot.lib.generic_subsystems.rollers.GenericRollersIOTalonFX;
import static frc.robot.subsystems.intake.intakeRollers.IntakeRollersConstants.*;
public class IntakeRollersIOTalonFX extends GenericRollersIOTalonFX implements IntakeRollersIO {
    public IntakeRollersIOTalonFX() {
        super(ID, CURRENT_LIMIT_AMPS, INVERTED, BRAKE, REDUCTION);
    }    
    
}

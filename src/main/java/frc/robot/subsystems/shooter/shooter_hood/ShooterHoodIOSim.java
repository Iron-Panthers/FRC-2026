package frc.robot.subsystems.shooter.shooter_hood;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.lib.generic_subsystems.superstructure.GenericSuperstructureIOSim;

public class ShooterHoodIOSim extends GenericSuperstructureIOSim implements ShooterHoodIO{
    
    //TODO decide whether if it should be an arm (if hood rotates around fixed pivot point)
    private final SingleJointedArmSim shooterHoodSim;
    private final double reduction;

    public ShooterHoodIOSim(){
        super(ShooterHoodConstants.SHOOTER_HOOD_CONFIG.motorID());
        
        this.reduction = ShooterHoodConstants.SHOOTER_HOOD_CONFIG.reduction();

        shooterHoodSim = 
            new SingleJointedArmSim(
                  DCMotor.getKrakenX60Foc(1),
                reduction,
                ShooterHoodConstants.PHYSICAL_CONSTANTS.momentOfInertia(),
                ShooterHoodConstants.PHYSICAL_CONSTANTS.lengthMeters(),
                ShooterHoodConstants.PHYSICAL_CONSTANTS.minAngleRads(),
                ShooterHoodConstants.PHYSICAL_CONSTANTS.maxAngleRads(),
                ShooterHoodConstants.PHYSICAL_CONSTANTS.simulateGravity(),
                0);
        setOffset();
        setSlot0(
            ShooterHoodConstants.GAINS.kP(),
            ShooterHoodConstants.GAINS.kI(),
            ShooterHoodConstants.GAINS.kD(),
            ShooterHoodConstants.GAINS.kS(),
            ShooterHoodConstants.GAINS.kV(),
            ShooterHoodConstants.GAINS.kA(),
            ShooterHoodConstants.GAINS.kG(),
            ShooterHoodConstants.MOTION_MAGIC_CONFIG.acceleration(),
            ShooterHoodConstants.MOTION_MAGIC_CONFIG.cruiseVelocity(),
            0,
            ShooterHoodConstants.GRAVITY_TYPE);
    }

    @Override
    public void updateInputs(GenericSuperstrucutreIOInputs) {
        //Update TalonFX state
    }

} 
    

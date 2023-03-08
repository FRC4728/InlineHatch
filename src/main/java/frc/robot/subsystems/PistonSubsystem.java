package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;

public class PistonSubsystem {
    
    private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
            Constants.ArmConstants.ArmPCMForward, Constants.ArmConstants.ArmPCMBackwards);
    
    public PistonSubsystem() {
        
        m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);


    }
    public void PneumaticsToggle() {
        m_doubleSolenoid.toggle();
    }

    public void PistonArmIn() {
        m_doubleSolenoid.set(Value.kReverse);
    }

    public void PistonArmOut() {
        m_doubleSolenoid.set(Value.kForward);
    }
    public Value PistonArmExtended() {
        return m_doubleSolenoid.get();
     }
}
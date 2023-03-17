package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;

public class PistonSubsystem {
   // PneumaticHub m_pH;
    private final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
            Constants.ArmConstants.ArmPCMForward, Constants.ArmConstants.ArmPCMBackwards);
    
//  DoubleSolenoid m_doubleSolenoid = m_pH.makeDoubleSolenoid(0,1);

    public PistonSubsystem() {//PneumaticHub m_pH) {
     //   this.m_pH = m_pH;

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
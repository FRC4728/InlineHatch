package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HopperSubsystem extends SubsystemBase {
    
    private final DoubleSolenoid m_doubleSolenoid = 
    new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.HopConstants.HopPCMForward, Constants.HopConstants.HopPCMBackwards);

  public HopperSubsystem() {

    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);

  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

public void AlternateHopper() {
    m_doubleSolenoid.toggle();
   
}

public Value HopperDown() {
  return m_doubleSolenoid.get();
}


public void Stop() {
  m_doubleSolenoid.set(Value.kOff);
}

  
public void HopIn() {
  m_doubleSolenoid.set(Value.kReverse);
}
  
public void HopOut() {
  m_doubleSolenoid.set(Value.kForward);
}

public Value IsHopOut() {
  return m_doubleSolenoid.get();
}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

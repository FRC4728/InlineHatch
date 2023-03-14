package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtendingSubsystem extends SubsystemBase {
    // The motors on the left side of the drive.

    CANSparkMax m_ArmExtend = new CANSparkMax(Constants.ArmConstants.ArmExtenderID, MotorType.kBrushless);
    PowerDistribution m_PDP;
    private SparkMaxPIDController m_PIDControllerExtend;

    private RelativeEncoder m_encoderExtend;

    private SparkMaxLimitSwitch m_fLimitSwitch;
    private SparkMaxLimitSwitch m_rLimitSwitch;

    public double maxVel, maxAcc;

    double processVariable;

    public ExtendingSubsystem(PowerDistribution m_PDP) {
        this.m_PDP = m_PDP;
        m_encoderExtend = m_ArmExtend.getEncoder();
       //  m_ArmExtend.setSoftLimit(SoftLimitDirection.kForward, 65);
       //  m_ArmExtend.enableSoftLimit(SoftLimitDirection.kForward, true);

        resetEncoders();

        m_ArmExtend.set(0);

        m_ArmExtend.restoreFactoryDefaults();

        m_ArmExtend.setInverted(true);

        m_ArmExtend.setIdleMode(IdleMode.kBrake);


        m_rLimitSwitch = m_ArmExtend.getReverseLimitSwitch(Type.kNormallyOpen);
        m_fLimitSwitch = m_ArmExtend.getForwardLimitSwitch(Type.kNormallyOpen);
        
        m_PIDControllerExtend = m_ArmExtend.getPIDController();

        m_PIDControllerExtend.setP(Constants.kArmExtendGains.kP);
        m_PIDControllerExtend.setI(Constants.kArmExtendGains.kI);
        m_PIDControllerExtend.setD(Constants.kArmExtendGains.kD);
        m_PIDControllerExtend.setFF(Constants.kArmExtendGains.kF);

        maxVel = 5676;
        maxAcc = 5676;

        int smartMotionSlot = 0;

        m_PIDControllerExtend.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        m_PIDControllerExtend.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        m_PIDControllerExtend.setSmartMotionAllowedClosedLoopError(.3, 0);

        m_ArmExtend.setSmartCurrentLimit(60);
        // m_ArmExtend.setSmartCurrentLimit(smartMotionSlot, smartMotionSlot,
        // smartMotionSlot);

        m_ArmExtend.burnFlash();

    }

    @Override
    public void periodic() {
     //   if (m_rLimitSwitch.isPressed()){
       //     resetEncoders();
        //};
SmartDashboard.putNumber("AcmeScrewposition", m_encoderExtend.getPosition());
//SmartDashboard.putNumber("AcmeScrewCurrent", m_PDP.getCurrent(9));

    }

    public void ExtendOverride(double Speed) {

        m_ArmExtend.set(Speed);

    }

    public void RetractButton() {
        m_ArmExtend.set(-.2);
    }

    public void Extend() {
        // resetEncoders();
        m_PIDControllerExtend.setReference(60, ControlType.kPosition, 0, 0, ArbFFUnits.kPercentOut);
        // m_PIDControllerExtend.setReference(joystickButton6, ControlType.kDutyCycle);
    }

    public void ExtendMid() {
        // resetEncoders();
        m_PIDControllerExtend.setReference(45, ControlType.kPosition, 0, 0, ArbFFUnits.kPercentOut);
        // m_PIDControllerExtend.setReference(joystickButton6, ControlType.kDutyCycle);
    }

    public void Retract() {
        m_PIDControllerExtend.setReference(0, ControlType.kPosition);
        // m_PIDControllerExtend.setReference(joystickButton6, ControlType.kDutyCycle);
    }

    public void ExtendToGround() {
        m_PIDControllerExtend.setReference(48.3, ControlType.kPosition);
        // m_PIDControllerExtend.setReference(joystickButton6, ControlType.kDutyCycle);
    }

    public void stop() {
        m_ArmExtend.set(0);
    }

    public void resetEncoders() {
        m_encoderExtend.setPosition(0);
    }

    public double getEncoderExtend() {

        return m_encoderExtend.getPosition();
    }
}

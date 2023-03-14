package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


     public class HandSubsystem extends SubsystemBase {
        // The motors on the left side of the drive.
         
        CANSparkMax m_HandMotor = new CANSparkMax(Constants.ArmConstants.HandMotorID, MotorType.kBrushless);
        
      private PowerDistribution m_PDP;
    
        private SparkMaxPIDController m_HandController;
      private double voltage;
        private RelativeEncoder m_HandEncoder;

        public Timer m_timer = new Timer();
        public HandSubsystem(PowerDistribution m_PDP) {

          this.m_PDP = m_PDP;
            m_HandEncoder = m_HandMotor.getEncoder();            
   
            m_HandMotor.set(0);

       //  SmartDashboard.putBoolean("True", false);

            m_HandMotor.restoreFactoryDefaults();


            m_HandMotor.setInverted(false);
       //     m_HandEncoder.setVelocityConversionFactor(0);

            m_HandMotor.setIdleMode(IdleMode.kBrake);


            m_HandController = m_HandMotor.getPIDController();

            m_HandController.setFeedbackDevice(m_HandEncoder);

            m_HandController.setP(.00007);
            m_HandController.setI(0);
            m_HandController.setD(0);
            m_HandController.setFF(0.01);

            m_HandMotor.setClosedLoopRampRate(.5);
            m_HandMotor.enableVoltageCompensation(12);

          


            m_HandMotor.burnFlash();
        }

        @Override
        public void periodic() {
        SmartDashboard.putNumber("Hand Voltage",    m_PDP.getCurrent(8));
       voltage = m_PDP.getCurrent(8);
         SmartDashboard.putNumber("Hand Velocity", m_HandEncoder.getVelocity());
        }

        public void RunHandsInCone(){
        //try
            m_HandController.setReference(-250
            , CANSparkMax.ControlType.kVelocity, 0, .0, ArbFFUnits.kPercentOut);

          //  processVariable = m_encoderActuate.getVelocity();
        }

        public void RunHandsOutCone(){
            //try
                m_HandController.setReference(250, CANSparkMax.ControlType.kVelocity);
              //  processVariable = m_encoderActuate.getVelocity();
            }

            
        public void RunHandsInCube(){
         //try
             m_HandController.setReference(300, CANSparkMax.ControlType.kVelocity, 0, .0, ArbFFUnits.kPercentOut);
 
           //  processVariable = m_encoderActuate.getVelocity();
         }
 
         public void RunThemHandsSlowly(){
          //try
              m_HandController.setReference(30, CANSparkMax.ControlType.kVelocity, 0, .0, ArbFFUnits.kPercentOut);
  
            //  processVariable = m_encoderActuate.getVelocity();
          }

         public void RunHandsOutCube(){
             //try
                 m_HandController.setReference(-150, CANSparkMax.ControlType.kVelocity);
               //  processVariable = m_encoderActuate.getVelocity();
             }

             public void HandOverride(){

               m_HandController.setReference(100, ControlType.kVelocity);
             }

         public double GetPosition(){
            return m_HandEncoder.getPosition();
         }

         public void TimerReset(){
            m_timer.start();
            m_timer.reset();
         }

      public boolean getvoltage(){

         if (voltage > 22
          & m_timer.get() >= 1.3){
           // SmartDashboard.putBoolean("true", true);
            return true;
         }
        else {
          //SmartDashboard.putBoolean("true", true);
         return false;}
      }
      public boolean getvoltageCube(){

        if (voltage > 14
         & m_timer.get() >= 1){
          // SmartDashboard.putBoolean("true", true);
           return true;
        }
       else {
         //SmartDashboard.putBoolean("true", true);
        return false;
        }   
      }

      public void Stop(){
         m_HandMotor.set(0);
      }


    }

 
    
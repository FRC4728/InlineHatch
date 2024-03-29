package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class SwerveModule {
    public int moduleNumber;
    public double RobotSpeed;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private DutyCycleEncoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
      ///  angleEncoder = new CANCoder(moduleConstants.cancoderID);
      //  configAngleEncoder();
        angleEncoder = new DutyCycleEncoder(new DigitalInput(moduleConstants.cancoderID));
        /* Angle Motor Config */
        
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
                configAngleMotor();
      

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean quickTurn, boolean zoom, boolean Auto){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState, quickTurn, zoom);
        setSpeed(desiredState, quickTurn, zoom, Auto);
    }


    public void driveSlowly(){
        mDriveMotor.set(ControlMode.PercentOutput, .2);

    }
    private void setSpeed(SwerveModuleState desiredState, boolean quickTurn, boolean zoom, boolean Auto){
     SmartDashboard.putBoolean("quickTurn", quickTurn);
        if (Auto == true){
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
    
        }
      else if (quickTurn == false & zoom == false) {

        
           double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity/4, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
    }
    else if (zoom == true){
        
        double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        mDriveMotor.set(ControlMode.Velocity, velocity/1.7, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
    
    }
    else{
        
       double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
        mDriveMotor.set(ControlMode.Velocity, velocity/10, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
   
    }
    }
    private void setAngle(SwerveModuleState desiredState, boolean quickTurn, boolean zoom){

            

       
        if(Math.abs(mDriveMotor.getSelectedSensorVelocity()) < 4000){
           Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        

           mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
           lastAngle = angle;
        }
        else if (quickTurn == true & Math.abs(mDriveMotor.getSelectedSensorVelocity()) < 6000)
       {
            Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        

            mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
            lastAngle = angle;
        }
       else {
         Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        
           mAngleMotor.set(ControlMode.MotionMagic, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
        lastAngle = angle;
      }
    }
    

    private void setAngleAuto(SwerveModuleState desiredState){
        //  if (quickTurn == true & WheelTurnSpeed == 4000){
       //       WheelTurnSpeed = 10000
      //        mAngleMotor.configMotionAcceleration(10000);
      //        mAngleMotor.configMotionCruiseVelocity(20000);
      //    }
      //    else if ( quickTurn = false & WheelTurnSpeed == 10000){
      //        WheelTurnSpeed = 4000;
              
      //    mAngleMotor.configMotionAcceleration(2000);
      //    mAngleMotor.configMotionCruiseVelocity(4000);
  
          RobotSpeed = Math.abs(mDriveMotor.getSelectedSensorVelocity());
             Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
          
  
             mAngleMotor.set(ControlMode.MotionMagic, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
             lastAngle = angle;

       

      
      }
  

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder(){

        double Absoluteangle = angleEncoder.getAbsolutePosition();

        Absoluteangle = Absoluteangle * 2 * Math.PI;

        return Rotation2d.fromRadians(Absoluteangle);
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
        SmartDashboard.putBoolean("funny", false);
    }

    

    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.configMotionAcceleration(10000,30);
        mAngleMotor.configMotionCruiseVelocity(10000);
        mAngleMotor.configMotionSCurveStrength(1);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveAutoFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }

    public void stop() {
        mDriveMotor.set(ControlMode.PercentOutput, 0);
        mAngleMotor.set(ControlMode.PercentOutput, 0);
    }
}
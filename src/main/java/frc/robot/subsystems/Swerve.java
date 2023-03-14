package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Optional;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {

  //  private PhotonVisionSubsystem vision1;
    public SwerveDriveOdometry swerveOdometry;
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveModule[] mSwerveMods;
    private final PIDController m_YController;
   // private DriverStation m_driverStation;
    private DriverStation.Alliance allianceColor;
    public Pigeon2 gyro;
   private double KpAim = -0.1;
    private double KpDistance = -0.1;
    private double min_aim_command = 0.05;

  // private static final edu.wpi.first.math.Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0, 0, 0);

  //  private static final edu.wpi.first.math.Vector<N3> driveMeasurementStdDevs = VecBuilder.fill(0, 0, 0);

    public Swerve() {//PhotonVisionSubsystem vision1) {
      //  this.vision1 = vision1; 
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();



         allianceColor = DriverStation.getAlliance();

         SmartDashboard.putString("Alliance Color", allianceColor.toString());


        m_YController = new PIDController(.007, 0, 0);

         m_YController.setTolerance(0.05);
  

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };


        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
       // Timer.delay(1.0);
       new Thread(() -> {
        try {
            //possible
        //    resetEncoders();
            Thread.sleep(3000);
            resetModulesToAbsolute();
            SmartDashboard.putBoolean("True", true);
        } catch (Exception e) {
        }
    }).start();
    
         poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)));//, driveMeasurementStdDevs);// visionMeasurementStdDevs);
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        
    }

    public void drive(Translation2d translation, double rotation, boolean quickTurn, boolean zoom, boolean angletohop) {
      if (angletohop == false){  SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()));
                               
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], quickTurn, zoom, false);
        }}

        if (angletohop == true){ 

            if  (quickTurn == true) {
            if (allianceColor == Alliance.Red){
     double y_SetPoint = 90;
            
     double y_Speed =  m_YController.calculate(getYaw().getDegrees(), y_SetPoint);
            SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    y_Speed, 
                                    getYaw()));
                               
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], quickTurn, zoom, false);
        }}
        else if (allianceColor == Alliance.Blue){
            double y_SetPoint = -90;
                   
            double y_Speed =  m_YController.calculate(getYaw().getDegrees(), y_SetPoint);
                   SwerveModuleState[] swerveModuleStates =
                   Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                       ChassisSpeeds.fromFieldRelativeSpeeds(
                                           translation.getX(), 
                                           translation.getY(), 
                                           y_Speed, 
                                           getYaw()));
                                      
               SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
       
               for(SwerveModule mod : mSwerveMods){
                   mod.setDesiredState(swerveModuleStates[mod.moduleNumber], quickTurn, zoom, false);
               }} 
        }
            else if (quickTurn == false){
             if (allianceColor == Alliance.Red){
                    double y_SetPoint = 90;
                           
                    double y_Speed =  m_YController.calculate(getYaw().getDegrees(), y_SetPoint);
                           SwerveModuleState[] swerveModuleStates =
                           Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                               ChassisSpeeds.fromFieldRelativeSpeeds(
                                                   translation.getX(), 
                                                   translation.getY(), 
                                                   y_Speed, 
                                                   getYaw()));
                                              
                       SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
               
                       for(SwerveModule mod : mSwerveMods){
                           mod.setDesiredState(swerveModuleStates[mod.moduleNumber], quickTurn, zoom, false);
                       }}
             else if (allianceColor == Alliance.Blue){
                           double y_SetPoint = -90;
                                  
                           double y_Speed =  m_YController.calculate(getYaw().getDegrees(), y_SetPoint);
                                  SwerveModuleState[] swerveModuleStates =
                                  Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                                      ChassisSpeeds.fromFieldRelativeSpeeds(
                                                          translation.getX(), 
                                                          translation.getY(), 
                                                          y_Speed, 
                                                          getYaw()));
                                                     
                              SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
                      
                              for(SwerveModule mod : mSwerveMods){
                                  mod.setDesiredState(swerveModuleStates[mod.moduleNumber], quickTurn, zoom, false);
                              }} 
            }

            }
    }

    public void DriveAutomatically(){
        for(SwerveModule mod : mSwerveMods){
            mod.driveSlowly();
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false, false, false);
        }
    }    

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(180);
    }

    public Rotation2d getYaw() {
        double yeehaw = gyro.getYaw();
        return Rotation2d.fromDegrees(Math.IEEEremainder(yeehaw, 360));
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void updateOdometry() {
        poseEstimator.update(getYaw(), getModulePositions());  

        // Also apply vision measurements. We use 0.3 seconds in the past as an example
        // -- on
        // a real robot, this must be calculated based either on latency or timestamps.

     //  Optional<EstimatedRobotPose> result =
      //         vision1.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());


     //   if (result.isPresent()) {
       //     EstimatedRobotPose camPose = result.get();
       //     SmartDashboard.putString("Photonvision pose", camPose.estimatedPose.toPose2d().toString());
       //     poseEstimator.addVisionMeasurement(
         //           camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
       
//    }
}
    @Override
    public void periodic(){
        updateOdometry();  
                    SmartDashboard.putString("Robot Pose", poseEstimator.getEstimatedPosition().toString());
                    SmartDashboard.putNumber("Pitch", getPitch());
                    allianceColor = DriverStation.getAlliance();
                    SmartDashboard.putString("Alliance Color", allianceColor.toString());
                    
                    for(SwerveModule mod : mSwerveMods){
                        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
                        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
                        SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
                    }


    }


    public double getTheta() {
        return getYaw().getRadians();
    }
    public void stopModules() {
        for(SwerveModule mod : mSwerveMods){
            mod.stop();
        }

    }

    public double getPitch()
    {
        return gyro.getPitch();
    }
}
/* 
public void Update_Limelight_Tracking()
{
      // These numbers must be tuned for your Robot!  Be careful!
      final double STEER_K = 0.03;                    // how hard to turn toward the target
      final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
      final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
      final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

      double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
      double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
      double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
      double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

      if (tv < 1.0)
      {
        m_LimelightHasValidTarget = false;
        m_LimelightDriveCommand = 0.0;
        m_LimelightSteerCommand = 0.0;
        return;
      }

 

        double heading_error = -tx;
        double distance_error = -ty;
        double steering_adjust = 0.0f;

        if (tx > 1.0)
        {
                steering_adjust = KpAim*heading_error - min_aim_command;
        }
        else if (tx < -1.0)
        {
                steering_adjust = KpAim*heading_error + min_aim_command;
        }

        double distance_adjust = KpDistance * distance_error;

        left_command += steering_adjust + distance_adjust;
        right_command -= steering_adjust + distance_adjust;

}
} */
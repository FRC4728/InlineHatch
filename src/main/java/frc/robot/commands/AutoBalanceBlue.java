package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Swerve;

//IMPORTANT: the auto balance works by getting the robot elevated on the charge station
//by driving onto it during auto using path planner and then the robot drives foward using
//a PID loop so that it slows down until it hits the set point and if it overshoot, the 
//robot turns around and corrects.


//TODO: check the unit of measurement for your roll, if degrees, leave the kp value for the PID controler, 
//if in radiands, divide by 360 and multiply by two PI

public class AutoBalanceBlue extends CommandBase {
  double reversed;
  double funnyreversed;
  double totalrotation;
  Translation2d m_robotSpeeds = new Translation2d(0, 0);
  private final PIDController m_XController;

  private final Swerve m_robotDrive;

  /** Creates a new SwerveWithPIDY. */

  public AutoBalanceBlue(Swerve robotDrive) 
  {


    m_XController = new PIDController(0.4, 0, 0.004);

    m_XController.setTolerance(.05);



    m_robotDrive = robotDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive);
  

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("getting scheduled");

  }

  // Called every time the scheduler runs while the command is scheduled.

 
  @Override
  public void execute() 
  {
     //y setpoint is the target roll of the robot which we want to be 0 so the robot targets level
     //will have to adjust the set point in degrees based off on the pitch of the robot at level. 
     //if your gyro isn't completley flat you would change the setpoint to what the gyro reads out
     //when it is flat. m

  // double roborotationpitch = Math.cos(m_robotDrive.getYaw().getRadians());
    //double roborotationroll = Math.abs(0)
    //double totalrotation = Math.cos(roborotation *m_robotDrive.getPitch())+ (m_robotDrive.getRoll() * Math.PI)/180);
   // SmartDashboard.putNumber("Inclination", totalrotation);

  totalrotation = m_robotDrive.getPitch();
     double inclination = Math.atan(Math.sqrt((Math.tan(m_robotDrive.getRoll()) * Math.tan(m_robotDrive.getRoll())) +
     (Math.tan(m_robotDrive.getPitch()) * Math.tan(m_robotDrive.getPitch())) 
     ));



    double X_SetPoint = 1;
    double X_Speed =  m_XController.calculate(inclination, X_SetPoint);

    
 //   double Y_SetPoint = 0;
  //  double Y_Speed =  m_YController.calculate(m_robotDrive.getRoll(), Y_SetPoint);
    

    double X_Rotate =  ((((Math.cos(m_robotDrive.getPose().getRotation().getRadians())))*((Math.cos(m_robotDrive.getPose().getRotation().getRadians()))))
    * (((Math.cos(m_robotDrive.getPose().getRotation().getRadians())))/ (Math.abs(Math.cos(m_robotDrive.getPose().getRotation().getRadians())))));
   
    if  (Math.cos(m_robotDrive.getPose().getRotation().getRadians()) >= 0 ) {
     reversed = 1;
    }

    if  (Math.cos(m_robotDrive.getPose().getRotation().getRadians()) < 0) {
       reversed = -1;
      }
    
      if (m_robotDrive.getPitch() >= 0) {
        funnyreversed = -1;
      }

      if (m_robotDrive.getPitch() < 0) {
        funnyreversed = 1;
      }

SmartDashboard.putNumber("totalRotation", totalrotation);

    double Y_Rotate = Math.cos(m_robotDrive.getPose().getRotation().getRadians());
   
     // if(totalrotation < -11){
     m_robotSpeeds = new Translation2d(-.1, 0);
   //if(totalrotation > 7){
     // m_robotSpeeds = new Translation2d(.04, 0);
   // }

   ////   if(totalrotation >= -11){
       // end(true);
    //}
   //   (X_Rotate*(-X_Speed)) + (Y_Rotate *(-Y_Speed)) 
    //   ,0);

    m_robotDrive.drive(m_robotSpeeds, 0, false, false, false);
    System.out.println("Balance running");



    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_robotDrive.drive(new Translation2d(0, 0), .01, false, false, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(totalrotation >= -13){
      return true;
  }
    return false;
  }
}

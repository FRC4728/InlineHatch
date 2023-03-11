package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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

public class AutoBalance extends CommandBase {

  private final PIDController m_YController;
  private final Swerve m_robotDrive;

  /** Creates a new SwerveWithPIDY. */

  public AutoBalance(Swerve robotDrive) 
  {


    m_YController = new PIDController(0.005, 0, 0);

    m_YController.setTolerance(0.05);



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
     //when it is flat. 
    double y_SetPoint = 0;
    double y_Speed =  m_YController.calculate(m_robotDrive.getPitch(), y_SetPoint);
    
    Translation2d m_robotSpeeds = new Translation2d(-y_Speed, 0);
    m_robotDrive.drive(m_robotSpeeds, 0, false, false, false);
    System.out.println("Balance running");


    
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_YController.atSetpoint() == true )
           return true;
    return false;
  }
}

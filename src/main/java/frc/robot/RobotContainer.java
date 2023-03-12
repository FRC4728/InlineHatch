package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.opencv.photo.Photo;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autos.Auto1;
import frc.robot.Autos.DoNothingAuto;
import frc.robot.Autos.PlaceAndBackUpAutoLow;
import frc.robot.Autos.PlaceandBackUpAutoHigh;
import frc.robot.Autos.TestAuto;
import frc.robot.Autos.ChargeStationAuto;
import frc.robot.commands.*;
import frc.robot.commands.ArmCommands.ArmOverride;
import frc.robot.commands.ArmCommands.ArmStopCommand;
import frc.robot.commands.ArmCommands.ArmToGroundCommand;
import frc.robot.commands.ArmCommands.ArmToHomeCommand;
import frc.robot.commands.ArmCommands.ArmToHopperCommand;
import frc.robot.commands.ArmCommands.ArmHighCommand;
import frc.robot.commands.ArmCommands.ArmHighCubeCommand;
import frc.robot.commands.ArmCommands.ArmHighHoldCommand;
import frc.robot.commands.ArmCommands.ArmHomeHold;
import frc.robot.commands.ArmCommands.ArmMiddleCommand;
import frc.robot.commands.ArmCommands.ArmMiddleCubeCommand;
import frc.robot.commands.ArmCommands.ArmMiddleHoldCommand;
import frc.robot.commands.ExtendCommands.ArmExtendCommand;
import frc.robot.commands.ExtendCommands.ArmExtendMidCommand;
import frc.robot.commands.ExtendCommands.ArmRetractCommand;
import frc.robot.commands.ExtendCommands.ExtendOverride;
import frc.robot.commands.ExtendCommands.ExtendToGroundCommand;
import frc.robot.commands.HandCommands.HandInCubeCommand;
import frc.robot.commands.HandCommands.HandInConeCommand;
import frc.robot.commands.HandCommands.HandOutConeCommand;
import frc.robot.commands.HandCommands.HandOutCubeCommand;
import frc.robot.commands.HandCommands.HandStopCommand;
import frc.robot.commands.HopCommands.HopperIn;
import frc.robot.commands.HopCommands.HopperOut;
import frc.robot.commands.PistonCommands.ArmPistonExtendCommand;
import frc.robot.commands.PistonCommands.ArmPistonRetractCommand;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = 1;
    private final int strafeAxis = 0;
    private final int rotationAxis = 4;

    /* Driver Buttons */
    private final JoystickButton c_1 = new JoystickButton(driver, 1);
    private final JoystickButton c_2 = new JoystickButton(driver, 2);
    private final JoystickButton c_3 = new JoystickButton(driver, 3);
    private final JoystickButton c_4 = new JoystickButton(driver, 4);
    private final JoystickButton c_5 = new JoystickButton(driver, 5);
    private final JoystickButton c_6 = new JoystickButton(driver, 6);
    private final JoystickButton c_7 = new JoystickButton(driver, 7);
    private final JoystickButton c_8 = new JoystickButton(driver, 8);
    private final JoystickButton c_9 = new JoystickButton(driver, 9);
    private final JoystickButton c_10 = new JoystickButton(driver, 10);
    private final JoystickButton c_11 = new JoystickButton(driver, 11);
    private final JoystickButton c_12 = new JoystickButton(driver, 12);

    private final JoystickButton c2_1 = new JoystickButton(operator, 1);
    private final JoystickButton c2_2 = new JoystickButton(operator, 2);
    private final JoystickButton c2_3 = new JoystickButton(operator, 3);
    private final JoystickButton c2_4 = new JoystickButton(operator, 4);
    private final JoystickButton c2_5 = new JoystickButton(operator, 5);
    private final JoystickButton c2_6 = new JoystickButton(operator, 6);
    private final JoystickButton c2_7 = new JoystickButton(operator, 7);
    private final JoystickButton c2_8 = new JoystickButton(operator, 8);
    private final JoystickButton c2_9 = new JoystickButton(operator, 9);
    private final JoystickButton c2_10 = new JoystickButton(operator, 10);
    private final JoystickButton c2_11 = new JoystickButton(operator, 11);
    private final JoystickButton c2_12 = new JoystickButton(operator, 12);
    /* Subsystems */

   // private final PhotonVisionSubsystem s_Vision = new PhotonVisionSubsystem();
    private final Swerve s_Swerve = new Swerve();
    private final ArmSubsystem s_Arm = new ArmSubsystem();
    private final HopperSubsystem s_Hopper = new HopperSubsystem();
    private final HandSubsystem s_Hand = new HandSubsystem();
    private final ExtendingSubsystem s_Extend = new ExtendingSubsystem();
    private final PistonSubsystem s_Piston = new PistonSubsystem();

    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    double r;
    double Theta;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
        public RobotContainer() {
       s_Swerve.setDefaultCommand(
         new TeleopSwerve(
                       s_Swerve,
                     () -> -driver.getRawAxis(translationAxis), // * -driver.getRawAxis(translationAxis),
                 () -> -driver.getRawAxis(strafeAxis), // * -driver.getRawAxis(strafeAxis),
                    () -> -driver.getRawAxis(rotationAxis),                                                                                                                                                                                                                                                                                                
                         () -> c_5.getAsBoolean(),
                      () -> c_6.getAsBoolean(),
                      () ->c_10.getAsBoolean()));
    
       s_Extend.setDefaultCommand(new ExtendOverride(
                s_Extend,
               () ->   -driver.getRawAxis(3)));

       // s_Arm.setDefaultCommand(new ArmOverride(
         //   s_Arm,
           //     () ->   -driver.getRawAxis(1)));

        m_chooser.setDefaultOption("OneBallAuto", new Auto1(s_Swerve, s_Arm, s_Hand, s_Extend, s_Hopper, s_Piston));
         m_chooser.addOption("DoNothingAuto", new DoNothingAuto(s_Swerve, s_Arm, s_Hand, s_Extend, s_Hopper, s_Piston));
         m_chooser.addOption("PlaceAndBackUpAutoHigh", new PlaceandBackUpAutoHigh(s_Swerve, s_Arm, s_Hand, s_Extend, s_Hopper, s_Piston));
         m_chooser.addOption("PlaceAndBackUpAutolow", new PlaceAndBackUpAutoLow(s_Swerve, s_Arm, s_Hand, s_Extend, s_Hopper, s_Piston));
         m_chooser.addOption("ChargeStationAuto", new ChargeStationAuto(s_Swerve, s_Arm, s_Hand, s_Extend, s_Hopper, s_Piston));
         m_chooser.addOption("TestAuto", new TestAuto(s_Swerve, s_Arm, s_Hand, s_Extend, s_Hopper, s_Piston));


        // Put the chooser on the dashboard+
        SmartDashboard.putData(m_chooser);
        


        // Configure the button bindings
        configureButtonBindings();
    
}

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */

     //  c_4.onTrue(ToHomeCommand());

    //    c_5.onTrue(new PistonArmOut(s_Piston));
     //   c_6.onTrue(new ArmPistonRetractCommand(s_Piston));
     //   c_7.onTrue(new HopperIn(s_Hopper));
     //   c_8.onTrue(new HopperOut(s_Hopper));


     //   c_6.whileTrue(new RunThemHandSlowly(s_Hand));
      //c_8.onTrue(DropCube());
     //c_1.onTrue(new InstantCommand(() -> s_Hopper.AlternateHopper()));
        //c_1.onTrue(ToIntake());
      //  c_2.onTrue(new );
    //    c_1.onTrue(ToGround());


        // c_7.onTrue(DropCone());

        //c_ArmUp.onTrue(new ArmUpCommand(s_Arm));
       // c_ArmRest.onTrue(new ArmUpCommand(s_Arm));
      //  c_9.onTrue(new ArmExtendCommand(s_Extend));
      //  c_10.onTrue(new ArmRetractCommand(s_Extend));
      //  c_ArmPneumatic.onTrue(new ArmPnuematicsCommand(s_Arm));

       // c_3.onTrue(new HandInConeCommand(s_Hand).until( () -> s_Hand.getvoltage()));
     //   c_9.whileTrue(new HandOutConeCommand(s_Hand));  

      //  c_3.onTrue(ToIntake());
        //c_6.onTrue(ToGround());


        c_3.onTrue(DropCone());
        c_7.whileTrue( new ArmPistonExtendCommand(s_Piston));
        c_8.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        c_1.onTrue(DropCube());
        c_4.onTrue(ToIntake());
        c_2.onTrue(ToGround());
        c_9.whileTrue(new ArmPistonRetractCommand(s_Piston));
      //  c_10.whileTrue(new InstantCommand(() -> s_Extend.RetractButton()));
        
        c2_1.onTrue(HighArmCone());
        c2_2.onTrue(HighArmCube());
        c2_3.onTrue(HighArmCone());
        c2_4.onTrue(MidArmCone());
        c2_5.onTrue(MidArmCube());
        c2_6.onTrue(LowScore());
        c2_7.whileTrue(new HandInConeCommand(s_Hand));
        c2_8.whileTrue(new HandInCubeCommand(s_Hand));
        c2_9.onTrue(ToHomeCubeCommand());
        c2_10.onTrue(OhShitHome());



       
        //
        //c2_9.onTrue();



       // c_3.onTrue(new ArmHighCommand(s_Arm));
      //  c_4.onTrue(new ArmPistonRetractCommand(s_Arm));  

       // c_11.onTrue(new ArmPistonRetractCommand(s_Arm));

      //  Teleop1.onTrue(ramseteTeleopCommand(
    //            new Pose2d(Units.inchesToMeters(570), Units.inchesToMeters(42.19), new Rotation2d((0) * Math.PI))));

  //  c_9.onTrue(MiddleArm());
    //    c_10.onTrue(toHomeCommand());

       // c_7.onTrue(ToHomeCommand());
        
    }


    /*      public Command HighArmCone() {
       return new SequentialCommandGroup(
                  new ArmRetractCommand(s_Extend).until(() -> (s_Extend.getEncoderExtend() <=.7)),
                new ParallelCommandGroup(new ArmHighCommand(s_Arm).until(() ->(s_Arm.getEncoderActuate() > 114.5) &  (s_Arm.getEncoderActuate() < 115.5)),
  new SequentialCommandGroup(new WaitUntilCommand(() -> (s_Arm.getEncoderActuate() > 114.5) &  (s_Arm.getEncoderActuate() < 115.5)),new ArmPistonExtendCommand(s_Piston))),
           // new ArmHighCommand(s_Arm).until(() ->(s_Arm.getEncoderActuate() > 114.5) &  (s_Arm.getEncoderActuate() < 115.5)),
                new ParallelCommandGroup(
                     //   new ParallelRaceGroup(
                       //         new ArmPistonExtendCommand(s_Piston),
                                new ArmExtendCommand(s_Extend).until(() -> ( s_Extend.getEncoderExtend() < 62) &  (s_Extend.getEncoderExtend() > 58)
                        ),
                        new ArmHighHoldCommand(s_Arm)
                )
        
       );
       
     */

      public Command HighArmCone() {
       return new SequentialCommandGroup(
                  new ArmRetractCommand(s_Extend).until(() -> (s_Extend.getEncoderExtend() <= 1)),
                 new ParallelRaceGroup(
                  new ArmHighCommand(s_Arm).until(() ->(s_Arm.getEncoderActuate() > 109.5) &  (s_Arm.getEncoderActuate() < 110.5)),
                  new ArmPistonExtendCommand(s_Piston).beforeStarting(new WaitUntilCommand(() -> s_Arm.getEncoderActuate() > 40))),
                new ParallelCommandGroup(
                      //  new ArmHighHoldCommand(s_Arm),
                        new ParallelRaceGroup(
                              //  new ArmPistonExtendCommand(s_Piston),
                                new ArmExtendCommand(s_Extend).until(() -> ( s_Extend.getEncoderExtend() < 62) &  (s_Extend.getEncoderExtend() > 58) )

                ),
                new ArmHighHoldCommand(s_Arm)

        )
       );
                                 
     }
     public Command HighArmCube() {
        return new SequentialCommandGroup(
                   new ArmRetractCommand(s_Extend).until(() -> (s_Extend.getEncoderExtend() <= 1)),
                   new ArmHighCubeCommand(s_Arm).until(() ->(s_Arm.getEncoderActuate() > 104.5) &  (s_Arm.getEncoderActuate() < 106.5)),
                   new ParallelRaceGroup(
                        new ArmHighHoldCommand(s_Arm),
                        new ArmExtendCommand(s_Extend).until(() -> ( s_Extend.getEncoderExtend() < 62) &  (s_Extend.getEncoderExtend() > 58)
                        )
                   )
        );
                                  
      }

     public Command MidArmCone() {
       return new SequentialCommandGroup(
                new ArmPistonRetractCommand(s_Piston).until(() -> (s_Piston.PistonArmExtended() == Value.kReverse)),  
                new ArmRetractCommand(s_Extend).until(() -> (s_Extend.getEncoderExtend() <= 1)),
                  new ArmMiddleCommand(s_Arm).until(() ->(s_Arm.getEncoderActuate() > 104.5) &  (s_Arm.getEncoderActuate() < 105.5)),
                  new ParallelCommandGroup(
                        new ArmMiddleHoldCommand(s_Arm),
                        new ArmExtendMidCommand(s_Extend).until(() -> (s_Extend.getEncoderExtend() < 47) & (s_Extend.getEncoderExtend() > 43))
                  )
       );
                       
                  
       
     }
     public Command MidArmCube() {
        return new SequentialCommandGroup(
                new ArmPistonRetractCommand(s_Piston).until(() -> (s_Piston.PistonArmExtended() == Value.kReverse)),
                new ArmRetractCommand(s_Extend).until(() -> (s_Extend.getEncoderExtend() <= 1)),
                new ArmMiddleCubeCommand(s_Arm).until(() ->(s_Arm.getEncoderActuate() > 89.5) &  (s_Arm.getEncoderActuate() < 90.5)),
                new ArmHighHoldCommand(s_Arm)
                );
                   
        
      }    
     
  
     
     
     public Command OhShitHome() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> s_Arm.Stop()).withTimeout(.1),
                new HandStopCommand(s_Hand).withTimeout(.1),
                new ParallelCommandGroup(
                        new ArmPistonRetractCommand(s_Piston).until(() -> (s_Piston.PistonArmExtended() == Value.kReverse)),
                        new ArmRetractCommand(s_Extend).until (() -> s_Extend.getEncoderExtend() <= 1)),
                new WaitCommand(1),
                new ArmToHomeCommand(s_Arm).until (() -> (s_Arm.getEncoderActuate() < -2.5) & (s_Arm.getEncoderActuate() > -7.5)),
                new ArmStopCommand(s_Arm).withTimeout(.1),
                new HopperIn(s_Hopper).until(() -> (s_Hopper.IsHopOut() == Value.kReverse))
        );
     };

     public Command ToHomeCubeCommand() {
        return new SequentialCommandGroup(
                //new InstantCommand(() -> s_Arm.Stop()).withTimeout(.1),
                //new HandStopCommand(s_Hand).withTimeout(.1),
                new ParallelCommandGroup(
                        new ArmPistonRetractCommand(s_Piston).until(() -> (s_Piston.PistonArmExtended() == Value.kReverse)),
                        new ArmRetractCommand(s_Extend).until (() -> s_Extend.getEncoderExtend() <= 1)),
                new ArmToHomeCommand(s_Arm).until (() -> (s_Arm.getEncoderActuate() < -2.5) & (s_Arm.getEncoderActuate() > -7.5)),
                new ArmStopCommand(s_Arm).withTimeout(.1)
        );
     };

    public Command ToIntake() {
     return new SequentialCommandGroup(
                new ParallelCommandGroup(  
                    new ArmPistonRetractCommand(s_Piston).until(() -> (s_Piston.PistonArmExtended() == Value.kReverse)) ,
                    new ArmRetractCommand(s_Extend).until (() -> (s_Extend.getEncoderExtend() <= 1))
                ),
                new ArmToHomeCommand(s_Arm).until (() -> (s_Arm.getEncoderActuate() < -2.5) & (s_Arm.getEncoderActuate() > -7.5)),
                new HopperOut(s_Hopper).until (() -> (s_Hopper.HopperDown() == Value.kForward)),
                new ArmToHopperCommand(s_Arm).until (() -> (s_Arm.getEncoderActuate() < -14.5) & (s_Arm.getEncoderActuate() > -15.5) ),
                new HandInConeCommand(s_Hand).until(() ->  s_Hand.getvoltage()),
                new ArmToHomeCommand(s_Arm).until(() -> (s_Arm.getEncoderActuate() < -1.5) &  (s_Arm.getEncoderActuate() > -8.5)),
                new HandInConeCommand(s_Hand).withTimeout(.25),
                new ArmStopCommand(s_Arm).withTimeout(.2),
                new HopperIn(s_Hopper).until (() -> s_Hopper.HopperDown() == Value.kReverse)
                );
        }
        public Command ToGround() {
                return new SequentialCommandGroup(
                                new ParallelCommandGroup(  
                                    new ArmPistonRetractCommand(s_Piston).until(() -> (s_Piston.PistonArmExtended() == Value.kReverse)) ,
                                    new ArmRetractCommand(s_Extend).until (() -> (s_Extend.getEncoderExtend() <= 1))
                                ),
                       new ArmToGroundCommand(s_Arm).until(() -> (s_Arm.getEncoderActuate() < 23.7) & (s_Arm.getEncoderActuate() > 23)),
                       new ArmStopCommand(s_Arm).withTimeout(.1),
                       new ExtendToGroundCommand(s_Extend).until(() -> (s_Extend.getEncoderExtend() < 47) & (s_Extend.getEncoderExtend() >46)),
                       new HandInCubeCommand(s_Hand).until(() -> (s_Hand.getvoltageCube() == true)),
                       new ArmRetractCommand(s_Extend).until (() -> s_Extend.getEncoderExtend() <= 1),
                       new ArmToHomeCommand(s_Arm).until(() -> (s_Arm.getEncoderActuate() < -2.5) &  (s_Arm.getEncoderActuate() > -7.5)),
                       new HandInCubeCommand(s_Hand).withTimeout(.3),
                       new ArmStopCommand(s_Arm).withTimeout(.05)

                );
                
        }

        public Command LowScore() {
          return new SequentialCommandGroup(
                          new ParallelCommandGroup(  
                              new ArmPistonRetractCommand(s_Piston).until(() -> (s_Piston.PistonArmExtended() == Value.kReverse)) ,
                              new ArmRetractCommand(s_Extend).until (() -> (s_Extend.getEncoderExtend() <= 1))
                          ),
                 new ArmToGroundCommand(s_Arm).until(() -> (s_Arm.getEncoderActuate() < 36) & (s_Arm.getEncoderActuate() > 34)),
                 new ArmStopCommand(s_Arm).withTimeout(.1)
          );
        }
        public Command DropCube() {
                return new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new ArmHighHoldCommand(s_Arm),
                                new ParallelRaceGroup(
                                        new HandOutCubeCommand(s_Hand),
                                        new WaitCommand(1)      
                                )
                        ),
                        new HandStopCommand(s_Hand).withTimeout(.2),
                        new ParallelRaceGroup(
                                new ArmPistonRetractCommand(s_Piston).withTimeout(1.5),
                                new ArmRetractCommand(s_Extend).until(() -> (s_Extend.getEncoderExtend() <= 1))
                        )
                        
          
                    //    new ArmToHomeCommand(s_Arm).until (() -> (s_Arm.getEncoderActuate() > -7.5) & (s_Arm.getEncoderActuate() < -2.5)),
                     //   new ArmStopCommand(s_Arm)
                        );
        }
        public Command DropCone() {
                return new SequentialCommandGroup(
                        new ParallelRaceGroup(
                                new ArmHighHoldCommand(s_Arm),
                        new ParallelRaceGroup(        
                                new HandOutConeCommand(s_Hand),
                                new WaitCommand(1)      
                        )),
                        new ParallelRaceGroup(
                                new ArmPistonRetractCommand(s_Piston),
                                new ArmRetractCommand(s_Extend).until(() -> (s_Extend.getEncoderExtend() <= 1))
                        ),
                        
          
                        new ArmToHomeCommand(s_Arm).until (() -> (s_Arm.getEncoderActuate() > -7.5) & (s_Arm.getEncoderActuate() < -2.5)),
                        new ArmStopCommand(s_Arm)
                        );
        }
        

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
      return m_chooser.getSelected();
 //return AutoTest();
    }


   
    // create a path to run on chooser
   /*  public Command AutoTemplate() {
        // Path Planner Path
        String robot_path = "Auto1";
        PathPlannerTrajectory TestPath = PathPlanner.loadPath(robot_path, new PathConstraints(2, .5));

        HashMap<String, Command> eventMap = new HashMap<>();

        // eventMap.put("Intake", new IntakeCommand(m_IntakeSubsystem));
        // eventMap.put("IntakeOff", new IntakeOffCommand(m_IntakeSubsystem));

        // eventMap.put("IntakeArmsDown", new PrintCommand("intakeArmsDown"));

        // 4. Construct command to follow trajectory

        // auto builder can use events added in through Path Planner
        SwerveAutoBuilder autobuilder = new SwerveAutoBuilder(
                s_Swerve::getPose,
                s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
                Constants.Swerve.swerveKinematics,
                new PIDConstants(.9, 0, .01),
                new PIDConstants(1, 0, .01),
                s_Swerve::setModuleStates,
                eventMap,
                s_Swerve);

        Command fullAuto = autobuilder.fullAuto(TestPath);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                // new InstantCommand(() ->
                // swerveSubsystem.resetOdometry(TestPath.getInitialPose())),
                fullAuto,
                new InstantCommand(() -> s_Swerve.stopModules()));

        // new InstantCommand(() -> swerveSubsystem.getPose()));
    }

    Pose2d updateangle(double D, double Angle, Rotation2d targetAngle) {

        Pose2d translate;
        double currentangle = s_Swerve.getTheta();
        double ChangedAngle = Angle - currentangle;

        double newX = D * Math.cos(ChangedAngle);
        double newY = D * -Math.sin(ChangedAngle);
        translate = new Pose2d(newX, newY, targetAngle);
        // SmartDashboard.putString("New Trajectory", translate.toString());

        return translate;
    }

    Trajectory newTrajectory(TrajectoryConfig config, Translation2d translation, Translation2d startingPose,
            Rotation2d angle, Rotation2d newAngle) {

        var interiorQuinticWaypoints = new ArrayList<Pose2d>();
        interiorQuinticWaypoints.add(new Pose2d(startingPose, angle));

        interiorQuinticWaypoints.add(new Pose2d(translation, newAngle));

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(interiorQuinticWaypoints, config);
        return trajectory;
    }*/


    // Teleop Ramsete Builder: creates paths and follows them
  /*   Command ramseteTeleopCommand(Pose2d targetPose2d) {
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // r = Math.sqrt((targetPose2d.getX()* targetPose2d.getX()) +
        // targetPose2d.getY() * targetPose2d.getY());
        // Theta = Math.atan(targetPose2d.getY() / targetPose2d.getX());

        // updateangle(r, Theta, targetPose2d.getRotation());

     //  Trajectory trajectory = newTrajectory(
       //         config,
         //       targetPose2d.getTranslation(),
           //     new Translation2d(s_Vision.CameraGet().estimatedPose.getTranslation().getX(), s_Vision.CameraGet().estimatedPose.getTranslation().getY()),
             //   s_Swerve.getYaw(),
               // targetPose2d.getRotation());

        var interiorQuinticWaypoints = new ArrayList<Pose2d>();
        interiorQuinticWaypoints.add(new Pose2d(0, 0, new Rotation2d(0)));
        interiorQuinticWaypoints.add(new Pose2d(1, 1, new Rotation2d(0)));
        // interiorQuinticWaypoints.add(new Pose2d(Units.feetToMeters(0),
        // Units.feetToMeters(0), new Rotation2d(90)));
        interiorQuinticWaypoints.add(new Pose2d(2, 0, new Rotation2d(0)));
        // interiorQuinticWaypoints.add(new Pose2d(1, 0,new Rotation2d(0)));
        // An example trajectory to follow. All units in meters.
        /*
         * Trajectory exampleTrajectory =
         * TrajectoryGenerator.generateTrajectory(
         * // Start at the origin facing the +X direction
         * new Pose2d(0, 0, new Rotation2d(0)),
         * // Pass through these two interior waypoints, making an 's' curve path
         * List.of(new Translation2d(.5, .5), new Translation2d(1, 1)),
         * // End 3 meters straight ahead of where we started, facing forward
         * new Pose2d(2, 0, new Rotation2d(0)),
         * config);
         * // Trajectory exampleTrajectory =
         * // TrajectoryGenerator.generateTrajectory(
         * // interiorQuinticWaypoints,
         * // config);
         */
    /*      var thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics, // SwerveDriveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        return new SequentialCommandGroup(

                new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(new Translation2d(s_Vision.CameraGet().estimatedPose.getTranslation().getX(), s_Vision.CameraGet().estimatedPose.getTranslation().getY()),
                 s_Swerve.getYaw()))),
                swerveControllerCommand);
    }*/

    Command AutoTest() {
        TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

       

        var interiorQuinticWaypoints = new ArrayList<Pose2d>();
        interiorQuinticWaypoints.add(new Pose2d(0, 0, new Rotation2d(0)));
       // interiorQuinticWaypoints.add(new Pose2d(1, 1, new Rotation2d(0)));
        // interiorQuinticWaypoints.add(new Pose2d(Units.feetToMeters(0),
        // Units.feetToMeters(0), new Rotation2d(90)));
        interiorQuinticWaypoints.add(new Pose2d(1, 0, new Rotation2d(0)));
        // interiorQuinticWaypoints.add(new Pose2d(1, 0,new Rotation2d(0)));
        // An example trajectory to follow. All units in meters.
        
      //   * Trajectory exampleTrajectory =
      //   * TrajectoryGenerator.generateTrajectory(
      //   * // Start at the origin facing the +X direction
      //   * new Pose2d(0, 0, new Rotation2d(0)),
      //   * // Pass through these two interior waypoints, making an 's' curve path
       //  * List.of(new Translation2d(.5, .5), new Translation2d(1, 1)),
      //   * // End 3 meters straight ahead of where we started, facing forward
      //   * new Pose2d(2, 0, new Rotation2d(0)),
        // * config);
          Trajectory exampleTrajectory =
          TrajectoryGenerator.generateTrajectory(
           interiorQuinticWaypoints,
           config);
         
        var thetaController = new ProfiledPIDController(
                1, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics, // SwerveDriveKinematics,
                new PIDController(12, 0, 0.5),
                new PIDController(12, 0, 0.5),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        return new SequentialCommandGroup(

                new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
                swerveControllerCommand);
    }

}
package frc.robot.Autos;

import frc.robot.Constants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.ArmCommands.ArmHighCommand;
import frc.robot.commands.ArmCommands.ArmHighCubeCommand;
import frc.robot.commands.ArmCommands.ArmHighHoldCommand;
import frc.robot.commands.ArmCommands.ArmStopCommand;
import frc.robot.commands.ArmCommands.ArmToGroundCommand;
import frc.robot.commands.ArmCommands.ArmToHomeCommand;
import frc.robot.commands.ExtendCommands.ArmExtendCommand;
import frc.robot.commands.ExtendCommands.ArmRetractCommand;
import frc.robot.commands.ExtendCommands.ExtendToGroundCommand;
import frc.robot.commands.HandCommands.HandInCubeCommand;
import frc.robot.commands.HandCommands.HandOutConeCommand;
import frc.robot.commands.HandCommands.HandOutCubeCommand;
import frc.robot.commands.HandCommands.HandStopCommand;
import frc.robot.commands.HopCommands.HopperIn;
import frc.robot.commands.PistonCommands.ArmPistonExtendCommand;
import frc.robot.commands.PistonCommands.ArmPistonRetractCommand;
import frc.robot.commands.PistonCommands.DriveAutomatically;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtendingSubsystem;
import frc.robot.subsystems.HandSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.PistonSubsystem;
import frc.robot.subsystems.Swerve;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ChargeStationAuto extends SequentialCommandGroup {
    


    public  ChargeStationAuto(Swerve s_Swerve, ArmSubsystem s_Arm, HandSubsystem s_Hand, ExtendingSubsystem s_Extend,
    HopperSubsystem s_Hopper, PistonSubsystem s_Piston){
    // Path Planner Path
    
      TrajectoryConfig config = new TrajectoryConfig(
              Constants.AutoConstants.kMaxSpeedMetersPerSecond,
              Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              .setKinematics(Constants.Swerve.swerveKinematics);

     

      var interiorQuinticWaypoints = new ArrayList<Pose2d>();
      interiorQuinticWaypoints.add(new Pose2d(0, 0, new Rotation2d(0)));
      interiorQuinticWaypoints.add(new Pose2d(3, 0, new Rotation2d(0)));
      // interiorQuinticWaypoints.add(new Pose2d(Units.feetToMeters(0),
      // Units.feetToMeters(0), new Rotation2d(90)));
    // interiorQuinticWaypoints.add(new Pose2d(1, 0, new Rotation2d(0)));
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
              Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);

      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
              exampleTrajectory,
              s_Swerve::getPose,
              Constants.Swerve.swerveKinematics, // SwerveDriveKinematics,
              new PIDController(1.8, 0, 0),
              new PIDController(1.8, 0, 0),
              thetaController,
              s_Swerve::setModuleStates,
              s_Swerve);

  




    // 5. Add some init and wrap-up, and return everything
    addCommands(
      new SequentialCommandGroup(
          new ArmHighCommand(s_Arm).until(() -> (s_Arm.getEncoderActuate() > 114.5) & (s_Arm.getEncoderActuate() < 115.5)),
        new ParallelRaceGroup(
            new ArmHighHoldCommand(s_Arm),
            new ParallelCommandGroup(
                new ArmPistonExtendCommand(s_Piston).withTimeout(2),
                new ArmExtendCommand(s_Extend).until(() -> (s_Extend.getEncoderExtend() < 62) & (s_Extend.getEncoderExtend() > 58)))),
        new ParallelRaceGroup(
            new ArmHighHoldCommand(s_Arm),
            new ParallelRaceGroup(
                new HandOutConeCommand(s_Hand),
                new WaitCommand(1)
            )),
            new ParallelCommandGroup(
              new ArmPistonRetractCommand(s_Piston).until(() -> (s_Piston.PistonArmExtended() == Value.kReverse)),
              new ArmRetractCommand(s_Extend).until (() -> s_Extend.getEncoderExtend() < .7)),
      new ArmToHomeCommand(s_Arm).until (() -> (s_Arm.getEncoderActuate() < -2.5) & (s_Arm.getEncoderActuate() > -7.5)),
      new ArmStopCommand(s_Arm).withTimeout(.1),          
             new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
        swerveControllerCommand)
     //  new AutoBalance(s_Swerve)
        );

    // new InstantCommand(() -> swerveSubsystem.getPose()));
  }
} 
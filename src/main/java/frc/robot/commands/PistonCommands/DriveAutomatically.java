package frc.robot.commands.PistonCommands;


import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.PistonSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveAutomatically extends CommandBase {
    private Swerve s_Swerve;



    public DriveAutomatically(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        //addRequirements(s_Piston);


    }


    @Override
    public void execute() {
        s_Swerve.DriveAutomatically();

        /* Drive */

    }

    public void end(boolean interrupted) {
        // when command ends, stop motors here
        s_Swerve.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

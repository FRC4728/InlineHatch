package frc.robot.commands.PistonCommands;


import frc.robot.subsystems.PistonSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmPistonExtendCommand extends CommandBase {
    private PistonSubsystem s_Piston;



    public ArmPistonExtendCommand(PistonSubsystem s_Piston) {
        this.s_Piston = s_Piston;
        //addRequirements(s_Piston);


    }


    @Override
    public void execute() {
        s_Piston.PistonArmOut();

        /* Drive */

    }

    public void end(boolean interrupted) {
        // when command ends, stop motors here

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

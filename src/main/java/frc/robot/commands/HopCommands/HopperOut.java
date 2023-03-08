package frc.robot.commands.HopCommands;

import frc.robot.subsystems.HopperSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class HopperOut extends CommandBase {
    private HopperSubsystem s_Hopper;



    public HopperOut(HopperSubsystem s_Hopper) {
        this.s_Hopper = s_Hopper;
        addRequirements(s_Hopper);


    }

    @Override
    public void execute() {
        s_Hopper.HopOut();

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

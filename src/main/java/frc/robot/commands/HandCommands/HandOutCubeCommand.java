package frc.robot.commands.HandCommands;

import frc.robot.subsystems.HandSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class HandOutCubeCommand extends CommandBase {
    private HandSubsystem s_Hand;



    public HandOutCubeCommand(HandSubsystem s_Hand) {
        this.s_Hand = s_Hand;
        addRequirements(s_Hand);


    }

    @Override
    public void execute() {
        s_Hand.RunHandsOutCube();

        /* Drive */

    }

    public void end(boolean interrupted) {
        // when command ends, stop motors here
        s_Hand.Stop();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

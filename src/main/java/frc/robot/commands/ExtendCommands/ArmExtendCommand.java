package frc.robot.commands.ExtendCommands;

import frc.robot.subsystems.ExtendingSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmExtendCommand extends CommandBase {
    private ExtendingSubsystem s_Extend;



    public ArmExtendCommand(ExtendingSubsystem s_Extend) {
        this.s_Extend = s_Extend;
        addRequirements(s_Extend);

    }

    public void initialize() {
        // Motor setup, start timers, ect.
    }

    @Override
    public void execute() {
        // Add in command to be executed
        s_Extend.Extend();
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

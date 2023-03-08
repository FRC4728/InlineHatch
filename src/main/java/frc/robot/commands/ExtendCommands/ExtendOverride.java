package frc.robot.commands.ExtendCommands;

import frc.robot.Constants;
import frc.robot.subsystems.ExtendingSubsystem;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ExtendOverride extends CommandBase {    
    private ExtendingSubsystem s_Extend;    
    private DoubleSupplier SpeedSupplier;

    

    public ExtendOverride(ExtendingSubsystem s_Extend, DoubleSupplier SpeedSupplier) {
        this.s_Extend = s_Extend;
        addRequirements(s_Extend);

        this.SpeedSupplier = SpeedSupplier;

    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double Speed = MathUtil.applyDeadband(SpeedSupplier.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        s_Extend.ExtendOverride(Speed);
            
      
    }

    public void end(boolean interrupted) {
        // when command ends, stop motors here
        s_Extend.stop();
    }

}
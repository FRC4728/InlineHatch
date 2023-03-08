package frc.robot.commands.ArmCommands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ArmOverride extends CommandBase {    
    private ArmSubsystem s_Arm;    
    private DoubleSupplier SpeedSupplier;

    

    public ArmOverride(ArmSubsystem s_Arm, DoubleSupplier SpeedSupplier) {
        this.s_Arm = s_Arm;
        addRequirements(s_Arm);

        this.SpeedSupplier = SpeedSupplier;

    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double Speed = MathUtil.applyDeadband(SpeedSupplier.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        s_Arm.Actuate(Speed);
            
      
    }
}
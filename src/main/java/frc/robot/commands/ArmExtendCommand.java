package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmExtendSubsystem;

public class ArmExtendCommand extends CommandBase {
    private ArmExtendSubsystem armExtendSubsystem;
    private double speed; 
    private boolean isAuto;

    public ArmExtendCommand (ArmExtendSubsystem subsystem, double speed){
        armExtendSubsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        armExtendSubsystem.setarmExtendMotor(speed);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        armExtendSubsystem.setarmExtendMotor(0);
    }
    
}

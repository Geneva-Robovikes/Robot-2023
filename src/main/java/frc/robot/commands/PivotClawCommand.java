package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotClawSubsystem;

public class PivotClawCommand extends CommandBase {
    private PivotClawSubsystem pivotClawSubsystem;
    private double speed; 
    //private boolean isAuto;

    public PivotClawCommand (PivotClawSubsystem subsystem, double speed){
        pivotClawSubsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        pivotClawSubsystem.setPivotMotor(speed);
    }
    
    //green larson
    @Override
    public boolean isFinished() {
        if (pivotClawSubsystem.getSwitchState()){
            return true;
        } else {
            return false;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        pivotClawSubsystem.setPivotMotor(0);
    }
}

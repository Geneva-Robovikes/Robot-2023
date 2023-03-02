package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.PivotClawSubsystem;

public class PivotClawCommand extends CommandBase {
    private PivotClawSubsystem pivotClawSubsystem;
    private double speed; 
    private boolean isAuto;

    public PivotClawCommand (PivotClawSubsystem subsystem, double speed){
        pivotClawSubsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        pivotClawSubsystem.setPivotMotor(speed);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        pivotClawSubsystem.setPivotMotor(0);
    }
}

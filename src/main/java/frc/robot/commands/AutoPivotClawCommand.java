package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotClawSubsystem;

public class AutoPivotClawCommand extends CommandBase {
    private PivotClawSubsystem pivotClawSubsystem;
    private double speed; 
    private double distance;

    public AutoPivotClawCommand (PivotClawSubsystem subsystem, double speed, double distance){
        pivotClawSubsystem = subsystem;
        this.speed = speed;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        pivotClawSubsystem.setControl(false);
        pivotClawSubsystem.resetEncoder();
        pivotClawSubsystem.setPivotMotor(speed);
    }
    
    @Override
    public void execute() {
        pivotClawSubsystem.setPivotMotor(speed);
    }
    
    @Override
    public boolean isFinished() {
        return Math.abs(pivotClawSubsystem.getDistance()) > distance;
    }
    
    @Override
    public void end(boolean interrupted) {
        pivotClawSubsystem.setPivotMotor(0);
        pivotClawSubsystem.setControl(true);
    }
}

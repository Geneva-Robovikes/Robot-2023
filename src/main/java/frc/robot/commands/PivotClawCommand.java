package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotClawSubsystem;

public class PivotClawCommand extends CommandBase {
    private PivotClawSubsystem pivotClawSubsystem;
    private double speed; 
    private boolean goingUp;

    public PivotClawCommand (PivotClawSubsystem subsystem, double speed, boolean goingUp){
        pivotClawSubsystem = subsystem;
        this.speed = speed;
        this.goingUp = goingUp;
    }

    @Override
    public void initialize() {
        pivotClawSubsystem.setControl(false);
        pivotClawSubsystem.setPivotMotor(speed);
    }
    
    @Override
    public void execute() {
        pivotClawSubsystem.setPivotMotor(speed);
    }
    
    @Override
    public boolean isFinished() {
        if (goingUp){
            return pivotClawSubsystem.getTopState();
        } else {
            return pivotClawSubsystem.getBottomState();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        pivotClawSubsystem.setPivotMotor(0);
        pivotClawSubsystem.setControl(true);
    }
}

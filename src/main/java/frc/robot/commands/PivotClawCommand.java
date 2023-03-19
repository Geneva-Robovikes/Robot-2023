package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class PivotClawCommand extends CommandBase {
    private ClawSubsystem clawSubsystem;
    private double speed; 
    private boolean goingUp;

    public PivotClawCommand (ClawSubsystem subsystem, double speed, boolean goingUp){
        clawSubsystem = subsystem;
        this.speed = speed;
        this.goingUp = goingUp;
    }

    @Override
    public void initialize() {
        clawSubsystem.setPivotMotor(speed);
    }
    
    @Override
    public void execute() {
        clawSubsystem.setPivotMotor(speed);
    }
    
    @Override
    public boolean isFinished() {
        if (goingUp){
            return clawSubsystem.getPivotTopState();
        } else {
            return clawSubsystem.getPivotBottomState();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setPivotMotor(0);
    }
}

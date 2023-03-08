package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotClawSubsystem;

public class PivotClawToAngle extends CommandBase {
    private PivotClawSubsystem pivotClawSubsystem;
    private double speed; 
    private double angle;
    private boolean forward;

    public PivotClawToAngle (PivotClawSubsystem subsystem, double speed, double angle){
        pivotClawSubsystem = subsystem;
        this.speed = speed;
        this.angle = angle;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        if(pivotClawSubsystem.getClawAngle() < angle) {
            pivotClawSubsystem.setPivotMotor(speed);
            forward = true;
        } else {
            pivotClawSubsystem.setPivotMotor(-speed);
            forward = false;
        }
    }
    
    @Override
    public boolean isFinished() {
        if (forward)
            return pivotClawSubsystem.getClawAngle() >= angle;
        else
            return pivotClawSubsystem.getClawAngle() <= angle;
    }
    
    @Override
    public void end(boolean interrupted) {
        pivotClawSubsystem.setPivotMotor(0);
    }
}

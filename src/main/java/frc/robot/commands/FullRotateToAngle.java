package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawArmPivotSubsystem;
import frc.robot.subsystems.PivotClawSubsystem;

public class FullRotateToAngle extends CommandBase{
    private ClawArmPivotSubsystem clawArmPivotSubsystem;
    private PivotClawSubsystem pivotClawSubsystem;
    private double speed;
    boolean goingUp;
    
    public FullRotateToAngle(ClawArmPivotSubsystem clawArmPivotSubsystem, PivotClawSubsystem pivotClawSubsystem, double speed, boolean goingUp) {
        this.clawArmPivotSubsystem = clawArmPivotSubsystem;
        this.pivotClawSubsystem = pivotClawSubsystem;
        this.speed = speed;
        this.goingUp = goingUp;
    }

    @Override
    public void initialize() {
        clawArmPivotSubsystem.setArmMotor(speed);
    }

    @Override
    public boolean isFinished() {
        if(goingUp) {
            return clawArmPivotSubsystem.getUpSwitch();
        }
        return clawArmPivotSubsystem.getDownSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        clawArmPivotSubsystem.setArmMotor(0);
    }

}

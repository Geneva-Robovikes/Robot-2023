package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawArmPivotSubsystem;
import frc.robot.subsystems.PivotClawSubsystem;

public class FullRotateToAngle extends CommandBase{
    private ClawArmPivotSubsystem clawArmPivotSubsystem;
    private PivotClawSubsystem pivotClawSubsystem;
    private double clawSpeed;
    private double armSpeed;
    boolean goingUp;
    
    public FullRotateToAngle(ClawArmPivotSubsystem clawArmPivotSubsystem, PivotClawSubsystem pivotClawSubsystem, double clawSpeed, double armSpeed, boolean goingUp) {
        this.clawArmPivotSubsystem = clawArmPivotSubsystem;
        this.pivotClawSubsystem = pivotClawSubsystem;
        this.clawSpeed = clawSpeed;
        this.armSpeed = armSpeed;
        this.goingUp = goingUp;
    }

    @Override
    public void initialize() {
        clawArmPivotSubsystem.setArmMotor(armSpeed);
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

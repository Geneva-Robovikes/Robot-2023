package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawArmPivotSubsystem;

public class ClawArmPivotCommand extends CommandBase{
    private ClawArmPivotSubsystem clawArmPivotSubsystem;
    private double speed;
    boolean goingUp;
    
    public ClawArmPivotCommand(ClawArmPivotSubsystem subsystem, double speed, boolean goingUp) {
        clawArmPivotSubsystem = subsystem;
        this.speed = speed;
        this.goingUp = goingUp;
    }

    @Override
    public void initialize() {
        clawArmPivotSubsystem.setControl(false);
        clawArmPivotSubsystem.setArmMotor(speed);
    }

    @Override
    public void execute() {
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
        clawArmPivotSubsystem.setControl(true);
    }

}

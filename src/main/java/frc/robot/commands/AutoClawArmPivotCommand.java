package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawArmPivotSubsystem;

public class AutoClawArmPivotCommand extends CommandBase{
    private ClawArmPivotSubsystem clawArmPivotSubsystem;
    private double speed;
    private double distance;
    
    public AutoClawArmPivotCommand(ClawArmPivotSubsystem subsystem, double speed, double distance) {
        clawArmPivotSubsystem = subsystem;
        this.speed = speed;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        clawArmPivotSubsystem.setControl(false);
        clawArmPivotSubsystem.resetEncoder();
        clawArmPivotSubsystem.setArmMotor(speed);
    }

    @Override
    public void execute() {
        clawArmPivotSubsystem.setArmMotor(speed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(clawArmPivotSubsystem.getArmPosition()) > distance;
    }

    @Override
    public void end(boolean interrupted) {
        clawArmPivotSubsystem.setArmMotor(0);
        clawArmPivotSubsystem.setControl(true);
    }

}

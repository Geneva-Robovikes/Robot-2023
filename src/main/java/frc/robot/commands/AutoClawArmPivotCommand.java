package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class AutoClawArmPivotCommand extends CommandBase{
    private ArmSubsystem armSubsystem;
    private double speed;
    private double distance;
    
    /*public AutoClawArmPivotCommand(ClawArmPivotSubsystem subsystem, double speed, double distance) {
        clawArmPivotSubsystem = subsystem;
        this.speed = speed;
        this.distance = distance;
    }*/

    public AutoClawArmPivotCommand(ArmSubsystem subsystem, double speed, double distance) {
        this.armSubsystem = subsystem;
        this.speed = speed;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        armSubsystem.resetArmPivotEncoder();
        armSubsystem.setArmPivotMotor(speed);
    }

    @Override
    public void execute() {
        armSubsystem.setArmPivotMotor(speed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(armSubsystem.getArmPivotPosition()) > distance;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setArmPivotMotor(0);
    }

}

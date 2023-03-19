package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawArmPivotSubsystem;

public class AutoClawArmPivotCommand extends CommandBase{
    private ClawArmPivotSubsystem clawArmPivotSubsystem;
    private ArmSubsystem armSubsystem;
    private double speed;
    private double distance;
    
    public AutoClawArmPivotCommand(ClawArmPivotSubsystem subsystem, double speed, double distance) {
        clawArmPivotSubsystem = subsystem;
        this.speed = speed;
        this.distance = distance;
    }

    public AutoClawArmPivotCommand(ArmSubsystem subsystem, double speed, double distance) {
        this.armSubsystem = subsystem;
        this.speed = speed;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        //clawArmPivotSubsystem.setControl(false);
        //clawArmPivotSubsystem.resetEncoder();
        //clawArmPivotSubsystem.setArmMotor(speed);
        armSubsystem.resetArmPivotEncoder();
        armSubsystem.setArmPivotMotor(speed);
        //armSubsystem.setArmPivotCanControl(isFinished());
    }

    @Override
    public void execute() {
        //clawArmPivotSubsystem.setArmMotor(speed);
        armSubsystem.setArmPivotMotor(speed);
    }

    @Override
    public boolean isFinished() {
        //return Math.abs(clawArmPivotSubsystem.getArmPosition()) > distance;
        return Math.abs(armSubsystem.getArmPivotPosition()) > distance;
    }

    @Override
    public void end(boolean interrupted) {
        //clawArmPivotSubsystem.setArmMotor(0);
        //clawArmPivotSubsystem.setControl(true);
        armSubsystem.setArmPivotMotor(0);
    }

}

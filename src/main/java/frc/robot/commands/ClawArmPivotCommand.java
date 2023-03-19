package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ClawArmPivotCommand extends CommandBase{
    private ArmSubsystem armSubsystem;
    private double speed;
    boolean goingUp;
    
    public ClawArmPivotCommand(ArmSubsystem subsystem, double speed, boolean goingUp) {
        armSubsystem = subsystem;
        this.speed = speed;
        this.goingUp = goingUp;
    }

    @Override
    public void initialize() {
        armSubsystem.setArmPivotMotor(speed);
    }

    @Override
    public void execute() {
        armSubsystem.setArmPivotMotor(speed);
    }

    @Override
    public boolean isFinished() {
        if(goingUp) {
            return armSubsystem.getArmPivotTopState();
        }
        return armSubsystem.getArmPivotBottomState();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setArmPivotMotor(0);
    }

}

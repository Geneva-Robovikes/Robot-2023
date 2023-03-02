package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotateArmSubsystem;

public class RotateArmCommand extends CommandBase{
    double speed;
    RotateArmSubsystem rotateArmSubsystem;

    public RotateArmCommand(RotateArmSubsystem subsystem, double speed) {
        rotateArmSubsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        rotateArmSubsystem.setArmRotationMotor(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        rotateArmSubsystem.setArmRotationMotor(0);
    }
}

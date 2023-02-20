package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmDownCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private double speed;

    public ArmDownCommand (ArmSubsystem subsystem, double speed) {
        armSubsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setArmMotor(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setArmMotor(0);
    }
}

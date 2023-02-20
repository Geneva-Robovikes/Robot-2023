package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawBackCommand extends CommandBase{
    private ClawSubsystem clawSubsystem;
    private double speed;

    public ClawBackCommand (ClawSubsystem subsystem, double speed) {
        clawSubsystem = subsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        clawSubsystem.setClawMotor(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setClawMotor(0);
    }
}

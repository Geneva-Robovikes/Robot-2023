package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;

public class PivotForwardCommand extends CommandBase{
    private PivotSubsystem pivotSubsystem;
    private double speed;

    public PivotForwardCommand (PivotSubsystem subsystem, double speed) {
        pivotSubsystem = subsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        pivotSubsystem.setPivotMotor(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.setPivotMotor(0);
    }
}

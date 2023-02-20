package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;

public class PivotBackCommand extends CommandBase{
    private PivotSubsystem pivotSubsystem;
    private double pivotSpeed;

    public PivotBackCommand (PivotSubsystem subsystem, double speed) {
        pivotSubsystem = subsystem;
        pivotSpeed = speed;
    }

    @Override
    public void initialize() {
        System.out.println("hereback");
        pivotSubsystem.setPivotMotor(pivotSpeed);
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

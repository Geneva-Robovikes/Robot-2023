package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PivotSubsystem;

public class PivotForwardCommand extends CommandBase{
    private PivotSubsystem pivotSubsystem;
    private double pivotSpeed;
    private boolean isAuto;

    public PivotForwardCommand (PivotSubsystem subsystem, double speed, boolean isAuto) {
        pivotSubsystem = subsystem;
        pivotSpeed = speed;
        this.isAuto = isAuto;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("hereforward");
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

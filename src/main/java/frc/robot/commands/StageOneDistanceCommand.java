package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StageOneSubsystem;

public class StageOneDistanceCommand extends CommandBase {
    private StageOneSubsystem StageOneSubsystem;
    private double speed;
    private double distance;

    //Assumes stage is at bottom
    public StageOneDistanceCommand(StageOneSubsystem subsystem, double speed, double distance) {
        StageOneSubsystem = subsystem;
        this.speed = speed;
        this.distance = distance;
        addRequirements(subsystem);
    } 

    @Override
    public void initialize() {
        StageOneSubsystem.resetDistance();
        StageOneSubsystem.setarmExtendMotor(speed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(StageOneSubsystem.getDistance()) > distance;
    }

    @Override
    public void end(boolean interrupted) {
        StageOneSubsystem.setarmExtendMotor(0);
    }
}

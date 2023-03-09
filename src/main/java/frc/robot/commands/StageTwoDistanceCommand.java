package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StageTwoSubsystem;

public class StageTwoDistanceCommand extends CommandBase {
    private StageTwoSubsystem stageTwoSubsystem;
    private double speed;
    private double distance;

    //Assumes stage is at bottom
    public StageTwoDistanceCommand(StageTwoSubsystem subsystem, double speed, double distance) {
        stageTwoSubsystem = subsystem;
        this.speed = speed;
        this.distance = distance;
        addRequirements(subsystem);
    } 

    @Override
    public void initialize() {
        stageTwoSubsystem.resetDistance();
        stageTwoSubsystem.setUpperMotor(speed);
    }

    @Override
    public boolean isFinished() {
        return stageTwoSubsystem.getDistance() > distance;
    }

    @Override
    public void end(boolean interrupted) {
        stageTwoSubsystem.setUpperMotor(0);
    }
}

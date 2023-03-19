package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.StageTwoSubsystem;

public class StageTwoDistanceCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private double speed;
    private double distance;

    //Assumes stage is at bottom
    public StageTwoDistanceCommand(ArmSubsystem subsystem, double speed, double distance) {
        armSubsystem = subsystem;
        this.speed = speed;
        this.distance = distance;
    } 

    @Override
    public void initialize() {
        armSubsystem.resetUpperExensionDistance();
        armSubsystem.setUpperExensionMotor(speed);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.getUpperExensionDistance() > distance;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setUpperExensionMotor(0);
    }
}

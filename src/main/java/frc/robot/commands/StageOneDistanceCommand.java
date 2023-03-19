package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class StageOneDistanceCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private double speed;
    private double distance;

    //Assumes stage is at bottom
    public StageOneDistanceCommand(ArmSubsystem subsystem, double speed, double distance) {
        armSubsystem = subsystem;
        this.speed = speed;
        this.distance = distance;
    } 

    @Override
    public void initialize() {
        armSubsystem.resetLowerExtensionDistance();
        armSubsystem.setLowerExtensionMotor(speed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(armSubsystem.getLowerExtensionDistance()) > distance;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setLowerExtensionMotor(0);
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StageTwoSubsystem;

public class Stage2Command extends CommandBase {

    private StageTwoSubsystem upperArmSubsystem;
    private double speed;

    public Stage2Command(StageTwoSubsystem subsystem, double speed) {
        upperArmSubsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    } 

    @Override
    public void initialize() {
        upperArmSubsystem.setUpperMotor(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        upperArmSubsystem.setUpperMotor(0);
    }
    
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.UpperArmSubsystem;

public class UpperArmCommand extends CommandBase {

    private UpperArmSubsystem upperArmSubsystem;
    private double speed;

    public UpperArmCommand(UpperArmSubsystem subsystem, double speed) {
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

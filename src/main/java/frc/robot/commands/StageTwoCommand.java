package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StageTwoSubsystem;

public class StageTwoCommand extends CommandBase {
    private StageTwoSubsystem stageTwoSubsystem;
    private double speed;
    private boolean goingUp;

    public StageTwoCommand(StageTwoSubsystem subsystem, double speed, boolean goingUp) {
        stageTwoSubsystem = subsystem;
        this.speed = speed;
        this.goingUp = goingUp;
        addRequirements(subsystem);
    } 

    @Override
    public void initialize() {
        stageTwoSubsystem.setUpperMotor(speed);
    }

    @Override
    public boolean isFinished() {
        if(goingUp) {
            return stageTwoSubsystem.getTopState();
        }
        return stageTwoSubsystem.getBottomState();
    }

    @Override
    public void end(boolean interrupted) {
        stageTwoSubsystem.setUpperMotor(0);
    }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StageOneSubsystem;

public class StageOneCommand extends CommandBase {
    private StageOneSubsystem stageOneSubsystem;
    private double speed; 
    private boolean goingUp;

    public StageOneCommand (StageOneSubsystem subsystem, double speed, boolean goingUp){
        stageOneSubsystem = subsystem;
        this.speed = speed;
        this.goingUp = goingUp;
        addRequirements(subsystem);
    }
    
    @Override
    public void initialize() {
        stageOneSubsystem.setarmExtendMotor(speed);
    }

    @Override
    public boolean isFinished() {
        if(goingUp) {
            return stageOneSubsystem.getTopState();
        }
        return stageOneSubsystem.getBottomState();
    }

    @Override
    public void end(boolean interrupted) {
        stageOneSubsystem.setarmExtendMotor(0);
    }
    
}

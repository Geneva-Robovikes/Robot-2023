package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StageOneSubsystem;

public class StageOneCommand extends CommandBase {
    private StageOneSubsystem stageOneSubsystem;
    private double speed; 
//green larson
    public StageOneCommand (StageOneSubsystem subsystem, double speed){
        stageOneSubsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }
    //green larson
    @Override
    public void initialize() {
        stageOneSubsystem.setarmExtendMotor(speed);
    }
    @Override
    public boolean isFinished() {
        /*if (stageOneSubsystem.getSwitchState()){
            return true; 
        } else{*/
            return false;
        //}
//greeen larson    
    }
    @Override
    public void end(boolean interrupted) {
        stageOneSubsystem.setarmExtendMotor(0);
    }
    
}

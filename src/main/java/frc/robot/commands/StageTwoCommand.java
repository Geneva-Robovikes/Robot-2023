package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StageTwoSubsystem;

public class StageTwoCommand extends CommandBase {

    private StageTwoSubsystem stageTwoSubsystem;
    private double speed;

    public StageTwoCommand(StageTwoSubsystem subsystem, double speed) {
        stageTwoSubsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    } 

    @Override
    public void initialize() {
        stageTwoSubsystem.setUpperMotor(speed);
    }

    @Override
    public boolean isFinished() {
        /*if (stageTwoSubsystem.getSwitchState()){
            return true;
        } else {*/
        
            return false;
        //}
    }
    //green larson

    @Override
    public void end(boolean interrupted) {
        stageTwoSubsystem.setUpperMotor(0);
    }
    //green larson
}

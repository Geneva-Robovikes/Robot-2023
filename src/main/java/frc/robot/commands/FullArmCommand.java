package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StageOneSubsystem;
import frc.robot.subsystems.StageTwoSubsystem;

public class FullArmCommand extends CommandBase {
    StageOneSubsystem stageOneSubsystem;
    StageTwoSubsystem stageTwoSubsystem;
    double speedOne;
    double speedTwo;
    //green larson

    public FullArmCommand(StageOneSubsystem stageOneSubsystem, StageTwoSubsystem stageTwoSubsystem, double speedOne, double speedTwo) {
        this.stageOneSubsystem = stageOneSubsystem;
        this.stageTwoSubsystem = stageTwoSubsystem;
        this.speedOne = speedOne;
        this.speedTwo = speedTwo;
    }

    @Override
    public void initialize() {
        stageOneSubsystem.setarmExtendMotor(speedOne);
        stageTwoSubsystem.setUpperMotor(speedTwo);
    }

    @Override
    public boolean isFinished() {
        //return stageOneSubsystem.getSwitchState()&&stageTwoSubsystem.getSwitchState();
        return false;
    }

    @Override
    public void execute() {
        /*if(stageOneSubsystem.getSwitchState()) {
            stageOneSubsystem.setarmExtendMotor(0);
        }
        if(stageTwoSubsystem.getSwitchState()) {
            stageTwoSubsystem.setUpperMotor(0);
        }*/
    }

    //green larson
    //green larson
    @Override
    public void end(boolean interrupted) {
        stageOneSubsystem.setarmExtendMotor(0);
        stageTwoSubsystem.setUpperMotor(0);
    }
    //green larson
}

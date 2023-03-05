package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.StageOneSubsystem;

public class Stage1Command extends CommandBase {
    private StageOneSubsystem stage1Subsystem;
    private double speed; 
    private boolean isAuto;

    public Stage1Command (StageOneSubsystem subsystem, double speed){
        stage1Subsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        stage1Subsystem.setarmExtendMotor(speed);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        stage1Subsystem.setarmExtendMotor(0);
    }
    
}

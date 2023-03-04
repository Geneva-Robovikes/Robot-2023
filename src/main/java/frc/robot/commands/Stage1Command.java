package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Stage1Subsystem;

public class Stage1Command extends CommandBase {
    private Stage1Subsystem armExtendSubsystem;
    private double speed; 
    private boolean isAuto;

    public Stage1Command (Stage1Subsystem subsystem, double speed){
        armExtendSubsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        armExtendSubsystem.setarmExtendMotor(speed);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        armExtendSubsystem.setarmExtendMotor(0);
    }
    
}

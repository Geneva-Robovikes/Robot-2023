package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase  {

private ClawSubsystem clawSubsystem;
private double speed;
//private boolean isAuto;
    public ClawCommand(ClawSubsystem subsystem, double speed){
        clawSubsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        clawSubsystem.setClawMotor(speed);
    }
    //green larson
    //green larson
    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setClawMotor(0);
    }
}

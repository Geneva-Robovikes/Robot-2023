package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase  {

private ClawSubsystem clawSubsystem;
private double speed;
private boolean isAuto;
    public ClawCommand(ClawSubsystem subsystem, double speed){
        clawSubsystem = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        clawSubsystem.setClawMotor(speed);
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        clawSubsystem.setClawMotor(0);
    }
}

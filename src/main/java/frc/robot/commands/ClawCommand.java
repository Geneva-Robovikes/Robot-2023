package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase  {
    private final ClawSubsystem clawSubsystem;
    private final double speed;
    private final double currentLimit;

    public ClawCommand(ClawSubsystem subsystem, double speed, double currentLimit){
        clawSubsystem = subsystem;
        this.speed = speed;
        this.currentLimit = currentLimit;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.setClawMotor(speed);
    }
    
    @Override
    public boolean isFinished() {
        return clawSubsystem.getClawMotorCurrent() > currentLimit;
    }

    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setClawMotor(0);
    }
}

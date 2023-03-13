package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class AutoClawCommand extends CommandBase  {

private ClawSubsystem clawSubsystem;
private double speed;
private double waitTime;
private double moveTime;
private Timer timer = new Timer();
private boolean started;

    public AutoClawCommand(ClawSubsystem subsystem, double speed, double waitTime, double moveTime){
        clawSubsystem = subsystem;
        this.speed = speed;
        this.waitTime = waitTime;
        this.moveTime = moveTime;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer.start();
        clawSubsystem.setClawMotor(speed);
    }

    @Override
    public void execute() {
        if (timer.get() > moveTime && !started) {
            started = true;
            clawSubsystem.setClawMotor(0);
            timer.reset();
        } else if (started && timer.get() > waitTime) {
            started = false;
            clawSubsystem.setClawMotor(speed);
            timer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setClawMotor(0);
    }
}

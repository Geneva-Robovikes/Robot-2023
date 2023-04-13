package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class AutoClawCommand extends CommandBase  {

private ClawSubsystem clawSubsystem;
private double speed;
private double waitTime;
private Timer timer = new Timer();

    /**
     * Command that starts the claw intake and stops after the specified time.
     * @param subsystem the claw subsystem
     * @param speed the speed to set the intake motor. Must be between -1.0 and 1.0
     * @param waitTime the time to run the intake for
     */
    public AutoClawCommand(ClawSubsystem subsystem, double speed, double waitTime){
        clawSubsystem = subsystem;
        this.speed = speed;
        this.waitTime = waitTime;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        timer.start();
        clawSubsystem.setClawMotor(speed);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > waitTime;
    }
    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setClawMotor(0);
        timer.reset();
    }
}

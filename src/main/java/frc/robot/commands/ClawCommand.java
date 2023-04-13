package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawCommand extends CommandBase  {
    private final ClawSubsystem clawSubsystem;
    private final Timer timer = new Timer();
    private final double currentLimit;
    private final double delay;
    private final double speed;

    /**
     * Command that starts the intake, stopping at the specified current limit after the specified delay.
     * @param subsystem the claw subsystem
     * @param speed the speed to set the intake
     * @param currentLimit the limit in amps to stop the motor at
     * @param delay the time to wait until the current limit kicks in
     */
    public ClawCommand(ClawSubsystem subsystem, double speed, double currentLimit, double delay){
        clawSubsystem = subsystem;
        this.speed = speed;
        this.delay = delay;
        this.currentLimit = currentLimit;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        clawSubsystem.setClawMotor(speed);
        SmartDashboard.putBoolean("Claw State", true);
    }
    
    @Override
    public boolean isFinished() {
        if(timer.get() > delay)
            return clawSubsystem.getClawMotorCurrent() > currentLimit;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("Claw State", false);
        clawSubsystem.setClawMotor(0);
    }
}

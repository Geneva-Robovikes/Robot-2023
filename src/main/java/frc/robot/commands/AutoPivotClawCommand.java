package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class AutoPivotClawCommand extends CommandBase {
    private ClawSubsystem clawSubsystem;
    private double speed; 
    private double distance;

    /**
     * Command to rotate the claw to the specified distance.
     * @param subsystem the claw subsystem
     * @param speed the speed to rotate the claw at. Must be between -1.0 and 1.0
     * @param distance the distance to stop at in encoder units
     */
    public AutoPivotClawCommand (ClawSubsystem subsystem, double speed, double distance) {
        this.clawSubsystem = subsystem;
        this.speed = speed;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        clawSubsystem.resetPivotEncoder();
        clawSubsystem.setPivotMotor(speed);
    }
    
    @Override
    public void execute() {
        clawSubsystem.setPivotMotor(speed);
    }
    
    @Override
    public boolean isFinished() {
        return Math.abs(clawSubsystem.getPivotDistance()) > distance;
    }
    
    @Override
    public void end(boolean interrupted) {
        clawSubsystem.setPivotMotor(0);
    }
}

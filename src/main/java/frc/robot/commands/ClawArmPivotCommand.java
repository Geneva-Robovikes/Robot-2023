package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawArmPivotSubsystem;

public class ClawArmPivotCommand extends CommandBase{
    private ClawArmPivotSubsystem clawArmPivotSubsystem;
    private double speed;
    
    public ClawArmPivotCommand(ClawArmPivotSubsystem subsystem, double speed) {

        clawArmPivotSubsystem = subsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        clawArmPivotSubsystem.turnArmMotor(speed);
    }

    @Override
    public boolean isFinished() {
        if(clawArmPivotSubsystem.getSwitchState()) {
            System.out.println("here");
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        clawArmPivotSubsystem.turnArmMotor(0);
    }

}

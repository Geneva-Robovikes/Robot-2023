package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class FullArmCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final double speed;
    private boolean upperStopped;
    private boolean lowerStopped;

    public FullArmCommand(ArmSubsystem armSubsystem, double speed) {
        this.armSubsystem = armSubsystem;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        upperStopped = false;
        lowerStopped = false;
        armSubsystem.setUpperExensionMotor(speed);
        armSubsystem.setLowerExtensionMotor(speed);
    }

    @Override
    public void execute() {
        if(speed > 0) {
            if(armSubsystem.getUpperExensionTopState()) {
                armSubsystem.setUpperExensionMotor(0);
                upperStopped = true;
            }
            if(armSubsystem.getLowerExtensionTopState()) {
                armSubsystem.setLowerExtensionMotor(0);
                lowerStopped = true;
            }
        } else {
            if(armSubsystem.getUpperExensionBottomState()) {
                armSubsystem.setUpperExensionMotor(0);
                upperStopped = true;
            }
            if(armSubsystem.getLowerExtensionBottomState()) {
                armSubsystem.setLowerExtensionMotor(0);
                lowerStopped = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        if(upperStopped && lowerStopped) return true;
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setUpperExensionMotor(0);
        armSubsystem.setLowerExtensionMotor(0);
    }
}

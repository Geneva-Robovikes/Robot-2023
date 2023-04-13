package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class AutoClawArmPivotCommand extends CommandBase{
    private ArmSubsystem armSubsystem;
    private double speed;
    private double distance;

    /**
     * Command to pivot the claw arm to a specific distance.
     * @param subsystem the arm subsystem
     * @param speed the speed to rotate the arm. Must be between -1.0 and 1.0
     * @param distance the distance to rotate the arm to in encoder units
     */
    public AutoClawArmPivotCommand(ArmSubsystem subsystem, double speed, double distance) {
        this.armSubsystem = subsystem;
        this.speed = speed;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        armSubsystem.resetArmPivotEncoder();
        armSubsystem.setArmPivotMotor(speed);
    }

    @Override
    public void execute() {
        armSubsystem.setArmPivotMotor(speed);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(armSubsystem.getArmPivotPosition()) > distance;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setArmPivotMotor(0);
    }

}

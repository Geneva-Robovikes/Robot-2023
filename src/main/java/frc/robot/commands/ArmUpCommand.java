package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmUpCommand extends CommandBase {
    private ArmSubsystem armSubsystem;
    private double speed;
    private boolean isAuto;

    public ArmUpCommand (ArmSubsystem subsystem, double speed, boolean isAuto) {
        armSubsystem = subsystem;
        this.speed = speed;
        this.isAuto = isAuto;
        addRequirements(subsystem);

    }

    @Override
    public void initialize() {
        armSubsystem.setArmMotor(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setArmMotor(0);
    }
}

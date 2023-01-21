package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class PneumaticsCommand extends CommandBase{

    private final PneumaticsSubsystem pneumaticsSubsystem;
    
    public PneumaticsCommand(PneumaticsSubsystem subsystem) {
        pneumaticsSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        pneumaticsSubsystem.setSolenoid(kForward);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pneumaticsSubsystem.setSolenoid(kReverse);
    }
}

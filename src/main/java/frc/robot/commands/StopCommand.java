package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem;

public class StopCommand extends CommandBase {
    DriveSubsystem driveSubsystem;
    public StopCommand(DriveSubsystem drive) {
        driveSubsystem = drive;
        addRequirements(drive);
    }
    
    @Override
    public void execute() {
        driveSubsystem.stop();
        System.out.println("Stopping");
    }
}

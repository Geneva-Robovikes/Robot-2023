package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class StopCommand extends CommandBase {
    DriveSubsystem driveSubsystem;

    /**
     * Stops all the swerve modules.
     * @param drive the drive subsystem
     */
    public StopCommand(DriveSubsystem drive) {
        driveSubsystem = drive;
        addRequirements(drive);
    }
    
    @Override
    public void execute() {
        driveSubsystem.stop();
    }
}

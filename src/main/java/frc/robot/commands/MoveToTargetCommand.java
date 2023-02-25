package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class MoveToTargetCommand extends CommandBase{

    CameraSubsystem cameraSubsystem;
    DriveSubsystem drive;
    MoveToTargetCommand(CameraSubsystem cameraSubsystem, DriveSubsystem drive) {
        this.cameraSubsystem = cameraSubsystem;
        this.drive = drive;
    }

    @Override
    public void execute() {
        
    }

    
    
}

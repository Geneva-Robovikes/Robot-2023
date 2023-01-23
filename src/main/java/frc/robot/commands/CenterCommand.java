package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;

public class CenterCommand extends CommandBase{
    CameraSubsystem cameraSubsystem;
    DriveSubsystem driveSubsystem;
    double targetSkew;
    double yVelocity;
    double angularVelocity;
    PIDController controller;
    
    public CenterCommand(CameraSubsystem cameraSubsystem, DriveSubsystem driveSubsystem) {
        this.cameraSubsystem = cameraSubsystem;
        this.driveSubsystem = driveSubsystem;
        addRequirements(cameraSubsystem, driveSubsystem);
        controller = new PIDController(Constants.OperatorConstants.pGain, 0, Constants.OperatorConstants.dGain);
    }

    @Override
    public void execute() {
        if (cameraSubsystem.hasTargets()) {

            targetSkew = cameraSubsystem.getTargetSkew();

            //TODO: possibly change from 0? find slightly larger value that still works?
            //leave x as 0, only y and angular need to change over time - x is forward and backward;
            yVelocity = controller.calculate(targetSkew, 0);
            if (targetSkew > 0) {
                //move robot one way
                
                driveSubsystem.setModuleStatesFromSpeeds(0, -yVelocity, -Math.PI/2);
            }

            if (targetSkew < 0) {
                //move robot other way
                driveSubsystem.setModuleStatesFromSpeeds(0, yVelocity, Math.PI/2);
            }
        }
    }

    @Override
    public boolean isFinished() {
        if(controller.atSetpoint()) return true;
        else return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setModuleStatesFromSpeeds(0, 0, 0);
    }
}

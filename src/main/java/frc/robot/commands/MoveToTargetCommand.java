package frc.robot.commands;

import javax.naming.spi.DirStateFactory.Result;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class MoveToTargetCommand extends CommandBase{

    PIDController controller = new PIDController(6, .003, 0);

    CameraSubsystem cameraSubsystem;
    

    DriveSubsystem drive; 
    //double cameraHeight = Units.inchesToMeters(7);
    double cameraHeight = Units.inchesToMeters(4);
    //double targetHeight = Units.inchesToMeters(12.5);
    double targetHeight = Units.inchesToMeters(13.5);

    double goalRangeMeters = 0;
    public MoveToTargetCommand(CameraSubsystem cameraSubsystem, DriveSubsystem drive) {
        this.cameraSubsystem = cameraSubsystem;
        this.drive = drive;
    }

    @Override
    public void execute() {
        double forwardSpeed;
        if (cameraSubsystem.hasTargets()) {
            System.out.println("target!");
            //double distane = PhotonUtils.getDistanceToPose(new Pose2d(), new Pose2d());
            double range = PhotonUtils.calculateDistanceToTargetMeters(cameraHeight, targetHeight, 0, Units.degreesToRadians(cameraSubsystem.getTargetPitch())/*/1.46143445*/);
            System.out.println(range);
            //it LIED dont make it negative >:(
            //forwardSpeed = controller.calculate(range, goalRangeMeters);
            forwardSpeed = 0;
            // System.out.println(controller.calculate(range, goalRangeMeters));
        } else {
            forwardSpeed = 0;
            //System.out.println("no targets :(");
        }

        drive.setModuleStatesFromSpeeds(forwardSpeed, 0, 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        drive.setModuleStatesFromSpeeds(0, 0, 0, false);
    }    

    @Override
    public boolean isFinished() {
        if(controller.atSetpoint()) return true;
        else return false;
    }
    
}

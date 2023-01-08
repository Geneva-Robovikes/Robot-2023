// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  Gyro gyro;

  SwerveModule frontLeftModule = new SwerveModule(0, 1);
  SwerveModule frontRightModule = new SwerveModule(2, 3);
  SwerveModule backLeftModule = new SwerveModule(4, 5);
  SwerveModule backRightModule = new SwerveModule(6, 7);

  // Positions are based of of 25in square robot
  Translation2d frontLeftLocation = new Translation2d(0.318, 0.318);
  Translation2d frontRightLocation = new Translation2d(0.318, -0.318);
  Translation2d backLeftLocation = new Translation2d(-0.318, 0.318);
  Translation2d backRightLocation = new Translation2d(-0.318, -0.318);

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    kinematics, gyro.getRotation2d(), new SwerveModulePosition[] {
      frontRightModule.getPosition(),
      frontRightModule.getPosition(),
      backLeftModule.getPosition(), 
      backRightModule.getPosition()
    }, 
    new Pose2d(0, 0, new Rotation2d()));
  Pose2d currenPose2d;

  public DriveSubsystem() {}

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(), frontRightModule.getPosition(),
        backLeftModule.getPosition(), backRightModule.getPosition()
      });
  }

  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        if(isFirstPath){
          this.resetOdometry(traj.getInitialHolonomicPose());
        }
      }),
        new PPSwerveControllerCommand(
        traj, 
        this::getPose, // Pose supplier
        this.kinematics, // SwerveDriveKinematics
        new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
        new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        this::setModuleStates, // Module states consumer
        this // Requires this drive subsystem
      )
    );
  }

  void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), new SwerveModulePosition[] {
      frontRightModule.getPosition(),
      frontRightModule.getPosition(),
      backLeftModule.getPosition(), 
      backRightModule.getPosition()
    }, pose);
  }

  Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  void setModuleStates(SwerveModuleState[] moduleStates) {
    frontLeftModule.setSesiredState(moduleStates[0]);
    frontRightModule.setSesiredState(moduleStates[1]);
    backLeftModule.setSesiredState(moduleStates[2]);
    backRightModule.setSesiredState(moduleStates[3]);
  }
}
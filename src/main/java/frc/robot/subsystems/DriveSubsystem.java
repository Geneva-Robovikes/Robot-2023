// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16448_IMU;

public class DriveSubsystem extends SubsystemBase {
  ADIS16448_IMU gyro;
  CameraSubsystem[] cameras;

  SwerveModule frontLeftModule;
  SwerveModule frontRightModule;
  SwerveModule backLeftModule;
  SwerveModule backRightModule;

  // Positions are based of of 25in square robot
  Translation2d frontLeftLocation = new Translation2d(0.318, 0.318);
  Translation2d frontRightLocation = new Translation2d(0.318, -0.318);
  Translation2d backLeftLocation = new Translation2d(-0.318, 0.318);
  Translation2d backRightLocation = new Translation2d(-0.318, -0.318);

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
    kinematics, new Rotation2d(gyro.getGyroAngleZ()), new SwerveModulePosition[] {
      frontRightModule.getPosition(),
      frontRightModule.getPosition(),
      backLeftModule.getPosition(), 
      backRightModule.getPosition()
    }, 
    new Pose2d(0, 0, new Rotation2d()));
  Pose2d currenPose2d;

  public DriveSubsystem(int[] motorIndexes, CameraSubsystem[] cameras) {
    this.cameras = cameras;
    frontLeftModule = new SwerveModule(motorIndexes[0], motorIndexes[1]);
    frontRightModule = new SwerveModule(motorIndexes[2], motorIndexes[3]);
    backLeftModule = new SwerveModule(motorIndexes[4], motorIndexes[5]);
    backRightModule = new SwerveModule(motorIndexes[6], motorIndexes[7]);
  }

  @Override
  public void periodic() {
    odometry.update(new Rotation2d(gyro.getGyroAngleZ()),
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(), frontRightModule.getPosition(),
        backLeftModule.getPosition(), backRightModule.getPosition()
      }
    );

    for (CameraSubsystem cam : cameras) {
      Optional<EstimatedRobotPose> result = cam.getEstimatedGlobalPose(odometry.getEstimatedPosition());
      EstimatedRobotPose camPose = result.get();
      odometry.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
    }
  }

  public void setModuleStatesFromSpeeds(double xVelocity, double yVelocity, double angularVelocity) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, angularVelocity, new Rotation2d(gyro.getGyroAngleZ()));
    setModuleStates(kinematics.toSwerveModuleStates(speeds));
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(new Rotation2d(gyro.getGyroAngleZ()), 
      new SwerveModulePosition[] {
        frontRightModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(), 
        backRightModule.getPosition()
      }, pose
    );
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(gyro.getGyroAngleZ());
  }
  
  public void addCameraToOdometry(Optional<EstimatedRobotPose> estimatedPose, double timestamp) {
    odometry.addVisionMeasurement(currenPose2d, timestamp);
  }

  void setModuleStates(SwerveModuleState[] moduleStates) {
    frontLeftModule.setSesiredState(moduleStates[0]);
    frontRightModule.setSesiredState(moduleStates[1]);
    backLeftModule.setSesiredState(moduleStates[2]);
    backRightModule.setSesiredState(moduleStates[3]);
  }

}
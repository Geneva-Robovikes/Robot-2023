// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16448_IMU;

public class DriveSubsystem extends SubsystemBase {

  ADIS16448_IMU gyro = new ADIS16448_IMU();
  boolean isFieldCentric = true;

  SwerveModule frontLeftModule;
  SwerveModule frontRightModule;
  SwerveModule backLeftModule;
  SwerveModule backRightModule;

  // Positions are based of of 25in square robot
  Translation2d frontLeftLocation = new Translation2d(0.3048, 0.3048);
  Translation2d frontRightLocation = new Translation2d(0.3048, -0.3048);
  Translation2d backLeftLocation = new Translation2d(-0.3048, 0.3048);
  Translation2d backRightLocation = new Translation2d(-0.3048, -0.3048);

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
  SwerveDriveOdometry odometry;
  Pose2d currenPose2d;

  public DriveSubsystem(int[] motorIndexes) {
    frontLeftModule = new SwerveModule(motorIndexes[0], motorIndexes[1], true);
    frontRightModule = new SwerveModule(motorIndexes[2], motorIndexes[3], false);
    backLeftModule = new SwerveModule(motorIndexes[4], motorIndexes[5], true);
    backRightModule = new SwerveModule(motorIndexes[6], motorIndexes[7], false);
    gyro.calibrate();

    odometry = new SwerveDriveOdometry(
      kinematics, new Rotation2d(0), new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(), 
        backRightModule.getPosition()
      }, 
      new Pose2d(0, 0, new Rotation2d()));
  }

  public void updateOdometry() {
    odometry.update(new Rotation2d(gyro.getGyroAngleZ()),
    new SwerveModulePosition[] {
      frontLeftModule.getPosition(), frontRightModule.getPosition(),
      backLeftModule.getPosition(), backRightModule.getPosition()
    }
  );
  }

  void setFieldCentricDrive (boolean enabled) {
    isFieldCentric = enabled;
  }

  public void setModuleStatesFromSpeeds(double xVelocity, double yVelocity, double angularVelocity) {
    ChassisSpeeds speeds;
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, angularVelocity, new Rotation2d(gyro.getGyroAngleZ()));
    /*if(isFieldCentric) {
    } else {
      speeds = new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);
    }*/
    //System.out.println(speeds);
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
    return odometry.getPoseMeters();
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(gyro.getGyroAngleZ());
  }

  void setModuleStates(SwerveModuleState[] moduleStates) {
    frontLeftModule.setDesiredState(moduleStates[0]);
    frontRightModule.setDesiredState(moduleStates[1]);
    backLeftModule.setDesiredState(moduleStates[2]);
    backRightModule.setDesiredState(moduleStates[3]);
  }

  public CommandBase resetOdometryCommand() {
    return runOnce(() -> resetOdometry(new Pose2d(odometry.getPoseMeters().getTranslation(), new Rotation2d())));
  }

  public CommandBase setDriveModeCommand() {
    return runEnd(() -> setFieldCentricDrive(true), () -> setFieldCentricDrive(false));
  }
}
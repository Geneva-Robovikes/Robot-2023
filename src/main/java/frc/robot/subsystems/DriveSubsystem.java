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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {

  ADIS16448_IMU gyro = new ADIS16448_IMU();
  boolean isFieldCentric = true;

  // Positions are based of of 25in square robot
  Translation2d frontLeftLocation = new Translation2d(0.3048, 0.3048);
  Translation2d frontRightLocation = new Translation2d(0.3048, -0.3048);
  Translation2d backLeftLocation = new Translation2d(-0.3048, 0.3048);
  Translation2d backRightLocation = new Translation2d(-0.3048, -0.3048);

  SwerveModule frontLeftModule = new SwerveModule(0, 1, false, true);
  SwerveModule frontRightModule = new SwerveModule(2, 3, false, true);
  SwerveModule backLeftModule = new SwerveModule(4, 5, false, true);
  SwerveModule backRightModule = new SwerveModule(6, 7, false, true);  

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    kinematics, getRotation2d(), new SwerveModulePosition[] {
      frontLeftModule.getPosition(),
      frontRightModule.getPosition(),
      backLeftModule.getPosition(), 
      backRightModule.getPosition()
    }, 
    new Pose2d(0, 0, new Rotation2d())
  );

  public DriveSubsystem() {
    gyro.calibrate();
  }

  public void updateOdometry() {
    odometry.update(getRotation2d(),
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
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, angularVelocity, getRotation2d());
    //speeds = new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);
    /*if(isFieldCentric) {
    } else {
    }*/
    //System.out.println(speeds);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 3);
    setModuleStates(states);
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), 
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
    SmartDashboard.putNumber("Angle", gyro.getGyroAngleZ());
    return new Rotation2d(gyro.getGyroAngleZ() / 57.295779513);
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
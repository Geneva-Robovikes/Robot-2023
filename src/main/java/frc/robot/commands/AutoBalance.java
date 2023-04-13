// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalance extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Timer timer = new Timer();
  private final double tolerance;
  private final double balanceSpeed;
  private final double driveSpeed;
  private final double waitTime;
  private boolean isOnScale = false;
  private boolean balance = false;

  /**
   * Command to automaically balance on the charge station.
   * @param driveSubsystem the drive subsystem
   * @param balanceSpeed the speed when attemting to balance
   * @param driveSpeed the speed when approaching the charge station
   * @param tolerance the angle tolerance to consider balanced
   * @param waitTime the time to wait before switching from the drive ot balance speeds
   */
  public AutoBalance(DriveSubsystem driveSubsystem, double balanceSpeed, double driveSpeed,  double tolerance, double waitTime) {
    this.driveSubsystem = driveSubsystem;
    this.tolerance = tolerance;
    this.balanceSpeed = balanceSpeed;
    this.driveSpeed = driveSpeed;
    this.waitTime = waitTime;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    double currentYAngle = driveSubsystem.getGyroAngleY();
    SmartDashboard.putNumber("Gyro Y Angle", currentYAngle);
    SmartDashboard.putBoolean("Is On Scale", isOnScale);

    if(timer.get() > 7) {
      driveSubsystem.setModuleStatesFromSpeeds(0, 0, 0, true);
    }

    if(!isOnScale && currentYAngle < (tolerance + 1) && currentYAngle > -(tolerance + 1)) {
      driveSubsystem.setModuleStatesFromSpeeds(driveSpeed, 0, 0, true);
      return;
    } else {
      isOnScale = true;
      timer.start();
    }

    SmartDashboard.putNumber("Timer", timer.get());

    if(isOnScale && !balance) {
      driveSubsystem.setModuleStatesFromSpeeds(driveSpeed, 0, 0, true);
      if (timer.get() > waitTime) {
        balance = true;
        timer.stop();
      } else {
        return;
      }
    }

    if(-currentYAngle > tolerance) {
      driveSubsystem.setModuleStatesFromSpeeds(-balanceSpeed, 0, 0, true);
    } else if(-currentYAngle < -tolerance) {
      driveSubsystem.setModuleStatesFromSpeeds(balanceSpeed, 0, 0, true);
    } else {
      driveSubsystem.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

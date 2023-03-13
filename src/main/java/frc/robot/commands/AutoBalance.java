// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalance extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final double tolerance;
  private final double speed;

  public AutoBalance(DriveSubsystem driveSubsystem, double speed, double tolerance) {
    this.driveSubsystem = driveSubsystem;
    this.tolerance = tolerance;
    this.speed = speed;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    double currentYAngle = driveSubsystem.getGyroAngleY();

    if(currentYAngle > tolerance) {
      driveSubsystem.setModuleStatesFromSpeeds(speed, 0, 0, false);
    } else {
      driveSubsystem.setModuleStatesFromSpeeds(-speed, 0, 0, false);
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

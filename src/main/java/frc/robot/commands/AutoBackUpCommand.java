// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBackUpCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private double distance;
  private double speed;
  private double initialX;
  private boolean isFieldCentric;

  public AutoBackUpCommand(DriveSubsystem driveSubsystem, double speed, double distance, boolean isFieldCentric) {
    this.driveSubsystem = driveSubsystem;
    this.distance = distance;
    this.speed = speed;
    this.isFieldCentric = isFieldCentric;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    driveSubsystem.resetOdometry(new Pose2d());
    initialX = Math.abs(driveSubsystem.getPose().getX());
  }

  @Override
  public void execute() {
      driveSubsystem.setModuleStatesFromSpeeds(speed, 0, 0, isFieldCentric);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setModuleStatesFromSpeeds(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(driveSubsystem.getPose().getX()) - initialX > distance;
  }
}

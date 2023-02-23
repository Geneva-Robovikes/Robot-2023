// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalance extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private PIDController balancePID = new PIDController(1, 0, 0);

  public AutoBalance(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    double currentAngle = driveSubsystem.getAngleAroundFieldY();
    SmartDashboard.putNumber("Angle Around Field Y", currentAngle);
    SmartDashboard.putNumber("Balance Velocity", balancePID.calculate(Math.sin(currentAngle), 0));
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

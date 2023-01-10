// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class TeleopCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final CommandXboxController controller;

  //TODO: Test and get good values
  private final double minSpeedX = 0.1;
  private final double maxSpeedX = 1;
  private final double minSpeedY = 0.1;
  private final double maxSpeedY = 1;
  private final double minSpeedTheta = Math.PI / 16;
  private final double maxSpeedTheta = Math.PI / 2;

  public TeleopCommand(DriveSubsystem driveSubsystem, CommandXboxController controller) {
    this.driveSubsystem = driveSubsystem;
    this.controller = controller;
    addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {
    double x1 = controller.getLeftX();
    double y1 = controller.getLeftY();
    double x2 = controller.getRightX();

    double vX = x1 * maxSpeedX;
    double vY = y1 * maxSpeedY;
    double vTheta = x2 * maxSpeedTheta;

    if(vX < minSpeedX) vX = 0;
    if(vY < minSpeedY) vY = 0;
    if(vTheta < minSpeedTheta) vTheta = 0;

    driveSubsystem.setModuleStatesFromSpeeds(vX, vY, vTheta);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setModuleStatesFromSpeeds(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

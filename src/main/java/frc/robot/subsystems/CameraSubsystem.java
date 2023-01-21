// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  PhotonCamera camera;
  double[] cameraPosition;
  PhotonPipelineResult result;

  public CameraSubsystem(String cameraName, double[] cameraPosition) {
    camera = new PhotonCamera(cameraName);
    result = camera.getLatestResult();
    this.cameraPosition = cameraPosition;
  }

  @Override
  public void periodic() {
    result = camera.getLatestResult();
  }

  public boolean hasTargets() {
    return result.hasTargets();
  }

  public void setPipeline(int index) {
    camera.setPipelineIndex(index);
  }
}
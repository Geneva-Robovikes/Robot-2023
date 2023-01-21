// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  AprilTagFieldLayout tagLayout;
  PhotonCamera camera;
  Transform3d cameraPosition;
  PhotonPipelineResult result;
  PhotonPoseEstimator poseEstimator;

  /**
   * Creates a new camera with name, position, and pitch from the horizontal.
   * @param cameraName name of the camera on Photon Vision Dashboard.
   * @param cameraPosition Position of the camera from robot center.
   * @param cameraPitch
   */
  public CameraSubsystem(String cameraName, Transform3d cameraPosition) {
    camera = new PhotonCamera(cameraName);
    result = camera.getLatestResult();
    this.cameraPosition = cameraPosition;

    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    } catch (Exception e) {
      System.err.println("File cannot be loaded");
    }
    poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, cameraPosition);
  }

  @Override
  public void periodic() {
    result = camera.getLatestResult();
  }

  /**
   * Get whether the camera sees targets.
   * @return true if targets are found.
   */
  public boolean hasTargets() {
    return result.hasTargets();
    //result.getBestTarget().getSkew();
  }

  //added by alex
  public double getTargetSkew() {
    return result.getBestTarget().getSkew();
  }

  

  /**
   * Sets the camera's pipeline to the index's corresponding pipeline.
   * @return The index of the pipeline. 0 = Cone, 1 = Cube, 2 = Tag, 3 = Tape
   */
  public int getPipelineIndex() {
    return camera.getPipelineIndex();
  }

  /**
   * Sets the camera's pipeline to the index's corresponding pipeline.
   * @param index The index of the pipeline. 0 = Cone, 1 = Cube, 2 = Tag, 3 = Tape
   */
  public void setPipeline(int index) {
    camera.setPipelineIndex(index);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    poseEstimator.setReferencePose(prevEstimatedRobotPose);
    return poseEstimator.update();
  }
}
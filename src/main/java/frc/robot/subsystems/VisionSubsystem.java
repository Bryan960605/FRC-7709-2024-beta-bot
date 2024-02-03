// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new Visionsubsystem. */
  private final PhotonCamera photonVisioncamera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
  private Optional<Alliance> alliance = DriverStation.getAlliance();
  
  private final PIDController xMovePID = new PIDController(0, 0, 0);
  private final PIDController yMovePID = new PIDController(0, 0, 0);
  private final PIDController zMovePID = new PIDController(0, 0, 0);

  private double xValue;
  private double yValue;
  private double zValue;
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    PhotonPipelineResult result = photonVisioncamera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    boolean hasTarget = result.hasTargets();
  }
}

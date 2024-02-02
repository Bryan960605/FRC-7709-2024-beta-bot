// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  private final WPI_VictorSPX climeLeftMotor = new WPI_VictorSPX(56);
  private final WPI_VictorSPX climbRightMotor = new WPI_VictorSPX(45);

  private final Servo rightServo = new Servo(0);
  private final Servo leftServo = new Servo(9);

  public boolean climbDone = false;
  public ClimbSubsystem() {
    climbRightMotor.setInverted(true);
    climeLeftMotor.setInverted(false);

    climbRightMotor.setNeutralMode(NeutralMode.Brake);
    climeLeftMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void climbMove(double leftSpeed, double rightSpeed){
    if(climbDone == false){
      climeLeftMotor.set(leftSpeed);
      climbRightMotor.set(rightSpeed);
    }
    else{
      climeLeftMotor.set(0);
      climbRightMotor.set(0);
    }
  }

  public void climbComplete(){
    if(climbDone ==false){
      climbDone = true;
    leftServo.setAngle(90);
    rightServo.setAngle(-90);
    }
    else{
    climbDone = false;
    leftServo.setAngle(0);
    rightServo.setAngle(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

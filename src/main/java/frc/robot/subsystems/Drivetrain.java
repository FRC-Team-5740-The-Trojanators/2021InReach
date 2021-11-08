// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.*;



public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  private WPI_VictorSPX frontLeft = new WPI_VictorSPX(Constants.frontLeftCAN);
  private WPI_VictorSPX backLeft = new WPI_VictorSPX(Constants.backLeftCAN);
	private WPI_VictorSPX backRight = new WPI_VictorSPX(Constants.backRightCAN);
	private WPI_VictorSPX frontRight = new WPI_VictorSPX(Constants.frontRightCAN);

  public Drivetrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

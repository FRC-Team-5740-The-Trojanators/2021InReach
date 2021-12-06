// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.can.*;



public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  private WPI_VictorSPX frontLeft = new WPI_VictorSPX(Constants.frontLeftCAN);
  private WPI_VictorSPX backLeft = new WPI_VictorSPX(Constants.backLeftCAN);
	private WPI_VictorSPX backRight = new WPI_VictorSPX(Constants.backRightCAN);
	private WPI_VictorSPX frontRight = new WPI_VictorSPX(Constants.frontRightCAN);

  private final SpeedControllerGroup leftDriveGroup = new SpeedControllerGroup(frontLeft, backLeft);
  private final SpeedControllerGroup rightDriveGroup = new SpeedControllerGroup(frontRight, backRight);

  private final DifferentialDrive drive = new DifferentialDrive(leftDriveGroup, rightDriveGroup);
 
  public Drivetrain() 
  {
    VictorConfig();
  }

  public void VictorConfig()
  {
    frontLeft.configFactoryDefault();
    backRight.configFactoryDefault();
    frontRight.configFactoryDefault();
    backLeft.configFactoryDefault();

    leftDriveGroup.setInverted(true);
    leftDriveGroup.setInverted(false);

    drive.setRightSideInverted(false);
  }

  public void DriveMotors(double speed)
  {
    leftDriveGroup.set(speed);
    rightDriveGroup.set(speed);
  }

  public void arcadeDrive(final double throttle, final double turn)
  {
    drive.arcadeDrive(throttle, turn);
  }

  public void deadbandedArcadeDrive() {
		double throttle, turn;
    if (RobotContainer.m_driverController.getRawAxis(Constants.kRightStickX) > 0.1	|| RobotContainer.m_driverController.getRawAxis(Constants.kRightStickX) < -0.1) 
    {
      if (RobotContainer.m_driverController.getRawAxis(Constants.kRightStickX) < 0) 
      {
				throttle = -Math.sqrt(Math.abs(RobotContainer.m_driverController.getRawAxis(Constants.kRightStickX)));
      } else 
      {
				throttle = Math.sqrt(RobotContainer.m_driverController.getRawAxis(Constants.kRightStickX));
			}
    } else 
    {
			throttle = 0;
		}
    /* check deadband */

    if (RobotContainer.m_driverController.getRawAxis(Constants.kLeftStickY) > 0.2	|| RobotContainer.m_driverController.getRawAxis(Constants.kLeftStickY) < -0.2) 
    {
      if (RobotContainer.m_driverController.getRawAxis(Constants.kLeftStickY) < 0) 
      {
				turn = -Math.sqrt(Math.abs(RobotContainer.m_driverController.getRawAxis(Constants.kLeftStickY)));
      } else 
      {
				turn = Math.sqrt(RobotContainer.m_driverController.getRawAxis(Constants.kLeftStickY));
			}
    } else 
    {
			turn = 0;
		}
		arcadeDrive(throttle, -turn);
	}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

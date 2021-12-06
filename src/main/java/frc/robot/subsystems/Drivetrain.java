// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.can.*;



public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  //Defining motor controllers - we use VictorSPX
  private WPI_VictorSPX frontLeft = new WPI_VictorSPX(Constants.frontLeftCAN);
  private WPI_VictorSPX backLeft = new WPI_VictorSPX(Constants.backLeftCAN);
	private WPI_VictorSPX backRight = new WPI_VictorSPX(Constants.backRightCAN);
	private WPI_VictorSPX frontRight = new WPI_VictorSPX(Constants.frontRightCAN);

  //Grouping motor controllers into left and right
  private final SpeedControllerGroup leftDriveGroup = new SpeedControllerGroup(frontLeft, backLeft);
  private final SpeedControllerGroup rightDriveGroup = new SpeedControllerGroup(frontRight, backRight);

  //combines groups into one overall drivetrain
  private final DifferentialDrive drive = new DifferentialDrive(leftDriveGroup, rightDriveGroup);
 
  // The left-side drive encoder
  private final Encoder m_leftEncoder =
  new Encoder(
      Constants.kLeftEncoderPorts[0],
      Constants.kLeftEncoderPorts[1],
      Constants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder m_rightEncoder =
  new Encoder(
     Constants.kRightEncoderPorts[0],
     Constants.kRightEncoderPorts[1],
     Constants.kRightEncoderReversed);

  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  public Drivetrain() 
  {
    VictorConfig();

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  public void VictorConfig()  //resets factory defaults and inverts motors
  { 
    frontLeft.configFactoryDefault();
    backRight.configFactoryDefault();
    frontRight.configFactoryDefault();
    backLeft.configFactoryDefault();

    leftDriveGroup.setInverted(true);
    leftDriveGroup.setInverted(false);

    drive.setRightSideInverted(false);
  }

  public void DriveMotors(double speed) //the parameter speed is passed through the drive groups
  {
    leftDriveGroup.set(speed);
    rightDriveGroup.set(speed);
  }

  public void arcadeDrive(final double throttle, final double turn) //doubles throttle and turn are passed through arcadeDrive
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
  public void periodic() 
  {
    // This method will be called once per scheduler run
     // Update the odometry in the periodic block
     m_odometry.update(
      m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  public Pose2d getPose() 
  {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() 
  {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }
  
  public void resetOdometry(Pose2d pose) 
  {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
  
  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() 
  {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() 
  {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() 
  {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() 
  {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) 
  {
    drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() 
  {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() 
  {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() 
  {
    return -m_gyro.getRate();
  }
}


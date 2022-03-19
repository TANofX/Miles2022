// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.

  private static DriveSubsystem instance;
  private MotorControllerGroup m_leftMotors;
  private TalonFXSensorCollection m_leftSensor;

  // The motors on the right side of the drive.
  private MotorControllerGroup m_rightMotors;
  private TalonFXSensorCollection m_rightSensor;

  // The robot's drive
  private DifferentialDrive m_drive;

  // The left-side drive encoder
  // private final Encoder m_leftEncoder = new Encoder(
  // DriveConstants.kLeftEncoderPorts[0],
  // DriveConstants.kLeftEncoderPorts[1],
  // DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  // private final Encoder m_rightEncoder = new Encoder(
  // DriveConstants.kRightEncoderPorts[0],
  // DriveConstants.kRightEncoderPorts[1],
  // DriveConstants.kRightEncoderReversed);

 

  /** Creates a new DriveSubsystem. */
  private DriveSubsystem() {
    WPI_TalonFX leftMotor1 = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
    WPI_TalonFX leftMotor2 = new WPI_TalonFX(DriveConstants.kLeftMotor2Port);
    configureDriveMotor(leftMotor1);
    configureDriveMotor(leftMotor2);
    m_leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2);

    m_leftSensor = leftMotor1.getSensorCollection();

    WPI_TalonFX rightMotor1 = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
    WPI_TalonFX rightMotor2 = new WPI_TalonFX(DriveConstants.kRightMotor2Port);
    configureDriveMotor(rightMotor1);
    configureDriveMotor(rightMotor2);
    m_rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2);
    m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

    m_rightSensor = rightMotor1.getSensorCollection();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
  }

  private void configureDriveMotor(WPI_TalonFX motor) {
    motor.configVoltageCompSaturation(9.5);
    motor.enableVoltageCompensation(true);
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 
                                                                        Constants.DriveConstants.kCurrentLimit, 
                                                                        Constants.DriveConstants.kThresholdCurrent, 
                                                                        Constants.DriveConstants.kThresholdTimeout));
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configOpenloopRamp(0.25);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Drive sensor position", m_leftSensor.getIntegratedSensorPosition());
    SmartDashboard.putNumber("Right Drive sensor position", m_rightSensor.getIntegratedSensorPosition());
    
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        m_leftSensor.getIntegratedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse,
        m_rightSensor.getIntegratedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse);
  }

  public double getWheelPosition() {
    return ((m_leftSensor.getIntegratedSensorPosition() - m_rightSensor.getIntegratedSensorPosition()) / 2) * DriveConstants.kEncoderDistancePerPulse;
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void curvatureDrive(double fwd, double rot) {
    boolean isQuickTurn = Math.abs(fwd) <= Constants.DriveConstants.kMinimumTurnRate;
    m_drive.curvatureDrive(fwd, rot, isQuickTurn);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftSensor.setIntegratedSensorPosition(0.0, 0);
    m_rightSensor.setIntegratedSensorPosition(0.0, 0);
  }



  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftSensor.getIntegratedSensorPosition() * DriveConstants.kEncoderDistancePerPulse
        + m_rightSensor.getIntegratedSensorPosition() * DriveConstants.kEncoderDistancePerPulse) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  // public Encoder getLeftEncoder() {
  // return m_leftEncoder;
  // }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  // public Encoder getRightEncoder() {
  // return m_rightEncoder;
  // }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public static DriveSubsystem getInstance() {
    if (instance == null) {
      instance = new DriveSubsystem();
    }
    return instance;
  }
}

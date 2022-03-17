// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private Shooter() {
    shooterMotor = new WPI_TalonFX(Constants.PRIMARY_SHOOTER_ID);
    configureFalcon(shooterMotor);
    shooterFollower = new WPI_TalonFX(Constants.SECONDARY_SHOOTER_ID);
    configureFalcon(shooterFollower);
    shooterFollower.follow(shooterMotor);
    shooterFollower.setInverted(InvertType.OpposeMaster);
    //to do: get correct can ID
   

  }
  private void configureFalcon(WPI_TalonFX shooterMotor) {
    shooterMotor.setNeutralMode(NeutralMode.Coast);
    shooterMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.SHOOTER_CURRENT_LIMIT, Constants.SHOOTER_THRESHOLD_CURRENT, Constants.SHOOTER_THRESHOLD_TIMEOUT));
    shooterMotor.configClosedloopRamp(0.1);
    shooterMotor.selectProfileSlot(0,0);
    shooterMotor.configVoltageCompSaturation(9);
    shooterMotor.enableVoltageCompensation(true);

    shooterMotor.config_kF(0, Constants.SHOOTER_F, 0);
    shooterMotor.config_kP(0, Constants.SHOOTER_P, 0);
    shooterMotor.config_kI(0, Constants.SHOOTER_I, 0);

    shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,0);
    shooterMotor.config_IntegralZone(0, 5.0 * Constants.SHOOTER_SPIN_ERROR);

    shooterMotor.setSelectedSensorPosition(0);
  }
/** change angles of shooter?
 * Does shooter wheels have the same speed or is it different?
 * Is the shooter folded in like the intake is? (will it interfere with climber)
 * How do we get ball up the shooter?
 * Falcon motor for shooter

 */

private WPI_TalonFX shooterMotor;
private WPI_TalonFX shooterFollower;
private ShooterSpeeds targetShooterSpeeds = ShooterSpeeds.OFF;
private static Shooter shooterInstance;





  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startShooter(ShooterSpeeds SpeedToShoot) {
    shooterMotor.set(ControlMode.Velocity, SpeedToShoot.getMotorSpeed());
    targetShooterSpeeds = SpeedToShoot;
   

    
  }

  public void stopShooter() {
    shooterMotor.set(ControlMode.PercentOutput, 0);
    targetShooterSpeeds = ShooterSpeeds.OFF;

  }

  public double getShooterOutput() {
    return shooterMotor.get();
  }

  public double getShooterSpeed() {
    return shooterMotor.getSelectedSensorVelocity(0);
  }

  public boolean correctSpeed() {
  
   if(Math.abs(targetShooterSpeeds.getMotorSpeed() - getShooterSpeed()) < Constants.SHOOTER_SPIN_ERROR) {
     return true;
   }
  
  return false; 

  }
  public ShooterSpeeds getTargetSpeed() {
    return targetShooterSpeeds;

  
  }
  public static Shooter getInstance() {
    if(shooterInstance == null) {
      shooterInstance = new Shooter();
    }
    return shooterInstance;
  }
}

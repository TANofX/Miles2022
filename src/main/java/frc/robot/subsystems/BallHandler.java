// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.schedulers.ConcurrentScheduler;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.commands.DefaultBallHandler;
import frc.robot.Constants;
import frc.robot.util.BallSensor;

public class BallHandler extends SubsystemBase {
  public enum HandlerState {
    EMPTY,
    ONEBALLINTAKE,
    ONEBALLREADY,
    TWOBALLINTAKE,
    TWOBALLREADY,
    ONEBALLRESET,
    ONEBALLSHOOT,
    ONEBALLREADYTOSHOOT,
    TWOBALLSHOOT,
    UNKNOWN, ONEBALLSHOT,
  }

  /** Creates a new BallHandler. */
  private CANSparkMax transitMotor;
  private BallSensor firstSensor; // far left (near intake)
  private BallSensor secondSensor;
  private BallSensor thirdSensor;
  private HandlerState currentState = HandlerState.EMPTY;
  private static BallHandler handlerInstance;
  private HandlerState targetState = HandlerState.EMPTY;
  private boolean shooterMode = false;
  private int numberOfBalls = 0;

  public BallHandler() {

    firstSensor = new BallSensor(Constants.HANDLERSENSOR_1_PORT);
    secondSensor = new BallSensor(Constants.HANDLERSENSOR_2_PORT);
    thirdSensor = new BallSensor(Constants.HANDLERSENSOR_3_PORT);
    // setDefaultCommand(new DefaultBallHandler());
    transitMotor = new CANSparkMax(Constants.TRANSIT_MOTOR_ID, MotorType.kBrushless);
    transitMotor.setSmartCurrentLimit(29, 10);
    transitMotor.setOpenLoopRampRate(0.05);
    transitMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * needs to extend (intake bar)
   * stop and start intake bar (spinning)
   * intake bar needs to spin in both directions (avoid jams)
   * start and stop lower wheels involved with intake/storage
   * need transit motors and control in both directions
   * Reverse for getting rid of ball(what has to be reversed)
   * 1 Neo motor for intake only -Spark = controller
   * 1 Neo motors for transit (intake)
   * 2 pistons to bring intake up and down
   * Retractable intake bar
   * color sensor, 4? sensors to detect ball
   * QUESTIONS:
   * How independent are the motors?
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    stateMachine();
    SmartDashboard.putString("HandlerState", currentState.name());
    SmartDashboard.putBoolean("ShooterMode", shooterMode);
    SmartDashboard.putNumber("number of Cargo", this.ballsInRobot());
    SmartDashboard.putBoolean("Ready for Intake", this.readyForIntake());
  }

  public void moveTransitMotor(double velocity) {
    transitMotor.set(velocity);
  }

  public void extake() {
    transitMotor.set(-Constants.TRANSIT_MOTOR_SPEED);
  }

  public void stopTransitMotor() {
    transitMotor.set(0);
  }

  public int ballsInRobot() {

    switch (currentState) {
      case ONEBALLINTAKE:
      case ONEBALLREADY:
      case ONEBALLREADYTOSHOOT:
      case TWOBALLSHOOT:
      case ONEBALLSHOT:
        numberOfBalls = 1;
        break;
      case TWOBALLINTAKE:
      case TWOBALLREADY:
        numberOfBalls = 2;
        break;
      default:
        numberOfBalls = 0;
    }

    return numberOfBalls;

  }

  public boolean readyForIntake() {
    switch (currentState) {
      case EMPTY:
      case ONEBALLREADY:
        return true;
      default:
        return false;
    }

  }

  public HandlerState getState() {
    return currentState;
  }

  public boolean shooterMode(boolean activate) {
    if (activate == true) {
      if ((currentState == HandlerState.ONEBALLREADY) || (currentState == HandlerState.TWOBALLREADY)) {
        shooterMode = activate;
        return true;
      } else {
        return shooterMode;
      }
    }
    shooterMode = activate;
    return true;
  }

  public boolean shooterActivation() {
    return shooterMode;
  }

  public void shoot() {
    if (currentState == HandlerState.ONEBALLREADYTOSHOOT) {
      currentState = HandlerState.ONEBALLSHOOT;
    }
    if (currentState == HandlerState.TWOBALLREADY) {
      currentState = HandlerState.TWOBALLSHOOT;
    }
  }

  public void runHandler() {
    if (shooterMode) {
      switch (currentState) {
        case ONEBALLSHOOT:
        case TWOBALLSHOOT:
          moveTransitMotor(Constants.TRANSIT_MOTOR_SPEED);
          break;
        case ONEBALLREADY:
        case ONEBALLSHOT:
          moveTransitMotor(Constants.TRANSIT_MOTOR_SPEED);
          break;
        default:
          stopTransitMotor();
      }

    } else {
      switch (currentState) {
        case ONEBALLINTAKE:
        case TWOBALLINTAKE:
          moveTransitMotor(Constants.TRANSIT_MOTOR_SPEED);
          break;
        case ONEBALLRESET:
          moveTransitMotor(-Constants.TRANSIT_MOTOR_SPEED);
          break;
        default:
          stopTransitMotor();
      }

    }
  }

  public boolean atState() {
    return (currentState == targetState);
  }

  private void stateMachine() {
    if (shooterMode == false) {
      switch (currentState) {
        case ONEBALLREADYTOSHOOT:
        case ONEBALLSHOOT:
        case ONEBALLSHOT:
          currentState = HandlerState.ONEBALLRESET;
          break;
        default:

      }

      if (firstSensor.isTriggered() && (currentState == HandlerState.EMPTY)) {
        currentState = HandlerState.ONEBALLINTAKE;
      }
      if (secondSensor.isTriggered() && (currentState == HandlerState.ONEBALLINTAKE)) {
        currentState = HandlerState.ONEBALLREADY;
      }
      if (firstSensor.isTriggered() && (currentState == HandlerState.ONEBALLREADY)) {
        currentState = HandlerState.TWOBALLINTAKE;
      }
      if (thirdSensor.isTriggered() && (currentState == HandlerState.TWOBALLINTAKE)) {
        currentState = HandlerState.TWOBALLREADY;
      }
      if (firstSensor.isTriggered() && (currentState == HandlerState.ONEBALLRESET)) {
        currentState = HandlerState.ONEBALLINTAKE;
      }
    } else {
      if (thirdSensor.isTriggered()
          && ((currentState == HandlerState.ONEBALLREADY) || (currentState == HandlerState.ONEBALLSHOT))) {
        currentState = HandlerState.ONEBALLREADYTOSHOOT;
      }
      if (!thirdSensor.isTriggered() && (currentState == HandlerState.ONEBALLSHOOT)) {
        currentState = HandlerState.EMPTY;
      }
      if (!thirdSensor.isTriggered() && (currentState == HandlerState.TWOBALLSHOOT)) {
        currentState = HandlerState.ONEBALLSHOT;
      }

    }
  }

  public static BallHandler getInstance() {
    if (handlerInstance == null) {
      handlerInstance = new BallHandler();
    }
    return handlerInstance;
  }

  public void reset() {
    currentState = HandlerState.EMPTY;
  }
}

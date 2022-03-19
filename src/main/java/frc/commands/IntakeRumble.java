// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import org.opencv.objdetect.CascadeClassifier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.RumbleRoar;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.BallHandler.HandlerState;

public class IntakeRumble extends CommandBase {
  HandlerState handlerstate;
  Timer timer;
  Boolean isRumbleHappening = false;
  boolean oneBallRumble;
  boolean twoBallRumble;
  private static final double RUMBLE_STRENGTH = 0.5;
  private static final double RUMBLE_DURATION = 0.15;
  
  boolean rumblePause;

  private enum RumbleState {
    NOT_RUMBLING,
    ONE_BALL_RUMBLE,
    TWO_BALL_RUMBLE_1,
    TWO_BALL_RUMBLE_2,
    RUMBLE_PAUSE
  }

  RumbleState currentState;

  /** Creates a new IntakeRumble. */
  public IntakeRumble() {
    timer = new Timer();
    addRequirements(RumbleRoar.getInstance());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isRumbleHappening = false;
    oneBallRumble = false;
    twoBallRumble = false;
    rumblePause = false;
    handlerstate = BallHandler.getInstance().getState();
    currentState = RumbleState.NOT_RUMBLING;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    HandlerState newState = BallHandler.getInstance().getState();
    switch (currentState) {
      case NOT_RUMBLING:
        if ((newState == HandlerState.ONEBALLINTAKE) && (handlerstate == HandlerState.EMPTY)) {
          currentState = RumbleState.ONE_BALL_RUMBLE;
          startRumble();
        }

        if ((newState == HandlerState.TWOBALLINTAKE) && (handlerstate == HandlerState.ONEBALLREADY)) {
          currentState = RumbleState.TWO_BALL_RUMBLE_1;
          startRumble();
        }
        break;
      case ONE_BALL_RUMBLE:
        if (timer.hasElapsed(RUMBLE_DURATION)) {
          currentState = RumbleState.NOT_RUMBLING;
          stopRumble();
          timer.stop();
        }
        break;
      case TWO_BALL_RUMBLE_1:
        if (timer.hasElapsed(RUMBLE_DURATION)) {
          currentState = RumbleState.RUMBLE_PAUSE;
          stopRumble();
          timer.reset();
          timer.start();
        }
        break;
      case RUMBLE_PAUSE:
        if (timer.hasElapsed(RUMBLE_DURATION)) {
          currentState = RumbleState.TWO_BALL_RUMBLE_2;
          startRumble();
        }
        break;
      case TWO_BALL_RUMBLE_2:
        if (timer.hasElapsed(RUMBLE_DURATION)) {
          currentState = RumbleState.NOT_RUMBLING;
          stopRumble();
          timer.stop();
        }
        break;
    }

    handlerstate = newState;
  }

  private void stopRumble() {
    RumbleRoar.getInstance().stopRumble();
  }

  private void startRumble() {
    isRumbleHappening = true;
    timer.reset();
    timer.start();
    RumbleRoar.getInstance().startRumble(RUMBLE_STRENGTH);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

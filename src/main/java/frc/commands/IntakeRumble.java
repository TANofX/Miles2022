// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.BallHandler.HandlerState;

public class IntakeRumble extends CommandBase {
  HandlerState handlerstate;
  Timer timer;
  Boolean isRumbleHappening = false;
  /** Creates a new IntakeRumble. */
  public IntakeRumble() {
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isRumbleHappening = false;
    handlerstate = BallHandler.getInstance().getState();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    HandlerState newState = BallHandler.getInstance().getState();
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

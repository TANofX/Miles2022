// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Climber;

public class CalibrateRobot extends CommandBase {
  /** Creates a new CalibrateClimber. */
  
  public CalibrateRobot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Climber.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    BallHandler.getInstance().reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Climber.getInstance().calibrateClimber());
  }
}

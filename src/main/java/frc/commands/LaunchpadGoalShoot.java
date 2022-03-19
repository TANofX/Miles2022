// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterSpeeds;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LaunchpadGoalShoot extends CommandBase {
  public LaunchpadGoalShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Shooter.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Shooter.getInstance().startShooter(ShooterSpeeds.LAUNCHPADGOAL);
    
  }

  public boolean isFinished() {
    return BallHandler.getInstance().shooterMode(true);
  }
}

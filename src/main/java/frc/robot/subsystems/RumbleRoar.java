// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RumbleRoar extends SubsystemBase {
  XboxController xbox;
  private static RumbleRoar rumble;
  public static RumbleRoar getInstance(){
    if (rumble == null){
      rumble = new RumbleRoar();
    }
    return rumble;
  } 

  public void setXbox(XboxController jackson) {
    xbox = jackson;

  }

  public void startRumble() {
    xbox.setRumble(RumbleType.kLeftRumble, 1);
    xbox.setRumble(RumbleType.kRightRumble, 1);
  }

  public void stopRumble() {
    xbox.setRumble(RumbleType.kRightRumble, 0);
    xbox.setRumble(RumbleType.kLeftRumble, 0);
  }

  
  /** Creates a new RumbleRoar. */
  public RumbleRoar() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

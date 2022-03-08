// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import commands.CalibrateRobot;
import commands.CancelClimber;
import commands.MoveRachelWithJoystick;
import commands.SetClimberState;
import commands.ToggleGabeClaw;
import commands.ToggleRachelReach;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberState;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.commands.ClearBallHandler;
import frc.commands.CloseHighGoalShoot;
import frc.commands.DefaultBallHandler;
import frc.commands.DriveXinches;
import frc.commands.FarHighGoalShoot;
import frc.commands.LaunchpadGoalShoot;
import frc.commands.LowGoalShoot;
import frc.commands.RunIntake;
import frc.commands.ShootAll;
import frc.commands.ShootOne;
import frc.commands.StopShooter;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.HatSwitchButton;
import frc.robot.util.JoyStickAxisButton;
import frc.robot.util.JoystickUtils;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  final Joystick m_stick = new Joystick(Constants.FLIGHTSTICK_PORT);
  final XboxController m_xbox = new XboxController(Constants.XBOX_PORT);

  final JoystickButton toggleGabe = new JoystickButton(m_stick, Constants.FLIGHTSTICK_GABE_PNEUMATICS_BUTTON);
  final JoystickButton toggleRachel = new JoystickButton(m_stick, Constants.FLIGHTSTICK_RACHEL_REACH_PNEUMATICS_BUTTON);
  // final JoystickButton highClimb = new JoystickButton(m_stick,
  // Constants.CLIMB_HIGH_BAR);
  // final JoystickButton traversalClimb = new JoystickButton(m_stick,
  // Constants.CLIMB_TRAVERSAL_BAR);

  private JoystickButton stopShooterButton = new JoystickButton(m_stick, Constants.FLIGHTSTICK_SHOOTER_STOP_BUTTON);
  private JoystickButton lowGoalShootButton = new JoystickButton(m_stick, Constants.FLIGHTSTICK_LOW_SHOT_BUTTON);
  private JoystickButton closeHighGoalButton = new JoystickButton(m_stick, Constants.FLIGHTSTICK_HIGH_GOAL_BUTTON);
  private JoystickButton farHighGoalButton = new JoystickButton(m_stick, Constants.FLIGHTSTICK_HIGH_GOAL_HARD_BUTTON);
  private JoystickButton launchpadGoalShoot = new JoystickButton(m_stick, Constants.FLIGHTSTICK_FULL_POWER_BUTTON);
  private JoystickButton shootOne = new JoystickButton(m_stick, Constants.FLIGHTSTICK_SHOOT_ONE_BUTTON);
  private JoystickButton shootAll = new JoystickButton(m_stick, Constants.FLIGHTSTICK_SHOOT_ALL_BUTTON);

  // final JoystickButton cancelClimber = new JoystickButton(m_stick,
  // Constants.CANCEL_CLIMBER);
  final JoystickButton calibrateClimber = new JoystickButton(m_stick, Constants.FLIGHTSTICK_CALIBRATE_CLIMBER_BUTTON);
  final JoystickButton lockRachelMoveJoystick = new JoystickButton(m_stick,
      Constants.FLIGHTSTICK_LOCK_RACHEL_MOVE_JOYSTICK_BUTTON);

  // private final HatSwitchButton prepareClimbButton =
  // JoystickUtils.getHatSwitchButton(m_stick,
  // Constants.FLIGHTSTICK_HAT_PREPARE_TO_CLIMB_DIRECTION);
  // private final HatSwitchButton simpleClimbButton =
  // JoystickUtils.getHatSwitchButton(m_stick,
  // Constants.FLIGHTSTICK_HAT_SIMPLE_CLIMB_DIRECTION);

  private final JoyStickAxisButton runIntake = new JoyStickAxisButton(m_xbox, Constants.XBOX_RUNINTAKE_BUTTON);
  private JoystickButton prepareClimbXbox = new JoystickButton(m_xbox, XboxController.Button.kA.value);
  private JoystickButton clearShooterXbox = new JoystickButton(m_xbox, XboxController.Button.kStart.value);
  private JoystickButton highBarXbox = new JoystickButton(m_xbox, XboxController.Button.kX.value);
  private JoystickButton traversalXbox = new JoystickButton(m_xbox, XboxController.Button.kB.value);
  private JoystickButton toggleColorSensor = new JoystickButton(m_xbox, XboxController.Button.kBack.value);
  private JoystickButton simpleClimbXbox = new JoystickButton(m_xbox, XboxController.Button.kY.value);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Climber.getInstance();

    // Configure the button bindings
    BallHandler.getInstance().setDefaultCommand(new DefaultBallHandler());
    Intake.getInstance();
    Shooter.getInstance();
    // Creates UsbCamera and MjpegServer [1] and connects them
    CameraServer.startAutomaticCapture();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    toggleGabe.whenPressed(new ToggleGabeClaw());
    toggleRachel.whenPressed(new ToggleRachelReach());

    // highClimb.whenPressed(getHighClimbCommand());
    // traversalClimb.whenPressed(getTraversalClimbCommand());

    // cancelClimber.whenPressed(new CancelClimber());
    calibrateClimber.whenPressed(new CalibrateRobot());
    clearShooterXbox.whenPressed(new ClearBallHandler());

    lockRachelMoveJoystick.whileHeld(new MoveRachelWithJoystick(Climber.getInstance(), m_stick));

    DriveSubsystem.getInstance().setDefaultCommand(new RunCommand(
        () -> DriveSubsystem.getInstance().curvatureDrive(
            -1.0 * JoystickUtils.scaleDeadband(m_xbox.getLeftY(), Constants.XBOX_DEADBAND)
                / (m_xbox.getLeftTriggerAxis() + 1.0),
            JoystickUtils.scaleDeadband(m_xbox.getRightX(), Constants.XBOX_DEADBAND)
                / (m_xbox.getLeftTriggerAxis() + 1.0)),
        DriveSubsystem.getInstance()));

    stopShooterButton.whenPressed(new StopShooter());
    lowGoalShootButton.whenPressed(new LowGoalShoot());
    closeHighGoalButton.whenPressed(new CloseHighGoalShoot());
    farHighGoalButton.whenPressed(new FarHighGoalShoot());
    launchpadGoalShoot.whenPressed(new LaunchpadGoalShoot());
    shootOne.whenPressed(new ShootOne());
    shootAll.whenPressed(new ShootAll());

    runIntake.whileActiveContinuous(new RunIntake());

    prepareClimbXbox.whenPressed(new SetClimberState(ClimberState.BABYS_FIRST_REACH));
    highBarXbox.whenPressed(getHighClimbCommand());
    traversalXbox.whenPressed(getTraversalClimbCommand());
    toggleColorSensor.whenPressed(new InstantCommand(() -> Intake.getInstance().toggleColorSensor()));
    simpleClimbXbox.whenPressed(getSimpleClimbCommand());
    // prepareClimbButton.whenPressed(new
    // SetClimberState(ClimberState.BABYS_FIRST_REACH));
    // simpleClimbButton.whenPressed(getSimpleClimbCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SequentialCommandGroup( new CalibrateRobot(),
                                       new WaitCommand(1.0),
                                       new CloseHighGoalShoot(),
                                       new ShootAll(),
                                       new WaitCommand(0.5),
                                       new DriveXinches(90.0)
    );
  }

  private Command getHighClimbCommand() {
    return new SequentialCommandGroup(
        new SetClimberState(ClimberState.BABYS_FIRST_REACH),
        new SetClimberState(ClimberState.SUCCESSFULL_PULL_UP),
        new SetClimberState(ClimberState.SUCCESSFULL_HANG),
        new SetClimberState(ClimberState.T_REX_REACH),
        new SetClimberState(ClimberState.BRONTOSAURUS_REACHING),
        new SetClimberState(ClimberState.REACH_PULL),
        new WaitCommand(1.5),
        new SetClimberState(ClimberState.REACH_CAUGHT),
        new SetClimberState(ClimberState.TRUST_FALL),
        new InstantCommand(() -> Climber.getInstance().stopRachel(), Climber.getInstance()));
  }

  private Command getTraversalClimbCommand() {
    return new SequentialCommandGroup(
        new SetClimberState(ClimberState.BABYS_FIRST_REACH),
        new SetClimberState(ClimberState.SUCCESSFULL_PULL_UP),
        new SetClimberState(ClimberState.SUCCESSFULL_HANG),
        new SetClimberState(ClimberState.T_REX_REACH),
        new SetClimberState(ClimberState.BRONTOSAURUS_REACHING),
        new SetClimberState(ClimberState.REACH_PULL),
        new WaitCommand(1.5),
        new SetClimberState(ClimberState.REACH_CAUGHT),
        new SetClimberState(ClimberState.TRUST_FALL),
        new WaitCommand(1.5),
        new SetClimberState(ClimberState.SUCCESSFULL_PULL_UP),
        new SetClimberState(ClimberState.SUCCESSFULL_HANG),
        new SetClimberState(ClimberState.T_REX_REACH),
        new SetClimberState(ClimberState.BRONTOSAURUS_REACHING),
        new SetClimberState(ClimberState.REACH_PULL),
        new WaitCommand(1.5),
        new SetClimberState(ClimberState.REACH_CAUGHT),
        new SetClimberState(ClimberState.TRUST_FALL),
        new InstantCommand(() -> Climber.getInstance().stopRachel(), Climber.getInstance()));
  }

  private Command getSimpleClimbCommand() {
    return new SequentialCommandGroup(
        new SetClimberState(ClimberState.BABYS_FIRST_REACH),
        new SetClimberState(ClimberState.SUCCESSFULL_PULL_UP),
        new SetClimberState(ClimberState.SUCCESSFULL_HANG),
        new InstantCommand(() -> Climber.getInstance().stopRachel(), Climber.getInstance()));
  }
}

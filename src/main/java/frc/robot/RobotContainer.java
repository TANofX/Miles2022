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
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.Climber.RachelExtensionStates;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.commands.ClearBallHandler;
import frc.commands.CloseHighGoalShoot;
import frc.commands.DefaultBallHandler;
import frc.commands.DriveXinches;
import frc.commands.FarHighGoalShoot;
import frc.commands.IntakeRumble;
import frc.commands.LaunchpadGoalShoot;
import frc.commands.LowGoalShoot;
import frc.commands.RunIntake;
import frc.commands.ShootAll;
import frc.commands.ShootOne;
import frc.commands.StopShooter;
import frc.robot.subsystems.BallHandler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RumbleRoar;
import frc.robot.subsystems.Shooter;
import frc.robot.util.HatSwitchButton;
import frc.robot.util.JoyStickAxisButton;
import frc.robot.util.JoystickUtils;
import frc.robot.util.logging.RobotLogging;

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
  private JoystickButton traversalFromHighStick = new JoystickButton(m_stick, Constants.FLIGHTSTICK_HIGH_TO_TRAVERSAL);
  private JoystickButton highFromMidXbox = new JoystickButton(m_xbox, Constants.XBOX_MID_TO_HIGH);
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
    UsbCamera c = CameraServer.startAutomaticCapture();
    c.setResolution(320, 240);

    configureLogging();
    configureButtonBindings();
    RumbleRoar.getInstance().setXbox(m_xbox);
    RumbleRoar.getInstance().setDefaultCommand(new IntakeRumble());
  }

  private void configureLogging() {
    // Climber Logging
    RobotLogging.getInstance().registerEnumHistorian("Climber State", () -> Climber.getInstance().getCurrentState());
    RobotLogging.getInstance().registerEnumHistorian("Rachel Height", () -> Climber.getInstance().getExtensionState());
    RobotLogging.getInstance().registerImmediate("Rachel Position", () -> Climber.getInstance().getRachelPosition(), true);
    RobotLogging.getInstance().registerImmediate("Rachel Velocity", () -> Climber.getInstance().getRachelVelocity(), true);
    RobotLogging.getInstance().registerImmediate("Rachel Left", () -> Climber.getInstance().getLeftRachelOutput(), true);
    RobotLogging.getInstance().registerImmediate("Rachel Right", () -> Climber.getInstance().getRightRachelOutput(), true);
    RobotLogging.getInstance().registerImmediate("Rachel Left Position", () -> Climber.getInstance().getLeftRachelPosition(), true);
    RobotLogging.getInstance().registerImmediate("Rachel Right Position", () -> Climber.getInstance().getRightRachelPosition(), true);

    // Shooter Logging
    RobotLogging.getInstance().registerEnumHistorian("Shooter Target Speed", () -> Shooter.getInstance().getTargetSpeed());
    RobotLogging.getInstance().registerHistorian("Shooter Target Velocity", () -> Shooter.getInstance().getTargetSpeed().getMotorSpeed());
    RobotLogging.getInstance().registerImmediate("Shooter Velocity", () -> Shooter.getInstance().getShooterSpeed(), true);
    RobotLogging.getInstance().registerImmediate("Shooter Output", () -> Shooter.getInstance().getShooterOutput(), true);
    RobotLogging.getInstance().registerImmediate("Shooter Follower Output", () -> Shooter.getInstance().getFollowerOutput(), true);
    RobotLogging.getInstance().registerImmediate("Shooter P Error", () -> Shooter.getInstance().getPError(), true);
    RobotLogging.getInstance().registerImmediate("Shooter I Error", () -> Shooter.getInstance().getIError(), true);
    RobotLogging.getInstance().registerImmediate("Shooter D Error", () -> Shooter.getInstance().getDError(), true);
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
    shootOne.whenPressed(new ClearBallHandler());
    shootAll.whenPressed(new ShootAll());

    runIntake.whileActiveContinuous(new RunIntake());

    prepareClimbXbox.whenPressed(new SetClimberState(ClimberState.BABYS_FIRST_REACH));
    highBarXbox.whenPressed(getHighClimbCommand());
    traversalXbox.whenPressed(getTraversalClimbCommand());
    traversalFromHighStick.whenPressed(getTransversalFromHighCommand());
    highFromMidXbox.whenPressed(getHighFromMidCommand());
    toggleColorSensor.whenPressed(new InstantCommand(() -> Intake.getInstance().toggleColorSensor()));
    simpleClimbXbox.whenPressed(getSimpleClimbCommand());
    // prepareClimbButton.whenPressed(new
    // SetClimberState(ClimberState.BABYS_FIRST_REACH));
    // simpleClimbButton.whenPressed(getSimpleClimbCommand());

    SmartDashboard.putData("Gabe Height", new InstantCommand(() -> Climber.getInstance().moveRachelPosition(RachelExtensionStates.GABE_HEIGHT)));
    SmartDashboard.putData("Release Reach", new InstantCommand(() -> Climber.getInstance().moveRachelPosition(RachelExtensionStates.RELEASE_REACH)));
    SmartDashboard.putData("Full Extension", new InstantCommand(() -> Climber.getInstance().moveRachelPosition(RachelExtensionStates.FULLY_EXTENDED)));

    SmartDashboard.putData("Drive Shooter", new InstantCommand(() -> Shooter.getInstance().shooterPercentMotor(0.75)));
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
                                       new FarHighGoalShoot(),
                                       new ShootAll(),
                                       new WaitCommand(0.5),
                                       new ParallelDeadlineGroup(new SequentialCommandGroup(new WaitCommand(1.0), new DriveXinches(100.0)), new RunIntake()), 
                                       new DriveXinches(-97),
                                       new FarHighGoalShoot(),
                                       new ShootAll(),
                                       new WaitCommand(0.5)
                                       );
  }

  private Command getHighClimbCommand() {
    return new SequentialCommandGroup(
        new SetClimberState(ClimberState.BABYS_FIRST_REACH),
        new SetClimberState(ClimberState.SUCCESSFULL_PULL_UP),
        new SetClimberState(ClimberState.SUCCESSFULL_HANG),
        new SetClimberState(ClimberState.SUCCESSFULL_HANG),
        new SetClimberState(ClimberState.T_REX_REACH),
        new SetClimberState(ClimberState.BRONTOSAURUS_REACHING),
        new SetClimberState(ClimberState.REACH_PULL),
        new WaitCommand(1),
        new SetClimberState(ClimberState.REACH_CAUGHT),
        new SetClimberState(ClimberState.TRUST_FALL),
        new InstantCommand(() -> Climber.getInstance().stopRachel(), Climber.getInstance()));
  }

  private Command getTransversalFromHighCommand() {
    return new SequentialCommandGroup(
      new SetClimberState(ClimberState.SUCCESSFULL_PULL_UP),
      new SetClimberState(ClimberState.SUCCESSFULL_HANG),
      new SetClimberState(ClimberState.SUCCESSFULL_HANG),
      new SetClimberState(ClimberState.T_REX_REACH),
      new SetClimberState(ClimberState.BRONTOSAURUS_REACHING),
      new SetClimberState(ClimberState.REACH_PULL),
      new WaitCommand(1),
      new SetClimberState(ClimberState.REACH_CAUGHT),
      new SetClimberState(ClimberState.TRUST_FALL),
      new InstantCommand(() -> Climber.getInstance().stopRachel(), Climber.getInstance()));
  }

  private Command getHighFromMidCommand() {
    return new SequentialCommandGroup(
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
        new SetClimberState(ClimberState.SUCCESSFULL_HANG),
        new SetClimberState(ClimberState.T_REX_REACH),
        new SetClimberState(ClimberState.BRONTOSAURUS_REACHING),
        new SetClimberState(ClimberState.REACH_PULL),
        new WaitCommand(1),
        new SetClimberState(ClimberState.REACH_CAUGHT),
        new SetClimberState(ClimberState.TRUST_FALL),
        new WaitCommand(.75),
        new SetClimberState(ClimberState.SUCCESSFULL_PULL_UP),
        new SetClimberState(ClimberState.SUCCESSFULL_HANG),
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
        new SetClimberState(ClimberState.SUCCESSFULL_HANG),
        new InstantCommand(() -> Climber.getInstance().stopRachel(), Climber.getInstance()));
  }
}

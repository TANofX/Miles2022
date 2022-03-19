// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;
import java.util.concurrent.RunnableScheduledFuture;
import java.util.spi.CurrencyNameProvider;

import javax.net.ssl.TrustManagerFactorySpi;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
      /** Creates a new Climber. */
      // Rachel Falcons to extend and retract climber bars
      private WPI_TalonFX leftRachelFalcon;
      private WPI_TalonFX rightRachelFalcon;

      // passed through motors so we can read them through there
      // Up and Down limits for the extending bars. These will only be used to
      // calibrate and as a fail safe if code tells them to go past their physical
      // limits\
      private TalonFXSensorCollection rachelLeftCollection;
      private TalonFXSensorCollection rachelRightCollection;

      // Pistons for Gabe's active claw and Rachel's Reach function
      private Solenoid gabeClaw;
      private DoubleSolenoid rachelReach;

      // Limit switches which will allow us to know if we are clawed on a bar for gabe
      // or rachel
      private DigitalInput gabeLeftIdentification;
      private DigitalInput gabeRightIndentification;
      private DigitalInput rachelLeftBarSensor;
      private DigitalInput rachelRightBarSensor;

      private boolean leftRachelLimitLatch = false;
      private boolean rightRachelLimitLatch = false;

      public static enum ClimberState {
            UNKNOWN,
            STARTING_CONFIG,
            REACHING,
            BABYS_FIRST_REACH,
            BABYS_FIRST_SEARCH,
            BABYS_FIRST_PULL_UP,
            SUCCESSFULL_PULL_UP,
            SUCCESSFULL_HANG,
            RELEASE_REACH,
            T_REX_REACH,
            STEGOSAURUS_REACHING,
            BRONTOSAURUS_REACHING,
            REACH_PULL,
            REACH_SEARCHING,
            REACH_CATCH,
            REACH_CAUGHT,
            TRUST_FALL
      }

      private static enum GabeStates {
            UNKNOWN,
            OPEN,
            CLOSED_WITH_BAR,
            CLOSED_WITHOUT_BAR
      }

      // None of the "expectecPosition" values are correct. They will need to be
      // determined experimentally
      public static enum RachelExtensionStates {
       //     FULLY_RETRACTED(0),
            GABE_HEIGHT(-20000),
            FULLY_EXTENDED(198000),
            MID_BAR_HEIGHT(165000),
            TRUST_FALL_LOCATION(90000),
            RELEASE_REACH(45000),
            UNKNOWN(-100000),
            EXTENDING(-100001),
            RETRACTING(-100002);

            private double expectedPosition;
            private static double allowedPositionError = Constants.CLIMB_HEIGHT_THRESHOLD;

            private RachelExtensionStates(double encoderPosition) {
                  expectedPosition = encoderPosition;
            }

            private double getMotorTarget() {
                  return expectedPosition;
            }

            private static RachelExtensionStates findState(double position) {
                  for (RachelExtensionStates s : RachelExtensionStates.values()) {
                        if (Math.abs(s.getMotorTarget() - position) < allowedPositionError) {
                              return s;
                        }
                  }

                  return UNKNOWN;
            }
      }

      private static enum RachelBarStates {
            UNKNOWN,
            REACHING_NO_BAR,
            REACHING_BAR_CONTACT,
            NOT_REACHING_NO_BAR,
            NOT_REACHING_BAR_CONTACT
      }

      private ClimberState currentState = ClimberState.UNKNOWN;
      private PneumaticsControlModule pcm;

      private static Climber climberInstance = null;

      public static Climber getInstance() {
            if (climberInstance == null) {
                  climberInstance = new Climber();
            }
            return climberInstance;
      }

      private Climber() {
            pcm = new PneumaticsControlModule(2);

            leftRachelFalcon = new WPI_TalonFX(Constants.LEFT_RACHEL_FALCON);
            rightRachelFalcon = new WPI_TalonFX(Constants.RIGHT_RACHEL_FALCON);
            // talon.getSensorCollection().isRevLimitSwitchClosed() == 1=closed and 0=open

            rachelRightCollection = rightRachelFalcon.getSensorCollection();
            rachelLeftCollection = leftRachelFalcon.getSensorCollection();

            gabeClaw = pcm.makeSolenoid(Constants.GABE_CLAW);
            rachelReach = pcm.makeDoubleSolenoid(Constants.RACHEL_FORWARD_REACH, Constants.RACHEL_BACKWARD_REACH);


            gabeLeftIdentification = new DigitalInput(Constants.GABE_LEFT_IDENTIFICATION);
            gabeRightIndentification = new DigitalInput(Constants.GABE_RIGHT_IDENTIFICATION);
            rachelLeftBarSensor = new DigitalInput(Constants.RACHEL_LEFT_BAR_SENSOR);
            rachelRightBarSensor = new DigitalInput(Constants.RACHEL_RIGHT_BAR_SENSOR);

            configureRachelMotors(leftRachelFalcon);
            configureRachelMotors(rightRachelFalcon);
      }

      private void configureRachelMotors(WPI_TalonFX falcon) {
            falcon.setNeutralMode(NeutralMode.Brake);
            falcon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, Constants.CLIMBER_CURRENT_LIMIT, Constants.CLIMBER_THRESHOLD_CURRENT, Constants.CLIMBER_THRESHOLD_TIMEOUT));
            falcon.configClosedloopRamp(0.1);
            falcon.selectProfileSlot(0,0);
        
            falcon.config_kF(0, 0.031, 0);
            falcon.config_kP(0, 0.05, 0);
            falcon.config_kI(0, .0005, 0);
            falcon.config_IntegralZone(0, 1000);

            falcon.config_kF(1, 0.034, 0);
            falcon.config_kP(1, 0.06, 0);
            falcon.config_kI(1, Constants.RACHEL_I, 0);
            falcon.config_IntegralZone(1, 0);

            falcon.config_kF(2, 0.028, 0);
            falcon.config_kP(2, 0.06, 0);
            falcon.config_kI(2, Constants.RACHEL_I, 0);
            falcon.config_IntegralZone(2, 0);

            falcon.config_kF(3, 0.043, 0);
            falcon.config_kP(3, 0.02, 0);
            falcon.config_kI(3, 0.0002, 0);
            falcon.config_IntegralZone(3, 10000);

            // NOTE:  These may mess everything up, change or set both to 1.0 if there is a real issue
            falcon.configPeakOutputForward(1.0);
            falcon.configPeakOutputReverse(-1.0);

            falcon.configMotionCruiseVelocity(Constants.CLIMBER_MAX_VELOCITY);
            falcon.configMotionAcceleration(Constants.CLIMBER_MAX_ACCELERATION);

            falcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,0);
            
            falcon.selectProfileSlot(3, 0);
            falcon.configForwardSoftLimitThreshold(RachelExtensionStates.FULLY_EXTENDED.getMotorTarget() + 10);
            falcon.configReverseSoftLimitThreshold(0);
        //    falcon.configForwardSoftLimitEnable(true);
        //    falcon.configReverseSoftLimitEnable(true);
        
            falcon.setSelectedSensorPosition(0);
      }

      private RachelBarStates getRachelBarState() {
            if (rachelLeftBarSensor.get() && rachelRightBarSensor.get()) {
                  if (rachelReach.get() == DoubleSolenoid.Value.kReverse) {
                        return RachelBarStates.REACHING_BAR_CONTACT;
                  } else if (rachelReach.get() == DoubleSolenoid.Value.kForward) {
                        return RachelBarStates.NOT_REACHING_BAR_CONTACT;
                  }
            } else if (!rachelLeftBarSensor.get() && !rachelRightBarSensor.get()) {
                  if (rachelReach.get() == DoubleSolenoid.Value.kReverse) {
                        return RachelBarStates.REACHING_NO_BAR;
                  } else if (rachelReach.get() == DoubleSolenoid.Value.kForward) {
                        return RachelBarStates.NOT_REACHING_NO_BAR;
                  }

            }

            return RachelBarStates.UNKNOWN;
      }

      private void setRachelPIDSlot() {
            if (rachelLeftBarSensor.get() && rachelRightBarSensor.get()) {
                  if (getGabeState() == GabeStates.CLOSED_WITH_BAR) {
                        leftRachelFalcon.selectProfileSlot(2, 0);
                        rightRachelFalcon.selectProfileSlot(2, 0);
                  } else {
                        leftRachelFalcon.selectProfileSlot(3, 0);
                        rightRachelFalcon.selectProfileSlot(3, 0);
                  }
            } else if (!rachelLeftBarSensor.get() && !rachelRightBarSensor.get()) {
                  leftRachelFalcon.selectProfileSlot(0, 0);
                  rightRachelFalcon.selectProfileSlot(0, 0);
            }
      }

      private GabeStates getGabeState() {
            if (!gabeClaw.get()) {
                  if (!gabeLeftIdentification.get() && !gabeRightIndentification.get()) {
                        return GabeStates.CLOSED_WITH_BAR;
                  } else if (gabeLeftIdentification.get() && gabeRightIndentification.get()) {
                        return GabeStates.CLOSED_WITHOUT_BAR;
                  }
            } else if (gabeClaw.get()) {
                  return GabeStates.OPEN;
            }

            return GabeStates.UNKNOWN;
      }

      public RachelExtensionStates getExtensionState() {
            double currentVelocity = leftRachelFalcon.getSelectedSensorVelocity();
            double currentPosition = leftRachelFalcon.getSelectedSensorPosition();

            double extensionState = Math.signum(currentVelocity);

            if (isRachelBottomLimit()) {
                  return RachelExtensionStates.GABE_HEIGHT;
            }
            // Check to see if the arms are moving and return just a movement direction
            if (Math.abs(currentVelocity) > 10.0) {
                  if (extensionState > 0) {
                        return RachelExtensionStates.EXTENDING;
                  } else if (extensionState < 0) {
                        return RachelExtensionStates.RETRACTING;
                  }
            }

           
            // Assume we are at a target position, because our velocity is small
            return RachelExtensionStates.findState(currentPosition);
      }

      public double getLeftRachelOutput() {
            return leftRachelFalcon.getMotorOutputPercent();
      }

      public double getRightRachelOutput() {
            return rightRachelFalcon.getMotorOutputPercent();
      }

      public double getLeftRachelPosition() {
            return leftRachelFalcon.getSelectedSensorPosition();
      }

      public double getRightRachelPosition() {
            return rightRachelFalcon.getSelectedSensorPosition();
      }

      public void stateMachineTwo() {
            GabeStates currentGabe = getGabeState();
            RachelExtensionStates currentRachelExt = getExtensionState();
            RachelBarStates currentRachelBar = getRachelBarState();

            switch (currentGabe) {
                  case OPEN:
                        switch (currentRachelExt) {
                              case EXTENDING:
                                    switch (currentRachelBar) {
                                          case NOT_REACHING_NO_BAR:
                                                currentState = ClimberState.REACHING;
                                                break;
                                          default:
                                                currentState = ClimberState.UNKNOWN;
                                                break;
                                    }
                                    break;
                              case RETRACTING:
                                    switch (currentRachelBar) {
                                          case NOT_REACHING_BAR_CONTACT:
                                                currentState = ClimberState.BABYS_FIRST_PULL_UP;
                                                break;
                                          case NOT_REACHING_NO_BAR:
                                                currentState = ClimberState.BABYS_FIRST_SEARCH;
                                                break;
                                          default:
                                                currentState = ClimberState.UNKNOWN;
                                                break;
                                    }
                                    break;
                              case GABE_HEIGHT:
                                    switch (currentRachelBar) {
                                          case NOT_REACHING_BAR_CONTACT:
                                                currentState = ClimberState.SUCCESSFULL_PULL_UP;
                                                break;
                                          default:
                                                currentState = ClimberState.UNKNOWN;
                                                break;
                                    }
                                    break;
                              case FULLY_EXTENDED:
                                    switch (currentRachelBar) {
                                          default:
                                                currentState = ClimberState.UNKNOWN;
                                                break;
                                    }
                                    break;
                              case MID_BAR_HEIGHT:
                                    switch (currentRachelBar) {
                                          case NOT_REACHING_NO_BAR:
                                                currentState = ClimberState.BABYS_FIRST_REACH;
                                                break;
                                          default:
                                                currentState = ClimberState.UNKNOWN;
                                                break;
                                    }
                                    break;
                              case TRUST_FALL_LOCATION:
                                    switch (currentRachelBar) {
                                          case NOT_REACHING_BAR_CONTACT:
                                                currentState = ClimberState.TRUST_FALL;
                                                break;
                                          default:
                                                currentState = ClimberState.UNKNOWN;
                                                break;
                                    }
                                    break;
                              default:
                                    currentState = ClimberState.UNKNOWN;
                                    break;
                        }
                        break;
                  case CLOSED_WITH_BAR:
                        switch (currentRachelExt) {
                              case EXTENDING:
                                    switch (currentRachelBar) {
                                          case NOT_REACHING_NO_BAR:
                                                currentState = ClimberState.RELEASE_REACH;
                                                break;
                                          case REACHING_NO_BAR:
                                                currentState = ClimberState.STEGOSAURUS_REACHING;
                                                break;
                                          default:
                                                currentState = ClimberState.UNKNOWN;
                                                break;
                                    }
                                    break;
                              case RETRACTING:
                                    switch (currentRachelBar) {
                                          case NOT_REACHING_NO_BAR:
                                                currentState = ClimberState.REACH_SEARCHING;
                                                break;
                                          case NOT_REACHING_BAR_CONTACT:
                                                currentState = ClimberState.REACH_CATCH;
                                                break;
                                          default:
                                                currentState = ClimberState.UNKNOWN;
                                                break;
                                    }
                                    break;
                              case GABE_HEIGHT:
                                    switch (currentRachelBar) {
                                          case NOT_REACHING_BAR_CONTACT:
                                                currentState = ClimberState.SUCCESSFULL_HANG;
                                                break;
                                          default:
                                                currentState = ClimberState.UNKNOWN;
                                                break;
                                    }
                                    break;
                              case RELEASE_REACH:
                                    switch (currentRachelBar) {
                                          case REACHING_NO_BAR:
                                                currentState = ClimberState.T_REX_REACH;
                                                break;
                                          default:
                                                currentState = ClimberState.UNKNOWN;
                                                break;
                                    }
                                    break;
                              case FULLY_EXTENDED:
                                    switch (currentRachelBar) {
                                          case REACHING_NO_BAR:
                                                currentState = ClimberState.BRONTOSAURUS_REACHING;
                                                break;
                                          case NOT_REACHING_NO_BAR:
                                                currentState = ClimberState.REACH_PULL;
                                                break;
                                          default:
                                                currentState = ClimberState.UNKNOWN;
                                                break;
                                    }
                                    break;
                              case TRUST_FALL_LOCATION:
                                    switch (currentRachelBar) {
                                          case NOT_REACHING_BAR_CONTACT:
                                                currentState = ClimberState.REACH_CAUGHT;
                                                break;
                                          default:
                                                currentState = ClimberState.UNKNOWN;
                                                break;
                                    }
                                    break;
                              default:
                                    currentState = ClimberState.UNKNOWN;
                                    break;
                        }
                        break;
                  case CLOSED_WITHOUT_BAR:
                        switch (currentRachelBar) {
                              case NOT_REACHING_NO_BAR:
                                    switch (currentRachelExt) {
                                          case GABE_HEIGHT:
                                                currentState = ClimberState.STARTING_CONFIG;
                                                break;
                                          default:
                                                currentState = ClimberState.UNKNOWN;
                                                break;
                                    }

                                    break;
                              default:
                                    currentState = ClimberState.UNKNOWN;
                                    break;
                        }

                        break;
                  default:
                        currentState = ClimberState.UNKNOWN;
                        break;
            }

      }

      public void toggleGabe() {
            boolean gabeClawL = gabeClaw.get();
            if (gabeClawL) {
                  switch (currentState) {
                        case STARTING_CONFIG:
                        case REACHING:
                        case BABYS_FIRST_REACH:
                        case BABYS_FIRST_SEARCH:
                        case BABYS_FIRST_PULL_UP:
                        case SUCCESSFULL_PULL_UP:
                              break;
                        case SUCCESSFULL_HANG:
                        case RELEASE_REACH:
                        case T_REX_REACH:
                        case STEGOSAURUS_REACHING:
                        case BRONTOSAURUS_REACHING:
                        case REACH_PULL:
                        case REACH_SEARCHING:
                              gabeClawL = !gabeClawL;
                              break;
                        case REACH_CATCH:
                        case REACH_CAUGHT:
                        case TRUST_FALL:
                              break;
                        default:
                  }
            }
            gabeClaw.set(!gabeClawL);

      }

      public void gabeOpen() {
            gabeClaw.set(true);
     
      }

      public void gabeClosed() {
            gabeClaw.set(false);
    
      }

      public void toggleRachel() {
            DoubleSolenoid.Value rachelReachL = rachelReach.get();
            if (rachelReachL == DoubleSolenoid.Value.kForward) {
                  rachelReachL = DoubleSolenoid.Value.kReverse;
            }
            else {
                  rachelReachL = DoubleSolenoid.Value.kForward;
            }
            rachelReach.set(rachelReachL);
            
          
      }

      public void rachelNoReach() {
            rachelReach.set(DoubleSolenoid.Value.kForward);
        
      }

      public void rachelReach() {
            rachelReach.set(DoubleSolenoid.Value.kReverse);
         
      }

      public ClimberState getCurrentState() {
            return currentState;
      }

      public double getRachelVelocity() {
            return (rightRachelFalcon.getSelectedSensorVelocity() + leftRachelFalcon.getSelectedSensorVelocity()) / 2.0;
      }

      public double getRachelPosition() {
            return (rightRachelFalcon.getSelectedSensorPosition() + leftRachelFalcon.getSelectedSensorPosition()) / 2.0;
      }

      public void moveRachelPosition(double position) {
            double rachelPosition = position;
            leftRachelFalcon.set(ControlMode.MotionMagic, rachelPosition);
            rightRachelFalcon.set(ControlMode.MotionMagic, rachelPosition);
            SmartDashboard.putNumber("Move Rachel To Position", rachelPosition);
      }

      public void moveRachelPosition(RachelExtensionStates targetState) {
            moveRachelPosition(targetState.getMotorTarget());
      }

      public void calibrateRachel() {
            leftRachelFalcon.setSelectedSensorPosition(0);
            rightRachelFalcon.setSelectedSensorPosition(0);
      }

      private void enableRachelSoftLimit(boolean enable) {
            rightRachelFalcon.configReverseSoftLimitEnable(enable);
            leftRachelFalcon.configReverseSoftLimitEnable(enable);
      }

      public boolean calibrateClimber() {
            enableRachelSoftLimit(false);
            if (!isRachelBottomLimit()) {
                  moveRachelDown();
            }
            else {calibrateRachel();
            gabeClosed();
            rachelNoReach();
            enableRachelSoftLimit(false);
            return true;
            }
            return false;
      }

      public void moveRachelDown() {
            leftRachelFalcon.set(ControlMode.PercentOutput, -0.3);
            rightRachelFalcon.set(ControlMode.PercentOutput, -0.3);
      }

      public void fullSpeedUp() {
            leftRachelFalcon.set(ControlMode.PercentOutput, .5);
            rightRachelFalcon.set(ControlMode.PercentOutput, .5);
      }

      public void fullSpeedDown() {
            leftRachelFalcon.set(ControlMode.PercentOutput, -.5);
            rightRachelFalcon.set(ControlMode.PercentOutput, -.5);
      }

      public void fullSpeedStop() {
            leftRachelFalcon.stopMotor();
            rightRachelFalcon.stopMotor();
      }

      public boolean isRachelBottomLimit() {
            if (rachelLeftCollection.isRevLimitSwitchClosed() == 1){
                  leftRachelLimitLatch = true;
            }
            if (rachelRightCollection.isRevLimitSwitchClosed() == 1) {
                  rightRachelLimitLatch =  true;
            }
            return leftRachelLimitLatch && rightRachelLimitLatch;
      }

      public void stopRachel() {
            leftRachelFalcon.set(ControlMode.PercentOutput, 0);
            rightRachelFalcon.set(ControlMode.PercentOutput, 0);
      }

      public boolean goToTargetState(ClimberState newState) {
            switch (currentState) {
                  case SUCCESSFULL_HANG:
                  case RELEASE_REACH:
                  case REACH_PULL:
                  case REACH_SEARCHING:
                  case REACH_CATCH:
                        switch (newState) {
                              case REACHING:
                              case BABYS_FIRST_REACH:
                              case BABYS_FIRST_PULL_UP:
                              case SUCCESSFULL_PULL_UP:
                              case TRUST_FALL:
                                    return false;
                              default:
                        }
                        break;
                  case REACH_CAUGHT:
                        switch (newState) {
                              case REACHING:
                              case BABYS_FIRST_REACH:
                              case BABYS_FIRST_PULL_UP:
                              case SUCCESSFULL_PULL_UP:
                                    return false;
                              default:
                        }
                        break;
                  case T_REX_REACH:
                  case STEGOSAURUS_REACHING:
                  case BRONTOSAURUS_REACHING:
                        switch (newState) {
                              case STARTING_CONFIG:
                              case REACHING:
                              case BABYS_FIRST_REACH:
                              case BABYS_FIRST_PULL_UP:
                              case SUCCESSFULL_HANG:
                              case SUCCESSFULL_PULL_UP:
                              case RELEASE_REACH:
                              case REACH_SEARCHING:
                              case REACH_CATCH:
                              case REACH_CAUGHT:
                              case TRUST_FALL:
                                    return false;
                              default:
                        }
                        break;
                  default:
            }
            switch (newState) {
                  case STARTING_CONFIG:
                        moveRachelPosition(RachelExtensionStates.GABE_HEIGHT);
                        gabeClosed();
                        rachelNoReach();
                        break;
                  case REACHING:
                        throw new IllegalStateException();
                  case BABYS_FIRST_REACH:
                        moveRachelPosition(RachelExtensionStates.MID_BAR_HEIGHT);
                        gabeOpen();
                        rachelNoReach();
                        break;
                  case BABYS_FIRST_SEARCH:
                  case BABYS_FIRST_PULL_UP:
                        throw new IllegalStateException();
                  case SUCCESSFULL_PULL_UP:
                        moveRachelPosition(RachelExtensionStates.GABE_HEIGHT);
                        gabeOpen();
                        rachelNoReach();
                        break;
                  case SUCCESSFULL_HANG:
                        moveRachelPosition(RachelExtensionStates.GABE_HEIGHT);
                        gabeClosed();
                        rachelNoReach();
                        break;
                  case RELEASE_REACH:
                        throw new IllegalStateException();
                  case T_REX_REACH:
                        moveRachelPosition(RachelExtensionStates.RELEASE_REACH);
                        gabeClosed();
                        rachelReach();
                        break;
                  case STEGOSAURUS_REACHING:
                        throw new IllegalStateException();
                  case BRONTOSAURUS_REACHING:
                        moveRachelPosition(RachelExtensionStates.FULLY_EXTENDED);
                        gabeClosed();
                        rachelReach();
                        break;
                  case REACH_PULL:
                        moveRachelPosition(RachelExtensionStates.FULLY_EXTENDED);
                        gabeClosed();
                        rachelNoReach();
                        break;
                  case REACH_SEARCHING:
                  case REACH_CATCH:
                        throw new IllegalStateException();
                  case REACH_CAUGHT:
                        moveRachelPosition(RachelExtensionStates.TRUST_FALL_LOCATION);
                        gabeClosed();
                        rachelNoReach();
                        break;
                  case TRUST_FALL:
                        moveRachelPosition(RachelExtensionStates.TRUST_FALL_LOCATION);
                        gabeOpen();
                        rachelNoReach();
                        break;
                  default:
            }
            return true;
      }

      @Override
      public void periodic() {
            if (getRachelPosition() > 5000) {
                  resetRachelLatch();
            }
            stateMachineTwo();
            setRachelPIDSlot();

            //SmartDashboard.putString("Current State", getCurrentState().name());
            SmartDashboard.putString("Rachel Bar State", getRachelBarState().name());
            SmartDashboard.putString("Gabe Bar State", getGabeState().name());
            //SmartDashboard.putString("Rachel Extention State", getExtensionState().name());
            // This method will be called once per scheduler run
            //SmartDashboard.putNumber("Current Velocity", getRachelVelocity());
            //SmartDashboard.putNumber("Current Rachel Position", getRachelPosition());

      }

      public void resetRachelLatch() {
            leftRachelLimitLatch = false;
            rightRachelLimitLatch = false;
      }

      public double getMaxVelicity() {
            return Constants.CLIMBER_MAX_VELOCITY;
      }
}

/*
 * Climber State Machine
 * ( Drives back up past bar)
 * Drives move forward till Climber forearm sensor trigger (Robot is in potion9)
 * ("love potion number 9!")
 * 
 * -Starting_Config- Both arms upright, retracted, active bar sensor not
 * triggered (starting config)
 * Gabe - Pneumatics Closed, Bar identification = No
 * Rachel - Falcons Not moving, Pneumatics closed, Low climbing Limit Switch =
 * yes, Active Bar sensor = no
 * 
 * -Reaching- Both arms upright, extending, active bar sensor not triggered,
 * grabby open
 * Gabe - Pneumatics Open, Bar identification = No
 * Rachel - Falcons Moving(extending), Pneumatics closed, No limit switches
 * triggered
 * 
 * -Babys_First_Reach- Both arms upright, extended, grabby open (initial
 * climbing position)
 * Gabe - Pneumatics Open, Bar identification = No
 * Rachel - Falcons Not moving, Pneumatics closed, Fully extended, Active Bar
 * sensor = no
 * 
 * -Babys_First_Pull_Up- Both arms upright, retracting, active bar sensor
 * triggered
 * Gabe - Pneumatics Open, Bar identification = No
 * Rachel - Falcons Moving(retracting to a position), Pneumatics closed, Active
 * Bar Sensor = Yes
 * 
 * -Sucessful_Pull_Up- Both arms upright, retracted, Bar identification
 * triggered
 * Gabe - Pneumatics Open, Bar identification = no
 * Rachel - Falcons Not moving, Pneumatics closed, Gabe height, Active Bar
 * Sensor = Yes
 * 
 * -Sucessful_Hang- Both arms upright, retracted, grabby grabbed bar
 * Gabe - Pneumatics Closed, Bar identification = yes
 * Rachel - Falcons Not moving, Pneumatics closed, Gabe Height, Active Bar
 * Sensor = Yes
 * 
 * -Release_Reach- Both arms upright, small extension, grabby grabbed bar
 * (unhooking from previous bar)
 * Gabe - Pneumatics Closed, Bar identification = yes
 * Rachel - Falcons Moving(extending), Pneumatics closed, Active Bar Sensor = No
 * 
 * -T_Rex_Reach- Gabe grabbed, small extension, active arm reaching
 * Gabe - Pneumatics Closed, Bar identification = yes
 * Rachel - Falcons Not Moving, Pneumatics open, Active Bar Sensor = No
 * 
 * -Stegosaurus_Reaching- Gabe grabbed, extending, active arm reaching
 * Gabe - Pneumatics Closed, Bar identification = yes
 * Rachel - Falcons Moving(Extended), Pneumatics open, Active Bar Sensor = No
 * 
 * -Brontosaurus_Reach- Gabe grabbed, active arm reaching and extended
 * Gabe - Pneumatics Closed, Bar identification = yes
 * Rachel - Falcons Not moving, Pneumatics open, Fully extended, Active Bar
 * Sensor = No
 * 
 * -Reach_Pull- Gabe grabbed, active arm upright and extended (active arm has
 * been pulled into the bar in order to get ready to grab the next bar)
 * Gabe - Pneumatics Closed, Bar identification = yes
 * Rachel - Falcons Not moving, Pneumatics closed, fully extended, Active Bar
 * Sensor = No,
 * 
 * -Reach_Searching- Gabe grabbed, active arm upright and retracting, active bar
 * sensor not triggered
 * Gabe - Pneumatics closed, Bar identification = yes
 * Rachel - Falcons Moving(retracting), Pneumatics closed, Active Bar Sensor =
 * No,
 * 
 * -Reach_Catch- Gabe grabbed, active arm upright and retracting, active bar
 * sensor triggered
 * Gabe - Pneumatics closed, Bar Identification = yes
 * Rachel - Falcons Moving(retracting) to a position, Pneumatics closed, Active
 * Bar Sensor = yes
 * 
 * -Reach_Caught- Gabe grabbed, active arm upright and retracting to a position,
 * active bar sensor triggered
 * Gabe - Pneumatics closed, Bar Identification = yes
 * Rachel - Falcons not moving at position, Pneumatics closed, Active Bar Sensor
 * = yes
 * 
 * -Trust_Fall- Gabe released, active arm upright and retracting, active bar
 * sensor triggered
 * Gabe - Pneumatics open, Bar Identification = No
 * Rachel - Falcons not moving, Pneumatics closed, Active bar sensor = yes
 * 
 * 
 * 
 * 
 * 
 * All Motors and Sensors
 * Active Arm - reachy arm - reachel (rachel)
 * Active Bar sensor = Rachel sensor for holding the climbing bar
 * 2 Falcons to drive each arm
 * 2 Pneumatic Pistons to flop
 * # of Limit switches for extended/retracted states of climber in a box
 * Limit switch for forearm limit switch (knowing when we are in contact with
 * the bar both when on the ground climbing up and reaching to the next stage)
 * 
 * 
 * Gabe - grabby arm - gabe
 * 2 Pneumatic Pistons to grab bar
 * 2 Limit switches for bar identification
 * 
 * 
 */
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import frc.robot.Constants.HopperSubsystemConstants;
import frc.robot.Constants.HopperSubsystemConstants.HopperSubSystemSetpoints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum HopperSetpoints {
    kBaseUpright,
    ktiltSetpoint,
    kDownSetpoint
  }

  // Initialize Hopper SPARK. We will use MAXMotion position control for the Hopper, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkFlex hopperMotor =
      new SparkFlex(HopperSubsystemConstants.kHopperMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController hopperController = hopperMotor.getClosedLoopController();
  private RelativeEncoder hopperEncoder = hopperMotor.getEncoder();

  // starting Setpoint for the Hopper. This will be updated when setSetpointCommand is called.
  private double HopperCurrentTarget = HopperSubSystemSetpoints.kBaseUpright;

  /** Creates a new HopperSubsystem. */
  public HopperSubsystem() {
    hopperMotor.configure(
        Configs.HopperSubsystem.hopperConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    this.setHopperPower(HopperSubsystemConstants.HopperSetpointTestSpeed);

    // Zero Hopper and elevator encoders on initialization
    hopperEncoder.setPosition(0);
  }

  /**
   * Drive the Hopper motor to its setpoint. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanism's
   * setpoint.
   */
  public void moveToSetpoint() {
    hopperController.setReference(HopperCurrentTarget, ControlType.kMAXMotionPositionControl);
  }


  /**
   * Command to set the subsystem setpoint. This will set the Hopper and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(HopperSetpoints setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case ktiltSetpoint:
              HopperCurrentTarget = HopperSubSystemSetpoints.ktiltedSetpoint;
              break;
            case kDownSetpoint:
              HopperCurrentTarget = HopperSubSystemSetpoints.kDownSetpoint;
              break;
            case kBaseUpright:
              HopperCurrentTarget = HopperSubSystemSetpoints.kBaseUpright;
              break;

          }
        });
  }

  @Override
  public void periodic() {
    // Move the hopper to its setpoint when called once per scheduler run
    moveToSetpoint();
    //hopperMotor.set(HopperSubsystemConstants.HopperSetpointTestSpeed);

    // Display subsystem values
    SmartDashboard.putNumber("Hopper/Target Position", HopperCurrentTarget);
    SmartDashboard.putNumber("Hopper/Actual Position", hopperEncoder.getPosition());
    SmartDashboard.putNumber("Hopper/Power", HopperSubsystemConstants.HopperSetpointTestSpeed);
  }
  
  /** Set Hopper motor power in the range of [-1, 1]. - TEST Purpose: step through */
  private void setHopperPower(double power) {
    hopperMotor.set(power);
  }

  /**
   * Command to run the hopper motor. 
   * Intended to step through to adjust proper setpoints
   * When the command is interrupted, e.g. the button is released, the motor will stop.
   */
  public Command runHopperUpCommand() {
    return this.startEnd(
        () -> this.setHopperPower(HopperSubsystemConstants.HopperSetpointTestSpeed), 
        () -> this.setHopperPower(0.0));
  }

  public Command runHopperDownCommand() {
    return this.startEnd(
        () -> this.setHopperPower((-1) * HopperSubsystemConstants.HopperSetpointTestSpeed), 
        () -> this.setHopperPower(0.0));
  }
}

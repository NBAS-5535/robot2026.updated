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
import frc.robot.Constants.IntakeSubsystemConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  // Initialize Intake SPARK. We will use MAXMotion position control for the Intake, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkFlex hopperMotor =
      new SparkFlex(IntakeSubsystemConstants.kIntakeMotorCanId, MotorType.kBrushless);

  private double m_motorPower = 0.0;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(double motorPower) {
    this.m_motorPower = motorPower;
    hopperMotor.configure(
        Configs.IntakeSubsystem.intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  }


  @Override
  public void periodic() {
    hopperMotor.set(m_motorPower);

    // Display subsystem values
    SmartDashboard.putNumber("Intake/Current Power Setting", m_motorPower);

  }
  
  /** Set Intake motor power in the range of [-1, 1]. - TEST Purpose: step through */
  public void resetIntakePower(double power) {
    m_motorPower = power;
  }

  /**
   * Command to run the hopper motor. 
   * Intended to step through to adjust proper setpoints
   * When the command is interrupted, e.g. the button is released, the motor will stop.
   */
  public Command runIntakeInCommand() {
    return this.startEnd(
        () -> this.resetIntakePower(IntakeSubsystemConstants.IntakeSetpointTestSpeed), 
        () -> this.resetIntakePower(0.0));
  }

  public Command runIntakeOutCommand() {
    return this.startEnd(
        () -> this.resetIntakePower((-1) * IntakeSubsystemConstants.IntakeSetpointTestSpeed), 
        () -> this.resetIntakePower(0.0));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.experimental;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;

public class TwoMotorShooterSubsystem extends SubsystemBase {
  /************************************/
  TalonFX leader = new TalonFX(17, new CANBus("rio")); 
  TalonFX follower = new TalonFX(18, new CANBus("rio"));

  private final DutyCycleOut driveRequest = new DutyCycleOut(0);

  TalonFXConfiguration config = new TalonFXConfiguration();

  // power setting to be issued from RobotContainer
  private double m_power = 0.0;

  // some constants
  private final int maxCurrentLimit = 40;
  private static final double kTestPower = 0.1; // Purpose: step through


  /** Constructor */
  public TwoMotorShooterSubsystem() {
     /* TalonFX configs */
       // Motor Output Settings
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Current Limiting (Crucial for dual-motor shafts)
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = maxCurrentLimit; // Amps

    // Apply configuration
    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);

    // Follower setup (Only needs to be called once, typically in robotInit)
    follower.setControl(new Follower(leader.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setPower(m_power);
  }

  /** Set motor power in the range of [-1, 1]. - TEST Purpose: step through */
  public void setPower(double power) {
    leader.setControl(driveRequest.withOutput(power));
  }
}

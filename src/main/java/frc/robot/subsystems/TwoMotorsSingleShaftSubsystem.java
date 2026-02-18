// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;

public class TwoMotorsSingleShaftSubsystem extends SubsystemBase {
  private final SparkMax leaderMotor = new SparkMax(51, MotorType.kBrushless);
  private final SparkMax followerMotor = new SparkMax(52, MotorType.kBrushless);

  private SparkMaxConfig leaderConfig = new SparkMaxConfig();
  private SparkMaxConfig followerConfig = new SparkMaxConfig();

  /************************************/
  TalonFX leader = new TalonFX(51, new CANBus("rio")); 
  TalonFX follower = new TalonFX(52, new CANBus("rio"));

  TalonFXConfiguration config = new TalonFXConfiguration();

  // power setting to be issued from RobotContainer
  private double m_power = 0.0;

  // some constants
  private final int maxCurrentLimit = 40;
  private static final double kTestPower = 0.1; // Purpose: step through

    /** Creates a new TwoMotorsSingleShaftSubsystem. */
  public TwoMotorsSingleShaftSubsystem() {
    /* SparkMax configs */
    leaderConfig
        .smartCurrentLimit(maxCurrentLimit)
        .idleMode(IdleMode.kBrake)
        .inverted(true);

    followerConfig
        .smartCurrentLimit(maxCurrentLimit)
        .idleMode(IdleMode.kBrake)
        .follow(leaderMotor);

    leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    /************************************/
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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setPower(m_power);
  }

  /** Set Lift motor power in the range of [-1, 1]. - TEST Purpose: step through */
  private void setPower(double power) {
    leaderMotor.set(power);
  }
}

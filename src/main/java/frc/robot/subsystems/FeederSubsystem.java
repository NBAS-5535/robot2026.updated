// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallHandlingSubsystemConstants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class FeederSubsystem extends SubsystemBase {
  private final SparkFlex leaderMotor = new SparkFlex(BallHandlingSubsystemConstants.kLeaderFeederMotorCanId, MotorType.kBrushless);
  private final SparkFlex followerMotor = new SparkFlex(BallHandlingSubsystemConstants.kFollowerFeederMotorCanId, MotorType.kBrushless);

  private SparkFlexConfig leaderConfig = new SparkFlexConfig();
  private SparkFlexConfig followerConfig = new SparkFlexConfig();


  // power setting to be issued from RobotContainer
  private double m_leaderPower = 0.0;
  private double m_followerPower = 0.0;

  /** Constructor */
  public FeederSubsystem() {
    /* SparkFlex configs */ // Verify which motor is inverted!!!!
    leaderConfig
        .smartCurrentLimit(BallHandlingSubsystemConstants.kMaxCurrentLimit)
        .idleMode(IdleMode.kBrake);
        //.inverted(true);

    followerConfig
        .smartCurrentLimit(BallHandlingSubsystemConstants.kMaxCurrentLimit)
        .idleMode(IdleMode.kBrake)
        //.inverted(true);
        .follow(leaderMotor);

    leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leaderMotor.set(m_leaderPower);
    followerMotor.set(m_followerPower);
  }

  public void setPower(String label, double power) {
    if ( label == "lead") {
      m_leaderPower = power;
    }else if ( label == "follow") {
      m_followerPower = power;
    }else if ( label == "both") {
      m_leaderPower = power;
      m_followerPower = power;
    }
  }
}

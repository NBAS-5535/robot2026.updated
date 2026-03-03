// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystemConstants;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;



public class ShooterSubsystem extends SubsystemBase {
  //blic static final AngularVelocity kFreeSpeed = RPM.of(6000);
  public static final int rightShooterMotorCanId = 17;
  public static final int leftShooterMotorCanId = 18;

    public enum Speed {
        STOP(0),
        SLOW(0.5),
        FAST(0.8);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    private final TalonFX rightShooterMotor;
    private final TalonFX leftShooterMotor;
    private final VoltageOut shooterVoltageRequest = new VoltageOut(0);

    public ShooterSubsystem() {
        rightShooterMotor = new TalonFX(rightShooterMotorCanId, new CANBus("rio"));
        leftShooterMotor = new TalonFX(leftShooterMotorCanId, new CANBus("rio"));
        configureRightShooterMotorMotor();
        configureLeftShooterMotorMotor();
        SmartDashboard.putData(this);
    }

    private void configureRightShooterMotorMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive) // <-- verify this is correct after testing
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            );
        rightShooterMotor.getConfigurator().apply(config);
    }

    private void configureLeftShooterMotorMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive) // <-- verify this is correct after testing
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            );
        rightShooterMotor.getConfigurator().apply(config);
    }
    
    public void set(Speed speed) {
        rightShooterMotor.setControl(
            shooterVoltageRequest
                .withOutput(speed.voltage())
        );
        leftShooterMotor.setControl(
            shooterVoltageRequest
                .withOutput(speed.voltage())
        );
    }


    public void fastMode(){
        set(Speed.FAST);
    }

    public void slowMode(){
        set(Speed.SLOW);
    }

    public void stopShooter(){
        set(Speed.STOP);
    }

    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("RPM", () -> rightShooterMotor.getVelocity().getValue().in(RPM), null);
                builder.addDoubleProperty("Shooter Supply Current", 
                () -> rightShooterMotor.getSupplyCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("RPM", () -> leftShooterMotor.getVelocity().getValue().in(RPM), null);
                builder.addDoubleProperty("Shooter Supply Current", 
                () -> leftShooterMotor.getSupplyCurrent().getValue().in(Amps), null);
    }
}

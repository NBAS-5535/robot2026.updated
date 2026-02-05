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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.TurretSubsystemConstants;
import frc.robot.Constants.TurretSubsystemConstants.TurretSubSystemSetpoints;

public class TurretSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum TurretSetpoints {
    kBase,
    kmaxRightSetpoint,
    kmaxLeftSetpoint,
  }
  
  /** Creates a new TurretSubsystem. */
  private final PIDController pid = new PIDController(0.025, 0, 0);
  private SparkFlex turretMotor =
      new SparkFlex(44, MotorType.kBrushless);
  private SparkClosedLoopController turretController = turretMotor.getClosedLoopController();
  public RelativeEncoder turretEncoder = turretMotor.getEncoder();

  private double TurretCurrentTarget = TurretSubSystemSetpoints.kBase;
  
  public TurretSubsystem() {

    // configure
    turretMotor.configure(
      Configs.TurretSubsystemConfiguration.turretConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    // Zero turret and elevator encoders on initialization
    turretEncoder.setPosition(0);
    pid.enableContinuousInput(-180, 180);
    turretMotor.setInverted(true);

  }

  public void moveToSetpoint() {
    turretController.setReference(TurretCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  public void setTurretPower(double power) {
    turretMotor.set(power);
  }

  

  public double rotate(double targetDegrees, double robotHeading) {

      double turretFieldAngle = getFieldRelativeTurretAngle(robotHeading);

      double output = pid.calculate(turretFieldAngle, targetDegrees);
      SmartDashboard.putNumber("Turret/PID-rate", output);
      output = MathUtil.clamp(output, -0.25, 0.25);

      if (Math.abs(targetDegrees - turretFieldAngle) < 2.0) {
          setTurretPower(0);
          return 0;
      }

      setTurretPower(output);
      return output;
  }


  public double getFieldRelativeTurretAngle(double robotHeading) {
    double motorRotations = turretEncoder.getPosition();
    double turretDegrees = (motorRotations / 20.0) * 360.0;
    double fieldAngle = turretDegrees + robotHeading;
    return MathUtil.inputModulus(fieldAngle, -180, 180);
  }
        
  

  public double getAngleinDegrees() { 
    double motorRotations = turretEncoder.getPosition(); 
    double turretDegrees = (motorRotations / 20.0) * 360.0;
    return MathUtil.inputModulus(turretDegrees, -180, 180); 
  }
// NEO encoder gives rotations return rotations * 360.0; 
    /**
   * Command to set the subsystem setpoint. This will set the Turret its predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(TurretSetpoints setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kmaxRightSetpoint:
              TurretCurrentTarget = TurretSubSystemSetpoints.kMoveRightSetpoint;
              break;
            case kmaxLeftSetpoint:
              TurretCurrentTarget = -TurretSubSystemSetpoints.kMoveLeftSetpoint;
              break;
            case kBase:
              TurretCurrentTarget = TurretSubSystemSetpoints.kBase;
              break;

          }
        });
  }
 
  public boolean isSetpointReached(double setpoint){
    SmartDashboard.putNumber("Turret/TurretCurrentTarget", TurretCurrentTarget);
    double currentPosition = turretEncoder.getPosition();
    SmartDashboard.putNumber("Turret/TurretPosition", currentPosition);
    // TO DO: may need to adjust
    double setpointTolerance = 0.5; 
    boolean condition = Math.abs(currentPosition - TurretCurrentTarget) <= setpointTolerance; 
    SmartDashboard.putBoolean("Turret/Turret-isSetpointReached", condition);
    if ( condition ) {
      setTurretPower(0.);
      System.out.println("Turret Motor Stopped: ");
      return true;
    } else {
      return false;
    }
  }

  /** Sets turret motor speed (-1 to 1) */ 
  public void setMotor(double speed) {
     turretMotor.set(speed); 
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //moveToSetpoint();

    // Display subsystem values
    SmartDashboard.putNumber("Turret/Target Position", TurretCurrentTarget);
    SmartDashboard.putNumber("Turret/Actual Position", turretEncoder.getPosition());
  }

   /** UTILITY COMMANDS
   * Command to run the turret motor. 
   * Intended to step through to adjust proper setpoints
   * When the command is interrupted, e.g. the button is released, the motor will stop.
   */
  public Command runTurretRightCommand() {
    return this.startEnd(
        () -> this.setTurretPower(TurretSubsystemConstants.TurretSetpointTestSpeed), 
        () -> this.setTurretPower(0.0));
  }

  public Command runTurretLeftCommand() {
    return this.startEnd(
        () -> this.setTurretPower((-1) * TurretSubsystemConstants.TurretSetpointTestSpeed), 
        () -> this.setTurretPower(0.0));
  }

  

  /*
  Functionality: soemthing that makes sure that 
  the motor is in constant alignement with a setpoint 

  How are we going to get the turret to align: some sort of align command 

  
   */
  

  

}

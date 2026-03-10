package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

public class Constants {
  public static final class FieldConstants {
    // starting points field dims: 27ft x 54ft = 8.07m x 16.54m
    public static double blueStart_Right_X = 3.5;
    public static double blueStart_Right_Y = 0.6;
    public static double blueStart_Left_X = 3.5;
    public static double blueStart_Left_Y = 7.38;

    public static double redStart_Right_X = 13.04;
    public static double redStart_Right_Y = 7.38;
    public static double redStart_Left_X = 13.04;
    public static double redStart_Left_Y = 0.6;

    // Fuel Towers
    public static double blueTower_X = 4.62;
    public static double blueTower_Y = 4.035;

    public static double redTower_X = 11.92;
    public static double redTower_Y = 4.035;

  }
  public static final class TurretSubsystemConstants {
      public static final int kTurretMotorCanId = 44;
  
      public static double kTurretRampRate = 0.1;
      public static int    kTurretCurrentLimit = 40;
      public static double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
      public static double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);
  
      public static final double kTurretKp = 0.5;
      public static final double kTurretKi = 0;
      public static final double kTurretKd = 0.;
      
      public static final double TurretSetpointTestSpeed = 0.1;
    
      public static final class TurretSubSystemSetpoints {
        public static final double kBase = 0;
        public static final double kMoveRightSetpoint = 17.5;//135.;//Math.PI / 4.;
        public static final double kMoveLeftSetpoint = 35.;//Math.PI / 4.;
        }
  
  }

  public static final class HoodSubsystemConstants {
      public static final int kHoodMotorCanId = 22;
  
      public static double kHoodRampRate = 0.1;
      public static int    kHoodCurrentLimit = 40;
      public static double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
      public static double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);
  
      public static final double kHoodKp = 0.5;
      public static final double kHoodKi = 0;
      public static final double kHoodKd = 0.;
      
      public static final double HoodSetpointTestSpeed = 0.5;
    
      public static final class HoodSubSystemSetpoints {
        public static final double kBase = 0;
        public static final double k6ftSetpoint = 2.9;
        public static final double k9ftSetpoint = 5.2;
        public static final double k13ftSetpoint = 8.58;
        public static final double k15ftSetpoint = 10.08;
        public static double kVariableSetpoint = 7.5;
        }
  
  }

  public static final class CANRangeConstants {
    public static final int kCANRangeId = 66;

    public static int    kMinSignalStrengthForValidMeasurement = 2000;
    public static double kProximityThreshold = 0.1;

    public static final double kUpdatePeriod = 0.5; // Update every 500 ms
  }
  
  /* *********************
   * IntakeSubsystemConstants
   */
  public static final class IntakeSubsystemConstants{
    public static final int kIntakeMotorCanId = 19;

    public static double kIntakeRampRate = 0.1;
    public static int    kIntakeCurrentLimit = 60;
    public static double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);

    public static final double IntakeSetpointTestSpeed = 0.5;
  }

  /* *********************
   * IntakeSubsystemConstants
   */
  public static final class BallHandlingSubsystemConstants{
    public static final int kIntakeMotorCanId = 19;

    public static final int kLeaderFeederMotorCanId = 23; // previously 20
    public static final int kFollowerFeederMotorCanId = 21;

    public static double kIntakeRampRate = 0.1;
    public static int    kMaxCurrentLimit = 60;
    public static double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);

    public static final double IntakeSetpointTestSpeed = 0.5;
  }

  /* *********************
   * HopperSubsystemConstants
   */
  public static final class HopperSubsystemConstants{
    public static final int kHopperMotorCanId = 20; // previously 50

    public static double kHopperRampRate = 0.1;
    public static int    kHopperCurrentLimit = 40;
    public static double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);

    public static final double kHopperKp = 0.1;
    public static final double kHopperKi = 0;
    public static final double kHopperKd = 0.;

    public static final double HopperSetpointTestSpeed = 0.5;
    // Hopper control parameters
    public static final class HopperSubSystemSetpoints {
      public static final double kBase = 0;
      public static final double ktiltedSetpoint = 0.;
     }

  }

}

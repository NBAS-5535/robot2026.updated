package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

public class Constants {
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

    public static final class CANRangeConstants {
      public static final int kCANRangeId = 16;
  
      public static int    kMinSignalStrengthForValidMeasurement = 2000;
      public static double kProximityThreshold = 0.1;

      public static final double kUpdatePeriod = 0.5; // Update every 500 ms
    }
  

}

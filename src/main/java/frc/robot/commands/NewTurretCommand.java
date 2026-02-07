package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.DynamicTurretSubsystem;

public class NewTurretCommand extends Command {

    private final DynamicTurretSubsystem dynamic_turret;
    //private final TurretSubsystem turret;
    private final CommandSwerveDrivetrain drivetrain;

    private static final double targetX = 11.7; //Blue=4.55;
    private static final double targetY = 3.97;
    private static final double gearratio = 70.0;

    private final PIDController pid = new PIDController(0.025, 0, 0);

    public NewTurretCommand(/*TurretSubsystem turret*/ DynamicTurretSubsystem turret ,CommandSwerveDrivetrain drivetrain) {
        this.dynamic_turret = turret;
        this.drivetrain = drivetrain;

        pid.enableContinuousInput(-180, 180);

        addRequirements(turret);
    }

     @Override
    public void initialize() {
        pid.setTolerance(10);
  }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getCurrentPose();
        double rotationangle = robotPose.getRotation().getDegrees();
        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();
        double targetFieldAngle = Math.atan2(dy, dx);
        double targetfieldangledegrees = Math.toDegrees(targetFieldAngle);
        targetfieldangledegrees = MathUtil.inputModulus(targetfieldangledegrees, -180,180);
        double dynamicmotorPosition = dynamic_turret.getPosition();
        double dynmaicturretencoderangle = dynamicmotorPosition * gearratio * dynamic_turret.encoderConversionFactor();
        double motorcountsperrev = dynamic_turret.encoderConversionFactor();
        double dynamicangle = Math.toDegrees((dynamic_turret.encoderConversionFactor() * dynamic_turret.getPosition() / gearratio) / 360);
        //dynmaic_turret.setPointAtTargetSetpointValue(targetFieldAngle);
        //dynmaic_turret.moveToSetpoint();
         double output = pid.calculate(dynamicangle, targetFieldAngle);
        double desiredTurretAngle = MathUtil.inputModulus( targetFieldAngle - rotationangle, -180, 180 );
        SmartDashboard.putNumber("Ttest/targetFieldAngle(rad)", targetFieldAngle);
        SmartDashboard.putNumber("Ttest/targetFieldAngle(deg)", targetfieldangledegrees);
        SmartDashboard.putNumber("Ttest/dynamicencoderposition", dynamicmotorPosition);
        SmartDashboard.putNumber("Ttest/dynamicturretencoderangle", dynmaicturretencoderangle);
        SmartDashboard.putNumber("Ttest/dynamicAngle", dynamicangle);
        SmartDashboard.putNumber("Ttest/Motorcounts", motorcountsperrev);
        SmartDashboard.putNumber("Ttest/PIDoutput",output);

        //dynamic_turret.setDynamicTurretPower(output);
        //dynamic_turret.setDynamicTurretPower(output);


        
        /* 
        double turretAngle = (turret.turretEncoder.getPosition()) * 360 / gearratio;
        
        turretAngle = MathUtil.inputModulus(turretAngle, -180, 180);
        // Angle from robot to target (field-relative)
        double desiredTurretAngle = MathUtil.inputModulus( targetFieldAngle - rotationangle, -180, 180 );
        //turret.rotate(desiredTurretAngle,rotationangle);
        SmartDashboard.putNumber("Ttest/ turret angle" , turretAngle);
        SmartDashboard.putNumber("Ttest/ turret Encoder Value",turret.turretEncoder.getPosition());
        
        
        SmartDashboard.putNumber("Ttest/RotationAngle", rotationangle);
        SmartDashboard.putNumber("Ttest/TargetFieldAngle", targetFieldAngle);
        SmartDashboard.putNumber("Ttest/TurretAngle", turret.getFieldRelativeTurretAngle(rotationangle));
        SmartDashboard.putNumber("Ttest/Targetx" , targetX);
        SmartDashboard.putNumber("Ttest/Targety" , targetY);
        SmartDashboard.putNumber("Ttest/RobotPoseX" , robotPose.getX());
        SmartDashboard.putNumber("Ttest/RobotPoseY" , robotPose.getY());
        
       
        
        // Robot heading (field-relative)
         
        double robotHeading = robotPose.getRotation().getDegrees();

        // Convert to turret-relative angle
       
       
        //output = MathUtil.clamp(output, -0.25, 0.25);

        

        SmartDashboard.putNumber("Ttest/Targetx" , targetX);
        SmartDashboard.putNumber("Ttest/Targety" , targetY);
        SmartDashboard.putNumber("Ttest/RobotPoseX" , robotPose.getX());
        SmartDashboard.putNumber("Ttest/RobotPoseY" , robotPose.getY());
        SmartDashboard.putNumber("Ttest/targetFieldAngle" , targetFieldAngle);
        SmartDashboard.putNumber("Ttest/robotHeading" , robotHeading);
        SmartDashboard.putNumber("Ttest/PIDOutput" , output);
        */
        /* 
        SmartDashboard.putNumber("Ttest/desiredTurretAngle" , desiredTurretAngle);
        SmartDashboard.putNumber("Ttest/currentTurretAngle" , currentTurretAngle);
       
        SmartDashboard.putNumber("Ttest/TurretRawAngle", turret.getAngle());
        SmartDashboard.putNumber("Ttest/TurretDegrees", turretDegrees);
        SmartDashboard.putNumber("Ttest/MotorRotations", motorRotations);
        SmartDashboard.putNumber("Ttest/CurrentTurretAngle", currentTurretAngle);
        */
        
        
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}

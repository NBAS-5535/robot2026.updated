package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DynamicTurretSubsystem;
import frc.robot.subsystems.DynamicTurretSubsystem.DynamicTurretSetpoints;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AutoAlignCommand extends Command {
    private final DynamicTurretSubsystem turret;
    private final CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.FieldCentric rotationRequest =
        new SwerveRequest.FieldCentric();

    public AutoAlignCommand(DynamicTurretSubsystem turret, CommandSwerveDrivetrain drivetrain) {
        this.turret = turret;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);   // DO NOT require turret
    }
    @Override
    public void initialize() { // Enable turret tracking 
        turret.setSetpointCommand(DynamicTurretSetpoints.kPointAtTargetSetpoint).schedule();
     }

    @Override
public void execute() {

    // Update turret tracking every cycle
    turret.setSetpointCommand(DynamicTurretSetpoints.kPointAtTargetSetpoint).schedule();

    double target = turret.setPointAtTargetSetpointValue();
    double current = turret.getPosition();
    double error = target - current;

    double kP = 0.015;
    double rotationSpeed = kP * error;

    drivetrain.applyRequest(() ->
        rotationRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(rotationSpeed)
    );
}


    @Override
    public void end(boolean interrupted) {
        drivetrain.applyRequest(() ->
            rotationRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }
}

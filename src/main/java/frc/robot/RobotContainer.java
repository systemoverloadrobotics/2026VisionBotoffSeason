package frc.robot;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class RobotContainer {
    // 1. Define telemetry / physical limits from TunerConstants
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
    // 2. Instantiate Subsystem & Controller using the correct factory constants
        // 2. Instantiate Subsystem & Controller using the correct 2026 template factory method
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); 
    private final CommandXboxController driverController = new CommandXboxController(0);




    // 3. Define Drive Request Types
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Set default field-centric drive behavior using controller joysticks
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> drive
                .withVelocityX(-driverController.getLeftY() * MaxSpeed)
                .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                .withRotationalRate(-driverController.getRightX() * MaxAngularRate)
            )
        );

        // Bind driver 'A' button to X-Brake the wheels
        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Bind driver 'Y' button to reset the robot heading (Field Oriented Heading Reset)
        driverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }
        /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public edu.wpi.first.wpilibj2.command.Command getAutonomousCommand() {
        // Returns an empty command that ends instantly (does nothing)
        return edu.wpi.first.wpilibj2.command.Commands.none();
    }

  }
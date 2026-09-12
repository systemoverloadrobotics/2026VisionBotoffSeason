package frc.robot;

import frc.robot.subsystems.VisionSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.commands.MapVisibleTagsCommand;
import frc.robot.Constants.AimConstants;
import frc.robot.Constants.RoutineConstants;
import frc.robot.commands.AimAtTagCommand;
import frc.robot.commands.DriveRoutines;
import frc.robot.commands.DriveToTagCommand;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class RobotContainer {
    // 1. Define telemetry / physical limits from TunerConstants
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 
    // 2. Instantiate Subsystem & Controller using the correct factory constants
        // 2. Instantiate Subsystem & Controller using the correct 2026 template factory method
    private final VisionSwerveDrivetrain drivetrain = new VisionSwerveDrivetrain();
    private final VisionSubsystem vision = new VisionSubsystem(drivetrain);
    private final CommandXboxController driverController = new CommandXboxController(0);

    /** Which tag the aiming commands target. Cycled with the D-pad; defaults to 16. */
    private int selectedTagId = AimConstants.kDefaultTargetTagId;




    // 3. Define Drive Request Types
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1)
        .withRotationalDeadband(MaxAngularRate * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public RobotContainer() {
        configureButtonBindings();
    }

    /**
     * True when the driver has pushed either stick far enough to mean it.
     *
     * An automated routine owns the drivetrain while it runs, so the sticks are
     * dead until it finishes. This lets the driver take the robot back by just
     * driving, instead of reaching for the disable button - which matters when
     * the robot is heading somewhere unexpected in a shop.
     */
    private boolean driverWantsControl() {
        double deflection = Math.max(
            Math.hypot(driverController.getLeftX(), driverController.getLeftY()),
            Math.abs(driverController.getRightX()));
        return deflection > RoutineConstants.kAbortStickDeflection;
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

        // 'Y' is DELIBERATELY left unbound. seedFieldCentric() resets the robot's
        // HEADING (Phoenix's JNI_SeedFieldCentric - "Resets the rotation of the
        // robot pose"), not its full pose. Vision is what establishes the field
        // frame here; re-seeding heading after that would fabricate a heading
        // disagreeing with the fused translation, and vision could not correct
        // it afterward - kThetaStdDevRadians is 0.5 rad (~29 degrees), so
        // heading corrections from vision are nearly weightless against a bad
        // seed. Per the design (spec Sec3.1), a pose reset appears in exactly
        // one place - the layout-mapping utility - not here.

        // Back - one-shot print of everything needed to hand-verify localization
        // against a tape measure. An InstantCommand, not a PrintCommand, because the
        // text has to be built from live sensor data at press time.
        driverController.back().onTrue(Commands.runOnce(() -> {
            var pose = drivetrain.getRobotPose();
            System.out.printf(
                "Pose: (%.3f, %.3f) heading=%.1fdeg  yawRate=%.2fdeg/s  "
                    + "visionHealthy=%s  visibleTags=%s  selectedTag=%d  targetInLayout=%s%n",
                pose.getX(), pose.getY(), pose.getRotation().getDegrees(),
                drivetrain.getYawRateDegPerSec(),
                vision.isHealthy(),
                java.util.Arrays.toString(vision.getVisibleTagIds()),
                selectedTagId,
                vision.getTargetPosition(selectedTagId).isPresent());
        }));

        // Practice-space layout authoring. D-pad, never stick clicks.
        // Remove or disable these bindings for competition.
        driverController.povUp().onTrue(MapVisibleTagsCommand.record(drivetrain, vision));
        driverController.povDown().onTrue(MapVisibleTagsCommand.write());

        // B - rotate in place to face the selected tag.
        driverController.b().onTrue(
            new AimAtTagCommand(drivetrain, vision, () -> selectedTagId)
                .withTimeout(AimConstants.kCommandTimeoutSeconds));

        // X - drive to the standoff distance from the selected tag, facing it.
        // This is the stand-in for taking a shot.
        driverController.x().onTrue(
            new DriveToTagCommand(drivetrain, vision, () -> selectedTagId)
                .withTimeout(AimConstants.kCommandTimeoutSeconds));

        // --- Drivebase verification routines (no camera required) ---------
        //
        // Each returns to its own starting point so the closing error can be
        // measured against a tape mark. Run SQUARE-HOLD and SQUARE-TURNS back
        // to back: if the held-heading square closes well and the turning one
        // does not, the difference is the gyro and the rotational half of the
        // kinematics, not the modules.
        //
        // All three abort if the driver moves a stick past a deadband, so the
        // robot can be taken back without reaching for the disable button.
        driverController.leftBumper().onTrue(
            DriveRoutines.outAndBack(drivetrain, this::driverWantsControl));
        driverController.rightBumper().onTrue(
            DriveRoutines.squareHoldingHeading(drivetrain, this::driverWantsControl));
        driverController.start().onTrue(
            DriveRoutines.squareWithTurns(drivetrain, this::driverWantsControl));

        // D-pad left/right - cycle the selected tag. Never stick clicks: pressing a
        // stick down deflects it at the same time, so the binding fires while the
        // robot is also being commanded to move.
        driverController.povRight().onTrue(Commands.runOnce(() -> {
            selectedTagId++;
            System.out.println("Selected target tag: " + selectedTagId);
        }));
        driverController.povLeft().onTrue(Commands.runOnce(() -> {
            selectedTagId--;
            System.out.println("Selected target tag: " + selectedTagId);
        }));
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
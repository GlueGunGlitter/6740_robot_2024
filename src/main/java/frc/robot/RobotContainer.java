package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.automations.*;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.driveCommands.TeleopSwerve;
import frc.robot.commands.driveCommands.TrunToAngle;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    public static final XboxController xboxController = new XboxController(0);
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    POVButton d_Uppov = new POVButton(driver, 0);
    POVButton d_Rightpov = new POVButton(driver, 90);
    POVButton d_Downpov = new POVButton(driver, 180);
    POVButton d_Leftpov = new POVButton(driver, 270);

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton testCmd = new JoystickButton(driver, XboxController.Button.kX.value);

    // private final JoystickButton IntakeEnableCommand = new JoystickButton(driver,
    // XboxController.Button.kRightBumper.value);
    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    public static final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    public static final ShooterCommand m_ShooterCommand = new ShooterCommand();
    public static final TransportationSubsystem m_TransportationSubsystem = new TransportationSubsystem();
    public static final TransportationCommand m_TransportationCommand = new TransportationCommand();

    private final SendableChooser<Command> autoChooser;

    /**
     * The container
     * for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Build an auto chooser. This will use Commands.none() as the default option.
        // Command intake = Commands.run(() -> m_TransportationSubsystem.setSpeed(0.6,
        // 0.6, 1));

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        registerCommands();
        configureButtonBindings();

        autoChooser = AutoBuilder.buildAutoChooser();

        Shuffleboard.getTab("Robot")
                .add("Sendable Title", autoChooser);
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));

        // Configure the button bindings

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        testCmd.onTrue(new enableHighShooter());
        d_Uppov.onTrue(new TrunToAngle(s_Swerve, 0));
        d_Rightpov.onTrue(new TrunToAngle(s_Swerve, 90));
        d_Downpov.onTrue(new TrunToAngle(s_Swerve, 180));
        d_Leftpov.onTrue(new TrunToAngle(s_Swerve, 270));

        // IntakeEnableCommand.onTrue(Commands.sequence(intake));
    }

    private void registerCommands() {
        NamedCommands.registerCommand("HighShooter", new enableHighShooter());
        NamedCommands.registerCommand("LowShooter", new enableLowShooter());
        NamedCommands.registerCommand("StopShooter", new disableShooter());
        NamedCommands.registerCommand("StartTransportation", new enableForwardTransportation());
        NamedCommands.registerCommand("StopTransportation", new disableTransportation());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}

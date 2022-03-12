package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private final SwerveDrive m_swerveDrive;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final boolean m_fieldRelative;

    public DefaultDriveCommand(SwerveDrive swerveDrive,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier, boolean fieldRelative) {
        this.m_swerveDrive = swerveDrive;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_fieldRelative = fieldRelative;

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
            m_swerveDrive.drive(
                    m_translationXSupplier.getAsDouble(),
                    m_translationYSupplier.getAsDouble(),
                    m_rotationSupplier.getAsDouble(),
                    m_fieldRelative
            );
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.drive(0, 0, 0, false);
    }
}

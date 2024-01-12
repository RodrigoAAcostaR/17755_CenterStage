package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.BrazoDefault;
import org.firstinspires.ftc.teamcode.commands.TankDriveCommand;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.subsystems.Brazo;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.TankDriveSubsystem;

@TeleOp
public class TeleOpMode2P extends CommandOpMode {
    @Override
    public void initialize() {
        GamepadEx gamepadDriver = new GamepadEx(gamepad1);
        GamepadEx gamepadX = new GamepadEx(gamepad2);

        SampleTankDrive sampleTankDrive = new SampleTankDrive(hardwareMap);
        TankDriveSubsystem driveSystem = new TankDriveSubsystem(sampleTankDrive);
        Intake intake = new Intake(telemetry, hardwareMap);
        Brazo brazo = new Brazo(hardwareMap, telemetry);

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(() -> intake.setPower(1))
                .whenReleased(() -> intake.setPower(0));

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(() -> intake.setPower(-.8))
                .whenReleased(() -> intake.setPower(0));

        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.Y)
                .whenPressed(() -> brazo.setPosition(4100));

        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.B)
                .whenPressed(() -> brazo.setPosition(350));

        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.X)
                .whenPressed(() -> brazo.setPosition(3700));

        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.A)
                .whenPressed(() -> brazo.setPosition(0))
                .whenPressed(() -> intake.hold());

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_UP)
                .whenPressed(()-> brazo.launch());

        new GamepadButton(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(()-> brazo.agarrar());

        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(()-> intake.hold());

        new GamepadButton(new GamepadEx(gamepad2), GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(()-> intake.leave());

        driveSystem.setDefaultCommand(new TankDriveCommand(
                driveSystem, () -> -gamepadDriver.getLeftY(), gamepadDriver::getRightX
        ));

        schedule(new RunCommand(() -> {
            driveSystem.update();
            telemetry.addData("Heading", driveSystem.getPoseEstimate().getHeading());
            telemetry.update();
        }));
    }
}

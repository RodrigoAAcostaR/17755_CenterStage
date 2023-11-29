package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@Autonomous
public class AutoAzul extends LinearOpMode {

    private DcMotor leftDrive = null, rightDrive = null;
    private IMU imu = null;
    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftSpeed     = 0;
    private double  rightSpeed    = 0;
    private int     leftTarget    = 0;
    private int     rightTarget   = 0;
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     VELOCIDADBACKDROP       = .1;
    static final double     TURN_SPEED              = 0.2;
    static final double     HEADING_THRESHOLD       = 1.0;
    static final double     P_TURN_GAIN            = 0.02;
    static final double     P_DRIVE_GAIN           = 0.03;
    DcMotorEx intake, brazo;
    ServoEx holder;

    @Override
    public void runOpMode() throws InterruptedException {
        leftDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        rightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        holder = new SimpleServo(hardwareMap, "holder", 0, 180, AngleUnit.DEGREES);
        brazo = hardwareMap.get(DcMotorEx.class, "brazo");
        brazo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brazo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brazo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        brazo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brazo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        hold();
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.update();
        }
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetHeading();

        //Azul derecha
        driveStraight(DRIVE_SPEED, 23.0, 0.0);
        turnToHeading( TURN_SPEED, -90.0);
        setPosition(500);
        setPower(-.4);
        sleep(1000);
        setPower(0);
        driveStraight(DRIVE_SPEED, -21, -90);
        driveStraight(DRIVE_SPEED, -10, -90);
        setPosition(4100);
        sleep(2000);
        driveStraight(VELOCIDADBACKDROP, -2, -90);
        sleep(1000);
        leave();
        sleep(1500);
        driveStraight(VELOCIDADBACKDROP, 2, -90);
        sleep(700);
        setPosition(0);
        hold();

/*
        //Azul medio
        driveStraight(DRIVE_SPEED, 19.0, 0.0);
        setPosition(450);
        setPower(-.4);
        sleep(1000);
        setPower(0);
        driveStraight(DRIVE_SPEED, -2, 0.0);
        turnToHeading(TURN_SPEED, -90);
        driveStraight(DRIVE_SPEED, -29, -90);
        setPosition(4200);
        sleep(2000);
        driveStraight(VELOCIDADBACKDROP, -2, -90);
        sleep(1000);
        leave();
        sleep(1500);
        driveStraight(VELOCIDADBACKDROP, 2, -90);
        sleep(700);
        setPosition(0);
        hold();


        //Azul izquierda
        driveStraight(DRIVE_SPEED, 25.0, 0.0);
        turnToHeading( TURN_SPEED, 90);
        turnToHeading(TURN_SPEED, 0);
        driveStraight(DRIVE_SPEED, -10, 0.0);
        turnToHeading(TURN_SPEED, 90);
        driveStraight(DRIVE_SPEED, 23, 90);

         */


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    public void setPower( double power){
        intake.setPower(power);
    }

    public void setPosition(int pos){
        brazo.setTargetPosition(pos);
        brazo.setPower(1);
        brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void hold(){
        holder.turnToAngle(2);
    }

    public void leave(){
        holder.turnToAngle(155);
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {

        getSteeringCorrection(heading, P_DRIVE_GAIN);

        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            moveRobot(0, turnSpeed);

            sendTelemetry(false);
        }

        moveRobot(0, 0);
    }

    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftDrive.getCurrentPosition(),
                    rightDrive.getCurrentPosition());

        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        telemetry.addData("Brazo", brazo.getCurrentPosition());
        telemetry.update();

    }

    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        if (opModeIsActive()) {

            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftTarget = leftDrive.getCurrentPosition() + moveCounts;
            rightTarget = rightDrive.getCurrentPosition() + moveCounts;

            leftDrive.setTargetPosition(leftTarget);
            rightDrive.setTargetPosition(rightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            while (opModeIsActive() &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                if (distance < 0)
                    turnSpeed *= -1.0;

                moveRobot(driveSpeed, turnSpeed);

                sendTelemetry(true);
            }

            moveRobot(0, 0);
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;

        robotHeading = getRawHeading() - headingOffset;

        headingError = targetHeading - robotHeading;

        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            moveRobot(0, turnSpeed);

            sendTelemetry(false);
        }

        moveRobot(0, 0);
    }

    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;
        turnSpeed  = turn;

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftDrive.setPower(leftSpeed);
        rightDrive.setPower(rightSpeed);
    }



    public double getRawHeading() {
        Orientation angles   = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void resetHeading() {
        headingOffset = getRawHeading();
        robotHeading = 0;
    }
}

package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class AutoAzulLargo extends LinearOpMode {

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

    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 720; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels
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

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);

        boolean isLeft = false, isRight = false, isMiddle = false;

        while(!isStarted()){
            if(cX > 0 &&  cX < 350 && width < 5) {
                telemetry.addLine("Izquierda");
                telemetry.addData("Coordenadas: ", (int)cX  + ", " + (int) cY);
                isRight = false;
                isMiddle = false;
                isLeft = true;
            } else if(cX > 350 && cX < 600 && width < 5) {
                telemetry.addLine("En medio");
                telemetry.addData("Coordenadas: ", (int)cX  + ", " + (int) cY);
                isMiddle = true;
                isLeft = false;
                isRight = false;
            }
            else {
                telemetry.addLine("A la derecha");
                isLeft = false;
                isMiddle = false;
                isRight = true;
            }
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.update();
        }

        waitForStart();
        //sleep(2000);
        hold();

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetHeading();


        if(isLeft) {
            izquierda();
        } else if(isMiddle) {
            medio();
        }
        else {
            derecha();

        }


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    public void izquierda(){

        driveStraight(DRIVE_SPEED, -2, 0.0);
        turnToHeading(TURN_SPEED, 180);
        driveStraight(DRIVE_SPEED, 15, 180);
        turnToHeading(TURN_SPEED,90);
        driveStraight(DRIVE_SPEED,5,90);
        brazoNeutral();
        setPower(-.4);
        sleep(1000);
        setPower(0);
        driveStraight(DRIVE_SPEED,-10,90);
        /*
        turnToHeading(TURN_SPEED,45);
        driveStraight(DRIVE_SPEED,-8,45);
        turnToHeading(TURN_SPEED,90);
        driveStraight(DRIVE_SPEED,-12,90);
        brazoDejar();
        sleep(3000);
        leave();
        sleep(1000);
        brazoAbajo();
        sleep(2000);
        driveStraight(DRIVE_SPEED,6,90);
        turnToHeading(TURN_SPEED,0);
        driveStraight(DRIVE_SPEED,-13,0);
        turnToHeading(TURN_SPEED,90);
        driveStraight(DRIVE_SPEED,-10,90);

         */

    }

    public void medio(){
        driveStraight(DRIVE_SPEED, -2, 0.0);
        turnToHeading(TURN_SPEED, 180);
        driveStraight(DRIVE_SPEED, 17, 180);
        brazoNeutral();
        setPower(-.4);
        sleep(1000);
        setPower(0);
        driveStraight(DRIVE_SPEED, -9, 180);
        /*
        turnToHeading(TURN_SPEED, 45);
        driveStraight(DRIVE_SPEED, -8, 45);
        turnToHeading(TURN_SPEED,90);
        driveStraight(DRIVE_SPEED,-18,90);
        brazoDejar();
        sleep(3000);
        leave();
        sleep(1000);
        brazoAbajo();
        sleep(2000);
        driveStraight(DRIVE_SPEED,6,90);
        turnToHeading(TURN_SPEED,0);
        driveStraight(DRIVE_SPEED,-18,0);
        turnToHeading(TURN_SPEED,90);
        driveStraight(DRIVE_SPEED,-14,90);

         */

    }

    public void derecha(){
        driveStraight(DRIVE_SPEED, -10, 0.0);
        turnToHeading(TURN_SPEED, 45);
        driveStraight(DRIVE_SPEED, -13, 45);
        brazoNeutral();
        setPower(-.4);
        sleep(1000);
        setPower(0);
        driveStraight(DRIVE_SPEED, -5, 45);
        /*
        turnToHeading(TURN_SPEED,145);
        driveStraight(DRIVE_SPEED,-13,145);
        turnToHeading(TURN_SPEED,90);
        driveStraight(DRIVE_SPEED,-4,90);
        brazoDejar();
        sleep(3000);
        leave();
        sleep(1000);
        brazoAbajo();
        sleep(2000);
        driveStraight(DRIVE_SPEED,6,90);
        turnToHeading(TURN_SPEED,0);
        driveStraight(DRIVE_SPEED,-18,0);
        turnToHeading(TURN_SPEED,90);
        driveStraight(DRIVE_SPEED,-19,90);

         */
    }

    public void brazoNeutral(){
        brazo.setTargetPosition(500);
        brazo.setPower(1);
        brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);    }

    public void brazoDejar(){//5300
        brazo.setTargetPosition(5350);
        brazo.setPower(1);
        brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void brazoAbajo(){
        brazo.setTargetPosition(0);
        brazo.setPower(0.65);
        brazo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    private void initOpenCV() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "camara1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new AutoAzulLargo.BlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    class BlobDetectionPipeline extends OpenCvPipeline {
        Mat blueMask;
        Mat hierarchy = new Mat();
        @Override
        public Mat processFrame(Mat input) {
            blueMask = preprocessFrame(input);
            // Preprocess the frame to detect blue regions
            // Find contours of the detected blue regions
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(blueMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest blue contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a blue outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();




                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }
            return input;
        }
        Mat hsvFrame = new Mat();
        private Mat preprocessFrame(Mat frame) {
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerblue = new Scalar(0, 100, 100);
            Scalar upperblue = new Scalar(60, 255, 255);


            blueMask = new Mat();
            Core.inRange(hsvFrame, lowerblue, upperblue, blueMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_CLOSE, kernel);

            return blueMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }
}

package teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BallOrCube extends LinearOpMode {
    OpenCvCamera webcam;
    SamplePipeline pipeline;
    HubPipeline blackPipeline;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor intakeMotor = null;
    private Servo intakeServo = null;

    double leftFrontPower = 0;
    double rightFrontPower = 0;
    double leftBackPower = 0;
    double rightBackPower = 0;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new SamplePipeline();
        blackPipeline = new HubPipeline();

        webcam.setPipeline(pipeline);

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_drive_front");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_drive_back");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_drive_front");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_drive_back");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeServo = hardwareMap.get(Servo.class, "intake_servo");

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                throw new RuntimeException();
            }
        });

        waitForStart();

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*
        intakeMotor.setPower(0.2);

        while(opModeIsActive() && intakeMotor.getCurrentPosition() < 1000){
            telemetry.addData("Status", "moving lift");
            telemetry.update();
        }

        intakeMotor.setPower(0);
        */

        moveForward();

        while (opModeIsActive() && pipeline.getType() != SamplePipeline.TYPE.CUBE) {
            telemetry.addData("Type", pipeline.getType());
            telemetry.addData("Average", pipeline.getAverage());
            telemetry.addData("Status", "Searching for cube");
            telemetry.update();
            sleep(50);
        }

        stopMoving();

        /*
        intakeMotor.setPower(-0.2);

        while(opModeIsActive() && intakeMotor.getCurrentPosition() > 0){
            telemetry.addData("Status", "Lowering lift");
            telemetry.update();
        }


        intakeMotor.setPower(0);
        */

        intakeServo.setPosition(180);

        while(opModeIsActive() && intakeServo.getPosition() < 1){
            telemetry.addData("Status", "grabbing");
            telemetry.addData("Servo Status", intakeServo.getPosition());
            telemetry.update();
        }


        /*intakeMotor.setPower(0.2);

        while(opModeIsActive() && intakeMotor.getCurrentPosition() < 1000){
            telemetry.addData("Status", "Raising lift");
            telemetry.update();
        }

        */

        webcam.setPipeline(blackPipeline);

        leftFrontDrive.setPower(0.2);
        leftBackDrive.setPower(-0.2);
        rightFrontDrive.setPower(-0.2);
        rightBackDrive.setPower(0.2);

        while(opModeIsActive() && !blackPipeline.getIsBlack()){
            telemetry.addData("Status", "looking for alliance hub");
            telemetry.update();
        }

        stopMoving();



        /*
        while (opModeIsActive() && pipeline.getType() != SamplePipeline.TYPE.CUBE) {
            telemetry.addData("Type", pipeline.getType());
            telemetry.addData("Average", pipeline.getAverage());
            telemetry.update();
            sleep(50);


        }

        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        webcam.setPipeline(blackPipeline);

        while (opModeIsActive()) {
            telemetry.addData("is black", blackPipeline.getIsBlack());
            telemetry.addData("Average", blackPipeline.getAverage());
            telemetry.update();
            sleep(50);


        }*/


    }

    public static class SamplePipeline extends OpenCvPipeline {
        private static final Scalar BLUE = new Scalar(0, 0, 255);

        private static final int THRESHOLD = 107;

        Point topLeft = new Point(300, 240);
        Point bottomRight = new Point(340, 360);

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        private volatile int average;
        private volatile TYPE type = TYPE.BALL;

        private void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat input) {
            inputToCb(input);

            region1_Cb = Cb.submat(new Rect(topLeft, bottomRight));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            average = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(input, topLeft, bottomRight, BLUE, 2);

            if (average > THRESHOLD) {
                type = TYPE.BALL;
            } else {
                type = TYPE.CUBE;
            }

            return input;
        }

        public TYPE getType() {
            return type;
        }

        public int getAverage() {
            return average;
        }

        public enum TYPE {
            BALL, CUBE
        }
    }

    public static class HubPipeline extends OpenCvPipeline {
        private static final Scalar BLUE = new Scalar(0, 0, 255);

        private static final int THRESHOLD = 25;

        Point topLeft = new Point(300, 0);
        Point bottomRight = new Point(340, 360);

        Mat region1_Y;
        Mat YCrCb = new Mat();
        Mat Y = new Mat();

        private volatile int average;
        private static boolean isBlack;

        private void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Y, 0);
        }

        @Override
        public void init(Mat input) {
            inputToCb(input);

            region1_Y = Y.submat(new Rect(topLeft, bottomRight));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            average = (int) Core.mean(region1_Y).val[0];

            Imgproc.rectangle(input, topLeft, bottomRight, BLUE, 2);

            if (average < THRESHOLD) {
                isBlack = true;
            } else {
                isBlack = false;
            }

            return input;
        }

        public boolean getIsBlack() {
            return isBlack;
        }

        public int getAverage() {
            return average;
        }

    }

    private void moveForward(){
        leftFrontDrive.setPower(0.2);
        leftBackDrive.setPower(0.2);
        rightFrontDrive.setPower(0.2);
        rightBackDrive.setPower(0.2);
    }

    private void stopMoving(){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
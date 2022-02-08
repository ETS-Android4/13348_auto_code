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

    int stageNum = 0;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new SamplePipeline();

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

        //raise intake

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setPower(0.2);

        while(opModeIsActive() && intakeMotor.getCurrentPosition() < 1000){
            telemetry.addData("Status", "moving lift");
            telemetry.update();
        }

        intakeMotor.setPower(0);

        OBJECTS currentObject = null;

        //move until it sees a duck or the red square on the ground
        moveForward();

        while(currentObject != OBJECTS.DUCK){
            while (opModeIsActive() && pipeline.getType() != SamplePipeline.TYPE.CUBE && !pipeline.getIsRed()) {
                telemetry.addData("Type", pipeline.getType());
                telemetry.addData("Average", pipeline.getAverage());
                telemetry.addData("Status", "Searching for cube");
                telemetry.update();
                sleep(50);
            }

            stopMoving();
            currentObject = checkObject();

            if(currentObject == OBJECTS.DUCK){
                grabObject();
            }else{
                stageNum++;
                moveRight();
            }
        }
        moveRight();

        //go right until it sees the center of the carousel
        pipeline.setTopLeft(300,0);
        pipeline.setBottomRight(340,360);

        while(opModeIsActive() && !pipeline.getIsRed()){
            telemetry.addData("Status", "looking for alliance hub");
            telemetry.update();
        }

        stopMoving();


    }

    public static class SamplePipeline extends OpenCvPipeline {
        private static final Scalar BLUE = new Scalar(0, 0, 255);

        private static final int THRESHOLD = 107;

        Point topLeft = new Point(300, 240);
        Point bottomRight = new Point(340, 360);

        private static boolean isRed = false;

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        Mat region1_Cr;
        Mat YCrCb2 = new Mat();
        Mat Cr = new Mat();

        private volatile int Cb_average;
        private volatile int Cr_average;
        private volatile TYPE type = TYPE.BALL;

        private void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);

            Imgproc.cvtColor(input, YCrCb2, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb2, Cr, 1);
        }

        @Override
        public void init(Mat input) {
            inputToCb(input);

            region1_Cb = Cb.submat(new Rect(topLeft, bottomRight));
            region1_Cr = Cr.submat(new Rect(topLeft, bottomRight));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            Cb_average = (int) Core.mean(region1_Cb).val[0];
            Cr_average = (int) Core.mean(region1_Cr).val[0];

            Imgproc.rectangle(input, topLeft, bottomRight, BLUE, 2);

            if (Cb_average > THRESHOLD) {
                type = TYPE.BALL;
            } else {
                type = TYPE.CUBE;
            }

            if (Cr_average > THRESHOLD) {
                isRed = true;
            } else {
                isRed = false;
            }

            return input;
        }

        public TYPE getType() {
            return type;
        }

        public int getAverage() {
            return Cb_average;
        }

        public boolean getIsRed(){
            return isRed;
        }

        public void setTopLeft(int x, int y){
            topLeft = new Point(x, y);
        }

        public void setBottomRight(int x, int y){
            bottomRight = new Point(x, y);
        }

        public enum TYPE {
            BALL, CUBE
        }
    }



    private void moveForward(){
        leftFrontDrive.setPower(0.2);
        leftBackDrive.setPower(0.2);
        rightFrontDrive.setPower(0.2);
        rightBackDrive.setPower(0.2);
    }

    private void moveLeft(){
        leftFrontDrive.setPower(0.2);
        leftBackDrive.setPower(-0.2);
        rightFrontDrive.setPower(-0.2);
        rightBackDrive.setPower(0.2);
    }

    private void moveRight(){
        leftFrontDrive.setPower(-0.2);
        leftBackDrive.setPower(0.2);
        rightFrontDrive.setPower(0.2);
        rightBackDrive.setPower(-0.2);
    }

    private void stopMoving(){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void grabObject(){
        //lower intake on top of the duck

        intakeMotor.setPower(-0.2);

        while(opModeIsActive() && intakeMotor.getCurrentPosition() > 0){
            telemetry.addData("Status", "Lowering lift");
            telemetry.update();
        }


        intakeMotor.setPower(0);

        //grab
        intakeServo.setPosition(180);

        while(opModeIsActive() && intakeServo.getPosition() < 1){
            telemetry.addData("Status", "grabbing");
            telemetry.addData("Servo Status", intakeServo.getPosition());
            telemetry.update();
        }

        //raise intake back up
        intakeMotor.setPower(0.2);

        while(opModeIsActive() && intakeMotor.getCurrentPosition() < 1000){
            telemetry.addData("Status", "Raising lift");
            telemetry.update();
        }

        intakeMotor.setPower(0);
    }

    private OBJECTS checkObject(){
        if(pipeline.getType() == SamplePipeline.TYPE.CUBE) {
            return OBJECTS.DUCK;
        }else if(pipeline.getIsRed()){
            return OBJECTS.SQUARE;
        }

        return null;


    }

    private enum OBJECTS{
        DUCK,
        SQUARE
    }
}
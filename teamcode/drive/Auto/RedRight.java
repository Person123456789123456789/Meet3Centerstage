package org.firstinspires.ftc.teamcode.drive.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.armsNStuff;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RedRight", group = "RED")
public class RedRight extends LinearOpMode {
    //armsNStuff arm;
    DcMotor Arm;
    Servo Box;
    CRServo Intake;
    OpenCvCamera webcam;
    GamePropRight.gamePropPosition propPosition = GamePropRight.gamePropPosition.LEFT;
    SampleMecanumDrive robot;
    int distancePark;
    @Override
    public void runOpMode() throws InterruptedException {

        //arm=new armsNStuff(hardwareMap);
        Arm=hardwareMap.get(DcMotor.class, "Arm");
        Box=hardwareMap.get(Servo.class, "Box");
        Intake=hardwareMap.get(CRServo.class, "Intake");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Box.setPosition(0.93);

        telemetry.addData("Start OpMode", "BLUE LEFT");
        telemetry.update();
        startCamera();
        telemetry.addData("Selected Starting Position", propPosition);
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive())
        {propPosition = getPropPosition();
            telemetry.addData("Identified Prop Location", propPosition);
            telemetry.update();
        }

        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            webcam.stopStreaming(); //Stop Webcam to preserve Controlhub cycles.
            runAutonoumousMode();
        }
    }

    public void runAutonoumousMode() {

        Pose2d initPose = new Pose2d(0, 0, Math.toRadians(0.0)); // Starting Pose --Update CoOrdinates
        Pose2d yellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d leftPose = new Pose2d(0, 0, 0);
        Pose2d purplePixelPose = new Pose2d(0, 0, Math.toRadians(0.0));;
        switch (propPosition) { //UPDATE THESE POSITIONS
            case LEFT:
                telemetry.addData("Left", "Left");
                yellowPixelPose = new Pose2d(29.5,-25, Math.toRadians(-90));
                purplePixelPose = new Pose2d(29, 4.75, Math.toRadians(90.0));
                leftPose = new Pose2d(29, -1, Math.toRadians(90.0));
                distancePark = 25;

                break;
            case CENTER:
                telemetry.addData("Center Position", "Center");
                yellowPixelPose = new Pose2d(25,-25 ,Math.toRadians(-90));
                purplePixelPose = new Pose2d(28, 3, Math.toRadians(0.0));
                leftPose = new Pose2d(24.5, 3, Math.toRadians(0.0));
                distancePark = 24;
                break;
            case RIGHT:
                leftPose = new Pose2d(21.5,-8.2, Math.toRadians(0));
                yellowPixelPose = new Pose2d(18, -25, Math.toRadians(-90));
                telemetry.addData("Left Position", "Left");
                purplePixelPose = new Pose2d(22,-8.2, Math.toRadians(0));
                distancePark = 18;
                break;

        }
        //Pose2d parkingPose = new Pose2d(-25, 75, -90); //UPDATE

        telemetry.update();
        sleep(200);

        robot = new SampleMecanumDrive(hardwareMap);
        robot.followTrajectorySequence(robot.trajectorySequenceBuilder(initPose) //Starting Pose
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Arm.setTargetPosition(300);Arm.setPower(0.8);Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);})
                .lineToLinearHeading(leftPose)
                .lineToLinearHeading(purplePixelPose)//Drop Purple Pixel
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> {2350})
                //.UNSTABLE_addTemporalMarkerOffset(0, () -> {0.45;})
                .back(5)
                .waitSeconds(0.3)
                .lineToLinearHeading(yellowPixelPose)
                //.waitSeconds(1)
                // .forward(4)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Arm.setTargetPosition(4900);Arm.setPower(0.8);Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);})
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {Box.setPosition(0.48);})
                .waitSeconds(2.2)
                .forward(4.5)
                .waitSeconds(0.5)
                .forward(.5)
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Intake.setPower(-0.2);})
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Intake.setPower(0);})
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Box.setPosition(0.97);})
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {Arm.setTargetPosition(0);Arm.setPower(0.5);Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);})
                .waitSeconds(3.5)
                .strafeRight(distancePark)
                .build());

    }
    public void startCamera() {
        //Initialize Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "FrontCam"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        webcam.setPipeline(new GamePropRight());
    }
	
	public GamePropRight.gamePropPosition getPropPosition() {
		return GamePropRight.position;
	}
	
}
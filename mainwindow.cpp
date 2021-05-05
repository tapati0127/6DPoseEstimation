#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "camera.h"
#include <opencv2/surface_matching/ppf_helpers.hpp>
using namespace Convert;
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ReadSettings();
    //UDP Connection Initialize
    motoudp = new MotoUDP(QHostAddress(ui->lineEditRobotIP->text()),10040);
    connect(motoudp, &MotoUDP::triggerPPF, this, &MainWindow::triggerByRobot);
    motoudp->start();

//    motoudp->connectMotoman();



    camera = new Camera(848, 480, 848, 480, 30,ui->lineEditCameraParam->text().toStdString());
    connect(camera, &Camera::framesReady, this, &MainWindow::receiveFrame);
    connect(camera, &Camera::pclReady, this, &MainWindow::receivePcl);
    connect(camera, &Camera::pointCloudReady, this, &MainWindow::receivePointCloud);
    camera->setMaxRange(ui->verticalSlider->value());
    connect(ui->verticalSlider,&QSlider::valueChanged,camera,&Camera::maxRangeChanged);
    connect(camera,&Camera::connected,this,&MainWindow::readyCameraStatus);
    camera->start();

    viewer = new pclViewer(ui->qvtkWidget);
    viewer_ = new pclViewer(ui->qvtkWidgetModel);

    servo = new ServoControl();


    ppf = new PPF(modelPath.toStdString());
    ppf->start();
    connect(ppf,&PPF::complete,this,&MainWindow::readyPPFStatus);
    cv::Matx44d mat(0.02518456169,  0.9961177387,  -0.0843515765,  0.2652645862,
                    0.9991963621,  -0.02245054794,  0.03320546554,  -0.2020528898,
                    0.03118281413,  -0.08512005347,  -0.9958826279,  0.1593551343,
                    0,  0,  0,  1);
    calib_pose = mat;

//    Convert::saveMatFile(mat,"/home/tapati/calib");

}

MainWindow::~MainWindow()
{
    WriteSettings();
    delete ui;
}

void MainWindow::receiveFrame(QImage rgb, QImage depth)
{
    ui->labelImage->setPixmap(QPixmap::fromImage(rgb).scaled(ui->labelImage->size().width(),ui->labelImage->size().height(),Qt::KeepAspectRatio));
    QString sizeString = QString("(%1,%2)").arg(ui->labelImage->size().width()).arg(ui->labelImage->size().height());
    //ui->labelSize->setText(sizeString);
}

void MainWindow::receivePcl(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcloud)
{
    //std::cout <<"Got PCL: " << pointcloud->points.size() << std::endl;
    viewer->displayPCL(pointcloud);
}

void MainWindow::receivePointCloud(cv::Mat pointcloud)
{
    if(isTrigger){
        if(isReady){
            ppf_match_3d::Pose3DPtr result;
            cv::Mat pc;
            ppf->caculatePPF(pointcloud,result,pc);
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_(new pcl::PointCloud<pcl::PointXYZRGBA>);
            mat2Pcl(pc,pcl_);
//            Eigen::Affine3f pose;
//            mat2eigen(result->pose,pose);
//            viewer->displayPCLScene(pcl_,"result",pose);

            Matx44d test{0,-1,0,0,
                         0,0,-1,0.1,
                         1,0,0,0,
                         0,0,0,1};

            Eigen::Affine3f pose;
            mat2eigen(result->pose*test,pose);
            viewer->displayPCLScene(pcl_,"result",pose);

            Matx44d robot_pos = calib_pose*result->pose*test;
            cout << "pose: " << robot_pos << std::endl;
            Mat as(robot_pos);
            Mat rot;
            as.colRange(0,3).rowRange(0,3).convertTo(rot,CV_32F);
            cout << "translation: " << as.rowRange(0,3).col(3)*1000 << std::endl;;
            cout << " rotation: " << Convert::rotationMatrixToEulerAngles(rot)*180/M_PI << std::endl;
            Vec3f rotation = Convert::rotationMatrixToEulerAngles(rot)*180/M_PI;

            motoudp->position[0] = round(as.at<double>(0,3)*1000000);
            motoudp->position[1] = round(as.at<double>(1,3)*1000000);
            motoudp->position[2] = round(as.at<double>(2,3)*1000000);
            motoudp->position[3] = round(rotation[0]*10000);
            motoudp->position[4] = round(rotation[1]*10000);
            motoudp->position[5] = round(rotation[2]*10000);
            motoudp->position[6] = round(as.at<double>(0,3)*1000000);
            motoudp->position[7] = round(as.at<double>(1,3)*1000000);
            motoudp->position[8] = round(as.at<double>(2,3)*1000000);
            motoudp->position[9] = round(rotation[0]*10000);
            motoudp->position[10] = round(rotation[1]*10000);
            motoudp->position[11] = round(rotation[2]*10000);
            motoudp->triggerWritePositions = true;



        }
        else{
            std::cout << "Not Ready" << std::endl;
        }
        isTrigger = false;
        ui->radioButtonTrigger->setChecked(false);
    }

}

void MainWindow::addCoordinate(Eigen::Affine3f aswer)
{
    viewer_->displayCoordiante(aswer);
}


void MainWindow::on_pushButton_3_clicked()
{
    std::cout << "Result " << servo->WriteServo(ui->spinBox->value()) << std::endl;
}

void MainWindow::on_pushButtonSendPosition_clicked()
{
    calib->startSendPosition();
}

void MainWindow::on_pushButtonGetPose_clicked()
{
    calib->startTrigger();
}

void MainWindow::on_pushButtonCaculate_clicked()
{
    calib->caculatePose();
}

void MainWindow::ReadSettings() {
  // Read Settings
    std::cout << "Read Settings" << std::endl;
    QSettings settings("setting", "ros_6d_pose_estimation");
    robotIP = settings.value("robotIP",QString("192.168.1.11")).toString();
    ui->lineEditRobotIP->setText(robotIP);
    gripperAddress = settings.value("gripperAddress",QString("/dev/ttyUSB0")).toString();
    ui->lineEditGripperAddress->setText(gripperAddress);
    firstPV = settings.value("firstPV",40).toInt();
    ui->spinBoxFirstPV->setValue(firstPV);
    firstBV = settings.value("firstBV",40).toInt();
    ui->spinBoxFirstBV->setValue(firstBV);
    calibFile = settings.value("calibFile",QDir::currentPath()).toString();
    ui->lineEditCalibrationFile->setText(calibFile);
    camFile = settings.value("camFile",QString("Choose Path")).toString();
    ui->lineEditCameraParam->setText(camFile);
    modelPath = settings.value("modelPath",QString("Choose Path")).toString();
    ui->lineEditModelPath->setText(modelPath);
    jobName = settings.value("jobName",QString("Choose Path")).toString();
    ui->lineEditJobName->setText(jobName);
    Convert::loadMatFile(calib_pose,calibFile.toStdString());
    //std::cout << calib_pose << std::endl;
}

void MainWindow::WriteSettings() {
    //Write Setting
    std::cout << "Write Settings" << std::endl;
    QSettings settings("setting", "ros_6d_pose_estimation");
    settings.setValue("robotIP",ui->lineEditRobotIP->text());
    settings.setValue("gripperAddress",ui->lineEditGripperAddress->text());
    settings.setValue("firstPV",QVariant(ui->spinBoxFirstPV->value()));
    settings.setValue("firstBV", QVariant(ui->spinBoxFirstBV->value()));
    settings.setValue("calibFile", ui->lineEditCalibrationFile->text());
    settings.setValue("camFile",ui->lineEditCameraParam->text());
    settings.setValue("modelPath",ui->lineEditModelPath->text());
    settings.setValue("jobName",ui->lineEditJobName->text());
}

void MainWindow::readyCameraStatus()
{
    if(camera->camera_running){
        ui->radioButtonRunning->setChecked(true);
        isCameraRunning = true;
    }
    if(camera->camera_running&&ppf->isComplete){
        ui->radioButtonReady->setChecked(true);
        isReady = true;
    }
}

void MainWindow::readyPPFStatus()
{
    std::cout << "PPF Ready" << std::endl;
    if(camera->camera_running&&ppf->isComplete){
        ui->radioButtonReady->setChecked(true);
        isReady = true;
    }
}

void MainWindow::triggerByRobot()
{
    isTrigger = true;
}


void MainWindow::on_toolButtonCalib_clicked()
{
    QFileDialog dialog;
    dialog.setFileMode(QFileDialog::DirectoryOnly);
    dialog.setOption(QFileDialog::ShowDirsOnly, false);
    if(dialog.exec()){
        QString fileName = dialog.directory().path();
        if(!fileName.isNull()&&!fileName.isEmpty()){
             ui->lineEditCalibrationFile->setText(fileName);
        }
    }

}

void MainWindow::on_toolButtonCalibCameraParam_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Chose File"),
                                                    ui->lineEditCameraParam->text(),
                                                    tr("JSON (*.json)"));

    if(!fileName.isNull()&&!fileName.isEmpty()){
         ui->lineEditCameraParam->setText(fileName);
    }
}

void MainWindow::on_toolButtonModelPath_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Chose File"),
                                                    ui->lineEditModelPath->text(),
                                                    tr("PLY (*.ply)"));

    if(!fileName.isNull()&&!fileName.isEmpty()){
         ui->lineEditModelPath->setText(fileName);
         pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_(new pcl::PointCloud<pcl::PointXYZRGBA>);
         cv::Mat input = ppf_match_3d::loadPLYSimple(fileName.toStdString().c_str(),0);
         mat2Pcl(input,pcl_);
         viewer_->displayPCLModel(pcl_,ui->lineEditModelPath->text().toStdString());
    }
}

void MainWindow::on_pushButtonStartCalib_clicked()
{
    isRuntime=false;
    camera->stop();
    delete camera;
    calib = new HandEyeCalibration(this->ui->lineEditRobotIP->text(),this->ui->lineEditCameraParam->text().toStdString());
    connect(calib, &HandEyeCalibration::finishCalibrate, this, &MainWindow::addCoordinate);
    //Realrun
    calib->start();
    //Just Test
    //calib->test();
}

void MainWindow::on_pushButtonStartSave_clicked()
{
    isRuntime = true;
    calib->stop();
    Convert::saveMatFile(calib->calib_pose,ui->lineEditCalibrationFile->text().toStdString());
    delete calib;

    camera = new Camera(848, 480, 848, 480, 30,ui->lineEditCameraParam->text().toStdString());
    connect(camera, &Camera::framesReady, this, &MainWindow::receiveFrame);
    connect(camera, &Camera::pclReady, this, &MainWindow::receivePcl);
    connect(camera, &Camera::pointCloudReady, this, &MainWindow::receivePointCloud);
    camera->start();
}

void MainWindow::on_verticalSlider_valueChanged(int value)
{

}



void MainWindow::on_radioButtonTrigger_toggled(bool checked)
{
    isTrigger=checked;
}

void MainWindow::on_pushButtonSaveImage_clicked()
{
    camera->capture = true;
}

void MainWindow::on_pushButtonStart_clicked()
{
    motoudp->triggerStartJob = true;
}

void MainWindow::on_pushButtonServoOn_clicked()
{
    if(ui->pushButtonServoOn->text()=="Servo On"){
        motoudp->triggerTurnOnServo = true;
        ui->pushButtonServoOn->setText("Servo Off");
    }
    else{
        motoudp->triggerTurnOffServo = true;
        ui->pushButtonServoOn->setText("Servo On");
    }
}

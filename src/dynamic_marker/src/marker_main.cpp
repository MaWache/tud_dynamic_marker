#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <fstream>//only needed for catkin_make

#include "marker_generator.h"


//parameters that should be changed by the user depending on the camera and screen specifications
double screensize_x=0.122;//in meter, x has to be the smaller value!
double screensize_y=0.217;
int screenresolution_x=1440;//in fact it is 1600, but the image display program can only display 16:9;
int screenresolution_y =2560;
int cameraresolution_u=1288;
int cameraresolution_v=964;
std::string pathtofolder = "/home/mw/Pictures/marker_images";
//TODO: initialize cameraMatrix in initialize_values()
cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);


//parameters that can be changed by the developer to improve the result.
int minimum_contour_size = 20;
uchar difference_threshold = 30; // Shadows should not be detected, but the marker should be detected with certainty.
int ellipse_threshold=100;
int minimum_pixels_recognize_marker=30*30;//make this lower to make the marker more sensitive.
int maximum_waiting_rounds=200;
int minimum_center_distance=10;//in pixel. centers on the markers have to be seperated.
double tolerance_relative = 0.1;
double tolerance_abs = 100;
bool equalization =true;//in darker environments or if the marker is far away use equalization


//parameters that need not be changed:
uchar white = 255;
bool waitingForCircles=false;
int waiting_rounds=0;
cv::Mat roi;//region of interest
std::vector<cv::Point3f> centers_in_marker_frame;
std::vector<cv::Point2f> centers_img_plane;
cv::Mat distCoeffs(4,1,cv::DataType<double>::type);//consists of zeros if a rectified image is used.


void initialize_values(){

    distCoeffs.at<double>(0) = 0;
    distCoeffs.at<double>(1) = 0;
    distCoeffs.at<double>(2) = 0;
    distCoeffs.at<double>(3) = 0;

    cameraMatrix.at<double>(0,0)= 510.352814;
    cameraMatrix.at<double>(0,1)=0;
    cameraMatrix.at<double>(0,2)=626.482638;//Data from yaml file

    cameraMatrix.at<double>(1,0)=0;
    cameraMatrix.at<double>(1,1)=615.608826;
    cameraMatrix.at<double>(1,2)=475.770588;

    cameraMatrix.at<double>(2,0)=0;
    cameraMatrix.at<double>(2,1)=0;
    cameraMatrix.at<double>(2,2)=1;


    double x1=0.25*screensize_x;
    double y1=0.25*screensize_y;

    centers_in_marker_frame.push_back(cv::Point3d(0,0,0));// middle point (in meter)
    centers_in_marker_frame.push_back(cv::Point3d(-x1,-y1,0));//upper left
    centers_in_marker_frame.push_back(cv::Point3d(-x1,y1,0));//upper right
    centers_in_marker_frame.push_back(cv::Point3d(x1,-y1,0));//...
    centers_in_marker_frame.push_back(cv::Point3d(x1,y1,0));

}
void getEulerAngles(cv::Mat &rotCamerMatrix,cv::Vec3d &eulerAngles){

    cv::Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;
    double* _r = rotCamerMatrix.ptr<double>();
    double projMatrix[12] = {_r[0],_r[1],_r[2],0,
                             _r[3],_r[4],_r[5],0,
                             _r[6],_r[7],_r[8],0};

    cv::decomposeProjectionMatrix(cv::Mat(3,4,CV_64FC1,projMatrix),
                                  cameraMatrix,
                                  rotMatrix,
                                  transVect,
                                  rotMatrixX,
                                  rotMatrixY,
                                  rotMatrixZ,
                                  eulerAngles);
}
cv::Point getCircleCenter(cv::Mat img,cv::Mat roi){
    cv::Point center(-1,-1);
    cv::Mat windowed;
    img.copyTo(windowed);
    int roi_area=0;
    for(int x=0;x<img.rows;x++){
        for(int y=0;y<img.cols;y++){
            if(roi.at<uchar>(x,y)==0){
                windowed.at<uchar>(x,y)=0;
            }
            else{
                roi_area++;
            }
        }
    }
    if(equalization){    cv::equalizeHist(windowed,windowed);}//Spread hist to make the contrast higher
    //cv::imshow("windowed",windowed);


    cv::Mat bin_windowed;
    cv::threshold(windowed,bin_windowed,ellipse_threshold,255,CV_8UC1);
    cv::imshow("binwindowed",bin_windowed);

    std::vector<std::vector<cv::Point> > many_contours;
    cv::findContours(bin_windowed,many_contours,cv::RETR_LIST,cv::CHAIN_APPROX_NONE);

    for(int i=0;i<many_contours.size();i++){
        std::vector<cv::Point> contour = many_contours.at(i);
        if(contour.size()>=minimum_contour_size){//if it is not big enough, the contour is discarded directly
            if(cv::contourArea(contour,false)>roi_area/60){
                cv::RotatedRect ellipse = cv::fitEllipse(contour);
                double a=(double) ellipse.size.width;
                double b=(double) ellipse.size.height;
                double ellipseArea = 0.25*3.14159*ellipse.size.height*ellipse.size.width;//standard formula for ellipse area
                double area_ratio_to_ellipse = (double)cv::contourArea(contour,false)/ellipseArea;
                double difference=(double)cv::contourArea(contour,false)-ellipseArea;
                if(difference<0){difference= -difference;}
                if(((1-tolerance_relative)<area_ratio_to_ellipse )&& (area_ratio_to_ellipse<(1+tolerance_relative))
                        &&(-tolerance_abs<difference)&&(difference<tolerance_abs)
                        &&(a/b)>0.2&&(b/a)>0.2
                        ){//test if contour is an ellipse
                    if(centers_img_plane.size()==0){
                        center = (cv::Point) ellipse.center;
                        std::cout<<"center.x = "<<center.x<<", centers.y = "<<center.y<<"centers_img_plane.size() = "<<centers_img_plane.size()<<std::endl;

                    }

                    else{
                        bool thisEllipseWasNotDetectedBefore = true;
                        for(int i=0;i<centers_img_plane.size();i++){
                            int last_x =centers_img_plane.at(i).x;
                            int last_y =centers_img_plane.at(i).y;
                            int x_diff = ellipse.center.x -last_x;
                            int y_diff = ellipse.center.y - last_y;
                            if(x_diff<0){x_diff=-x_diff;}
                            if(y_diff<0){y_diff=-y_diff;}
                            int max=minimum_center_distance;
                            if((x_diff<max)&& (y_diff<max)){
                                thisEllipseWasNotDetectedBefore=false;// it was detected before.
                            }
                        }
                        if(thisEllipseWasNotDetectedBefore){
                            center = ellipse.center;
                            std::cout<<"center.x = "<<center.x<<", centers.y = "<<center.y<<"centers_img_plane.size() = "<<centers_img_plane.size()<<std::endl;

                        }
                    }
               }
            }
        }

    }

    return center;
}
void showCenters(){
    cv::Mat centerShow;
    roi.copyTo(centerShow);
    //Draw white background
    for(int x=0;x<centerShow.rows;x++){
        for(int y=0;y<centerShow.cols;y++){
            centerShow.at<uchar>(x,y)=255;

        }
    }

    for(int i=0;i<centers_img_plane.size();i++){
        int r=5;
        cv::Size axes(r,r);

        cv::ellipse(centerShow, centers_img_plane.at(i),axes,0.0,0.0,360.0,0, -1, 8, 0);//full, black circle without shift.

    }
    std::string filepathname = pathtofolder +"/foundCenters.png";
    cv::imwrite(filepathname,centerShow);
    std::cout<<"Writing centers to "<<filepathname<<std::endl;

}

void img_callback(const sensor_msgs::ImageConstPtr &msg){//Type defines what message from the topic is chosen


    cv_bridge::CvImageConstPtr ptr;
    ptr=cv_bridge::toCvShare(msg,"mono8" );//blackwhite would be "mono8" "rgb" changes colors "bgr8" makes it stay the way it is.
    //msg is ptr to the message /image_raw...type
    cv::Mat img = ptr->image;
    // cv::imshow("img",img);

    static cv::Mat lastImage= img;//if already initialized, this step is skipped.
    cv::Mat diff=img-lastImage;
    int difference_sum=0;//find out how much the image changed
    for(int x=0;x<img.rows;x++){
        for(int y=0;y<img.cols;y++){
            if(diff.at<uchar>(x,y) > difference_threshold){
                difference_sum++;
            }
        }
    }
    bool somethingMoved=difference_sum > minimum_pixels_recognize_marker;
    bool notWaitingForCircles = !waitingForCircles;
    if(notWaitingForCircles && somethingMoved ){
        waitingForCircles = true;
        //binarize diff and put in roi
        cv::threshold(diff,roi,difference_threshold,255,CV_8UC1);
    }

    if(waitingForCircles){
        //cv::imshow("roi",roi);
        //std::cout<<centers.size()<<" ellipses found."<<std::endl;
        cv::Point center = getCircleCenter(img, roi);
        if(center.x==-1){
            //nothing found.
        }
        else{
            centers_img_plane.push_back(center);
        }
    }

    if(centers_img_plane.size()==5){
        std::cout<<"5 ellipses found."<<std::endl;
        showCenters();
        waitingForCircles=false;
        cv::Mat r(3,1,cv::DataType<double>::type);
        cv::Mat t(3,1,cv::DataType<double>::type);
        //std::cout<<"centers.size = "<<centers.size()<<", real_centers.size = "<<real_centers.size()<<std::endl;

        cv::solvePnP(centers_in_marker_frame,centers_img_plane,cameraMatrix,distCoeffs,r,t,false,CV_ITERATIVE);//solvepnp needs cv::points, so that integer values for x_center and y_center are obligatory.
        cv::Vec3d eulerAngles;
        cv::Mat R (3,3,cv::DataType<double>::type) ;
        cv::Rodrigues(r,R);

        std::cout<<"pnp did not fail."<<std::endl;

        //write r and t to file
        std::string s= pathtofolder+"/rtmat.mat";
        std::ofstream outfile(s.c_str());
        outfile <<"# name: rt"<<std::endl
               <<"# type: matrix"<<std::endl
              <<"# rows: "<<3<<std::endl//plus first row
             <<"# columns: "         << 2 <<std::endl;
        for(int i=0;i<3;i++){
           std::cout<<r.at<double>(0,i)<<"    "<<t.at<double>(0,i)<<std::endl;
            outfile<<r.at<double>(0,i)<<"    "<<t.at<double>(0,i)<<std::endl;
        }

        getEulerAngles(R,eulerAngles);
        for(int i=0;i<3;i++){
            std::cout<<"angle "<<i<<" = "<<eulerAngles[i]<<std::endl;
            outfile<<"angle "<<i<<" = "<<eulerAngles[i]<<std::endl;
        }
        outfile.close();
        centers_img_plane.clear();
    }


    waiting_rounds++;
    if(waiting_rounds > maximum_waiting_rounds){
        waiting_rounds =0;
        waitingForCircles=false;
    }
    lastImage =img.clone();
    cv::waitKey(4);



}


int main(int argc, char **argv){
    std::cout<<"dynamic marker main started"<<std::endl;
    initialize_values();
    //ROS_INFO("dynamic marker_main started");
     marker_generator mg;
    mg.generate_markers(pathtofolder, screenresolution_x,screenresolution_y);
    ros::init(argc, argv, "subscribernode");
    ros::NodeHandle nh;

    image_transport::ImageTransport imtrans(nh);
    image_transport::Subscriber sub=imtrans.subscribe("/camera/image_rect", 10, img_callback);

    ros::spin();
    return 0;


}

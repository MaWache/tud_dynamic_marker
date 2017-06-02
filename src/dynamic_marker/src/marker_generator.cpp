#include "marker_generator.h"

marker_generator::marker_generator()
{

}
void marker_generator::generate_markers(std::string pathtofolder, int xresolution, int yresolution){
    cv::Mat markerimg;
    cv::Mat whiteimg;
    markerimg.create(xresolution,yresolution,CV_8UC1);
    whiteimg.create(xresolution,yresolution,CV_8UC1);



    //Draw white background
    for(int x=0;x<xresolution;x++){
        for(int y=0;y<yresolution;y++){
            whiteimg.at<uchar>(x,y)=255;

        }
    }

    //black image
    std::string filepath=pathtofolder+"/dynamic_marker0.png";
    cv::imwrite(filepath,markerimg);

    //white image
    whiteimg.copyTo(markerimg);
    filepath=pathtofolder+"/dynamic_marker1.png";
    cv::imwrite(filepath,markerimg);

    //Draw big circle
    int r=0.45*xresolution;
    int y_center=0.5*xresolution;//for cv::ellipse the axes are swapped.
    int x_center=0.5*yresolution;
    cv::Size axes(r,r);
    cv::Point center(x_center,y_center);
    cv::ellipse(markerimg, center,axes,0.0,0.0,360.0,0, -1, 8, 0);//full, black circle without shift.
    //write to file
    filepath=pathtofolder+"/dynamic_marker2.png";
    cv::imwrite(filepath,markerimg);

    //Draw small circle
    whiteimg.copyTo(markerimg);
    r=0.2*xresolution;
    y_center=0.25*xresolution;
    x_center=0.25*yresolution;
    axes.height=r;
    axes.width=r;
    center.x=x_center;
    center.y=y_center;
    cv::ellipse(markerimg, center,axes,0.0,0.0,360.0,0, -1, 8, 0);
    //write to file
    filepath=pathtofolder+"/dynamic_marker3.png";
    cv::imwrite(filepath,markerimg);


    whiteimg.copyTo(markerimg);
    y_center=0.25*xresolution;
    x_center=0.75*yresolution;
    center.x=x_center;
    center.y=y_center;
    cv::ellipse(markerimg, center,axes,0.0,0.0,360.0,0, -1, 8, 0);
    //write to file
    filepath=pathtofolder+"/dynamic_marker4.png";
    cv::imwrite(filepath,markerimg);

    whiteimg.copyTo(markerimg);
    y_center=0.75*xresolution;
    x_center=0.25*yresolution;
    center.x=x_center;
    center.y=y_center;
    cv::ellipse(markerimg, center,axes,0.0,0.0,360.0,0, -1, 8, 0);
    //write to file
    filepath=pathtofolder+"/dynamic_marker5.png";
    cv::imwrite(filepath,markerimg);

    whiteimg.copyTo(markerimg);
    y_center=0.75*xresolution;
    x_center=0.75*yresolution;
    center.x=x_center;
    center.y=y_center;
    cv::ellipse(markerimg, center,axes,0.0,0.0,360.0,0, -1, 8, 0);
    //write to file
    filepath=pathtofolder+"/dynamic_marker6.png";
    cv::imwrite(filepath,markerimg);

    //Now the aruco marker.

    whiteimg.copyTo(markerimg);

    int grid=0.9*xresolution/7;

    int xedge=0.05*xresolution;
    int yedge=0.5*(yresolution-0.95*xresolution);

    //start blackening the most
    for(int i=xedge;i<xedge+7*grid;i++){
        for(int j=yedge;j<yedge+7*grid;j++){
            markerimg.at<uchar>(i,j)=0;


        }

    }
    //white patches
    for(int i=xedge + 1*grid;i<xedge+3*grid;i++){
        for(int j=yedge+2*grid;j<yedge+3*grid;j++){
            markerimg.at<uchar>(i,j)=255;
        }
    }
    for(int i=xedge + 1*grid;i<xedge+6*grid;i++){
        for(int j=yedge+5*grid;j<yedge+6*grid;j++){
            markerimg.at<uchar>(i,j)=255;
        }
    }
    for(int i=xedge + 3*grid;i<xedge+5*grid;i++){
        for(int j=yedge+1*grid;j<yedge+2*grid;j++){
            markerimg.at<uchar>(i,j)=255;
        }
    }
    for(int i=xedge + 5*grid;i<xedge+6*grid;i++){
        for(int j=yedge+2*grid;j<yedge+3*grid;j++){
            markerimg.at<uchar>(i,j)=255;
        }
    }
    for(int i=xedge + 3*grid;i<xedge+5*grid;i++){
        for(int j=yedge+3*grid;j<yedge+5*grid;j++){
            markerimg.at<uchar>(i,j)=255;
        }
    }

    //write to file
    filepath=pathtofolder+"/aruco_marker.png";
    cv::imwrite(filepath,markerimg);
}

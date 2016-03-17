#include <iostream>
#include <string>
#include <vector>
#include <stdint.h>

#include <viso_mono.h>
#include <png++/png.hpp>

using namespace std;

int main (int argc, char** argv) {

  // we need the path name to 2010_03_09_drive_0019 as input argument
  if (argc<2) {
    cerr << "Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019" << endl;
    return 1;
  }

  // sequence directory
  string dir = argv[1];

  // set most important visual odometry parameters
  // for a full parameter list, look at: viso_stereo.h
  VisualOdometryMono::parameters param;
  
  // calibration parameters for sequence 2010_03_09_drive_0019 
  param.calib.f  = 645.24; // focal length in pixels
  param.calib.cu = 635.96; // principal point (u-coordinate) in pixels
  param.calib.cv = 194.13; // principal point (v-coordinate) in pixels
  
  // init visual odometry
  VisualOdometryMono mono_viso(param);
  
  // current pose (this matrix transforms a point from the current
  // frame's camera coordinates to the first frame's camera coordinates)
  Matrix pose = Matrix::eye(4);
    
  // loop through all frames i=0:372
  for (int32_t i=0; i<373; i++) {

    // input file names
    char base_name[256]; sprintf(base_name,"%010d.png",i);
   
    string folder1 = "/home/pang/dataset/kitti/2011_09_26_drive_0001_sync/image_00/data/" ;
   
    string left_img_file_name  = folder1 + base_name;
    
    
    // catch image read/write errors here
    try {

      // load left and right input image
      png::image< png::gray_pixel > left_img(left_img_file_name);
      

      // image dimensions
      int32_t width  = left_img.get_width();
      int32_t height = left_img.get_height();

      // convert input images to uint8_t buffer
      uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
      
      int32_t k=0;
      for (int32_t v=0; v<height; v++) {
        for (int32_t u=0; u<width; u++) {
          left_img_data[k]  = left_img.get_pixel(u,v);
         
          k++;
        }
      }

      // status
      cout << "Processing: Frame: " << i;
      
      // compute visual odometry
      int32_t dims[] = {width,height,width};
      if (mono_viso.process(left_img_data,dims)) {
      
        // on success, update current pose
        pose = pose * Matrix::inv(mono_viso.getMotion());
      
        // output some statistics
        double num_matches = mono_viso.getNumberOfMatches();
        double num_inliers = mono_viso.getNumberOfInliers();
        cout << ", Matches: " << num_matches;
        cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
        cout << pose << endl << endl;

      } else {
        cout << " ... failed!" << endl;
      }

      // release uint8_t buffers
      free(left_img_data);
     
    // catch image read errors here
    } catch (...) {
      cerr << "ERROR: Couldn't read input files!" << endl;
      return 1;
    }
  }
  
  // output
  cout << "Demo complete! Exiting ..." << endl;

  // exit
  return 0;
}
#ifndef _TEST_APP
#define _TEST_APP

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxOsc.h"

#define HOST "localhost"
#define PORT 12345

class testApp : public ofBaseApp{
public:
    void setup();
    void update();
    void draw();

    void mousePressed(int x, int y, int button);
    void keyPressed  (int key);

    void update_tranlate_matrix(float *s_x,float *s_y,float *d_x,float *d_y);

    bool look_for_leds(unsigned char *data);

    ofVideoGrabber movie;

    ofxCvColorImage rgb,hsb;
    ofxCvGrayscaleImage hue,sat,bri,filtered;
    ofxCvContourFinder contours;

    int w,h;
    int findHue;

    std::vector<int> path_x;
    std::vector<int> path_y;
    std::vector<int> vel_x;
    std::vector<int> vel_y;
    std::vector<int> accel_x;
    std::vector<int> accel_y;

    std::vector<int> corrected_path_x;
    std::vector<int> corrected_path_y;
    std::vector<int> corrected_vel_x;
    std::vector<int> corrected_vel_y;
    std::vector<int> corrected_accel_x;
    std::vector<int> corrected_accel_y;


    float camera_corner_x[4];
    float camera_corner_y[4];
    float screen_corner_x[4];
    float screen_corner_y[4];

    int miss_count;

    ofxOscSender sender;

    float p_trans[8];

    ofSerial mySerial;

    int led_brightness;

    bool calibration_in_progress;

    int calibration_phase;

    bool found_4_leds;


};

#endif

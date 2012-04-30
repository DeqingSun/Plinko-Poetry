#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup() {
    ofSetFrameRate(30);
    ofBackground(0,0,0);

    w = 320;
    h = 240;

    movie.initGrabber(w, h, true);




    //reserve memory for cv images
    rgb.allocate(w, h);
    hsb.allocate(w, h);
    hue.allocate(w, h);
    sat.allocate(w, h);
    bri.allocate(w, h);
    filtered.allocate(w, h);

    findHue=165;
    miss_count=0;

    // open an outgoing connection to HOST:PORT
	sender.setup( HOST, PORT );
	camera_corner_x[0]=0;
	camera_corner_x[1]=320;
	camera_corner_x[2]=320;
	camera_corner_x[3]=0;
	camera_corner_y[0]=0;
	camera_corner_y[1]=0;
	camera_corner_y[2]=240;
	camera_corner_y[3]=240;
	screen_corner_x[0]=0;
	screen_corner_x[1]=1200;
	screen_corner_x[2]=1200;
	screen_corner_x[3]=0;
	screen_corner_y[0]=0+40;
	screen_corner_y[1]=0+40;
	screen_corner_y[2]=1920+80;
	screen_corner_y[3]=1920+80;



//manually measured
/*    camera_corner_x[0]=317;
	camera_corner_x[1]=59;
	camera_corner_x[2]=92;
	camera_corner_x[3]=261;
	camera_corner_y[0]=7;
	camera_corner_y[1]=3;
	camera_corner_y[2]=235;
	camera_corner_y[3]=233;
	screen_corner_x[0]=0;
	screen_corner_x[1]=1200;
	screen_corner_x[2]=1200;
	screen_corner_x[3]=0;
	screen_corner_y[0]=0;
	screen_corner_y[1]=0;
	screen_corner_y[2]=1674;
	screen_corner_y[3]=1674;*/


    mySerial.setup("/dev/tty.usbmodem3d11", 38400);
    unsigned char command[]="KONFIGURATION_1*";
    mySerial.writeBytes(command,16);

    led_brightness=0;
}

//--------------------------------------------------------------
void testApp::update(){

    movie.update();

    if (movie.isFrameNew()) {

        //copy webcam pixels to rgb image
        rgb.setFromPixels(movie.getPixels(), w, h);

        //mirror horizontal
        rgb.mirror(false, true);

        //duplicate rgb
        hsb = rgb;

        //convert to hsb
        hsb.convertRgbToHsv();

        //store the three channels as grayscale images
        hsb.convertToGrayscalePlanarImages(hue, sat, bri);

        unsigned char* hue_ptr=hue.getPixels();
        unsigned char* bri_ptr=bri.getPixels();
        unsigned char* sat_ptr=sat.getPixels();
        unsigned char* filtered_ptr=filtered.getPixels();
        
        if (calibration_in_progress){
            unsigned char serial_command[16];
            bool byteWasWritten;
            if (calibration_phase<30){
                sprintf ((char *)serial_command, "a0%02x*", calibration_phase);
                mySerial.writeBytes(serial_command,5);
                byteWasWritten = mySerial.writeBytes(serial_command,5);
                if ( !byteWasWritten ) printf("byte was not written to serial port");
                for (int i=0; i<w*h; i++) {
                    if (hue_ptr[i]>=(120-15) && hue_ptr[i]<=(120+15)){
                        if (bri_ptr[i]>100){
                             if (sat_ptr[i]>30){
                                filtered_ptr[i]=255;
                                continue;
                             }
                        }
                    }
                    filtered_ptr[i]=0;
                }
                filtered.flagImageChanged();
                if (!found_4_leds){
                    if (look_for_leds(filtered_ptr)){
                        found_4_leds=true;
                    }
                }

            }else if(calibration_phase<60){
                sprintf ((char *)serial_command, "a0%02x*", 59-calibration_phase);
                mySerial.writeBytes(serial_command,5);
                byteWasWritten = mySerial.writeBytes(serial_command,5);
            }else{
                calibration_in_progress=false;
            }
            calibration_phase++;
        }else{
            //filter image based on the hue value were looking for
            for (int i=0; i<w*h; i++) {
                if (hue_ptr[i]>=(findHue-15) && hue_ptr[i]<=(findHue+15)){
                    if (bri_ptr[i]>30){
                         if (sat_ptr[i]>150){
                            filtered_ptr[i]=255;
                            continue;
                         }
                    }
                }
                filtered_ptr[i]=0;
            }
            filtered.flagImageChanged();

            //run the contour finder on the filtered image to find blobs with a certain hue
            contours.findContours(filtered, 50, 80*80, 1, false);

            if (contours.nBlobs>0){
                update_tranlate_matrix(camera_corner_x,camera_corner_y,screen_corner_x,screen_corner_y);
                int x_s=contours.blobs[0].centroid.x;
                int y_s=contours.blobs[0].centroid.y;
                int x_m=(x_s*p_trans[0]+y_s*p_trans[1]+p_trans[2])/(x_s*p_trans[6]+y_s*p_trans[7]+1);
                int y_m=(x_s*p_trans[3]+y_s*p_trans[4]+p_trans[5])/(x_s*p_trans[6]+y_s*p_trans[7]+1);

       /*         path_x.push_back (x_s);
                path_y.push_back (y_s);
                corrected_path_x.push_back (x_m);
                corrected_path_y.push_back (y_m);
                miss_count=0;
                int path_size=path_x.size();

                if (path_size>=2){
                    //vel_x.push_back(path_x[path_size-1]-path_x[path_size-2]);
                    //vel_y.push_back(path_y[path_size-1]-path_y[path_size-2]);
                    corrected_vel_x.push_back(corrected_path_x[path_size-1]-corrected_path_x[path_size-2]);
                    corrected_vel_y.push_back(corrected_path_y[path_size-1]-corrected_path_y[path_size-2]);
                    if (path_size>=3){
                        corrected_accel_x.push_back(corrected_vel_x[path_size-2]-corrected_vel_x[path_size-3]);
                        corrected_accel_y.push_back(corrected_vel_y[path_size-2]-corrected_vel_y[path_size-3]);
                    }
                }*/




                ofxOscMessage m;
                m.setAddress( "/plate/position" );
                m.addIntArg( contours.blobs[0].centroid.x );
                m.addIntArg( contours.blobs[0].centroid.y );
                sender.sendMessage( m );

                ofxOscMessage n;
                n.setAddress( "/plate/corrected" );
                n.addIntArg( x_m );
                n.addIntArg( y_m );
                sender.sendMessage( n );
            }else{
                miss_count++;
                if (miss_count==60){
              //      path_x.clear();
               //     path_y.clear();
                    miss_count=0;
                }
            }
        }


    }

    if(mySerial.available() > 0) {
        mySerial.flush(true,false);
    }



}

//--------------------------------------------------------------
void testApp::draw(){
    int i;
    ofSetColor(255,255,255);

    //draw all cv images
    rgb.draw(0,0);
    rgb.draw(320,0);
//    rgb.draw(640,0);
    //hsb.draw(640,0);
//    hue.draw(0,240);
//    sat.draw(320,240);
//    bri.draw(640,240);
//    filtered.draw(0,480);
//    contours.draw(0,480);

    ofSetColor(255, 0, 0);
    ofFill();

    //draw red circles for found blobs
    for (i=0; i<contours.nBlobs; i++) {
        ofCircle(contours.blobs[i].centroid.x, contours.blobs[i].centroid.y, 20);
    }

    for (i=1;i<corrected_accel_x.size();i++){

        float color = ofMap(ofDist(corrected_accel_x[i-1], corrected_accel_y[i-1], corrected_accel_x[i], corrected_accel_y[i]),0,5,0,200);

        ofSetColor(200-color, color, 0);
        ofLine(path_x[i-1], path_y[i-1], path_x[i], path_y[i]);
    }

    //draw camera_corners
    ofSetColor(0,0,255);
    for (i=0;i<4;i++){
        ofLine(camera_corner_x[i], camera_corner_y[i], camera_corner_x[(i+1)&3], camera_corner_y[(i+1)&3]);
    }
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button) {

    //calculate local mouse x,y in image
    int mx = x % w;
    int my = y % h;

    //get hue value on mouse position
  //  findHue = hue.getPixels()[my*w+mx];

    printf("Hue: %d Sat: %d Bri :%d \t",hue.getPixels()[my*w+mx],sat.getPixels()[my*w+mx],bri.getPixels()[my*w+mx]);
    printf("x %d y: %d \n",x,y);

}

//--------------------------------------------------------------
void testApp::keyPressed  (int key){
    bool byteWasWritten ;
    unsigned char serial_command[16]="a000*";

	switch (key){
		case 's':
			movie.videoSettings();
			break;
        case 'c':
			path_x.clear();
            path_y.clear();
            vel_x.clear();
			vel_y.clear();
			accel_x.clear();
			accel_y.clear();
            corrected_path_x.clear();
            corrected_path_y.clear();
            corrected_vel_x.clear();
			corrected_vel_y.clear();
			corrected_accel_x.clear();
			corrected_accel_y.clear();

			break;
        case '=':
            led_brightness+=5;
            if (led_brightness>255) led_brightness=255;
            sprintf ((char *)serial_command, "a0%02x*", led_brightness);
            byteWasWritten = mySerial.writeBytes(serial_command,5);
            if ( !byteWasWritten ) printf("byte was not written to serial port");
            break;
        case '-':
            led_brightness-=5;
            if (led_brightness<0) led_brightness=0;
            sprintf ((char *)serial_command, "a0%02x*", led_brightness);
            byteWasWritten = mySerial.writeBytes(serial_command,5);
            if ( !byteWasWritten ) printf("byte was not written to serial port");
            break;
        case ' ':
            calibration_in_progress=!calibration_in_progress;
            found_4_leds=false;
            calibration_phase=0;
            break;
	}

}

void testApp::update_tranlate_matrix(float *s_x,float *s_y,float *d_x,float *d_y){
    int i;
    float a[64]={0};
    for (i=0;i<4;i++){
        a[i*8+0]=-s_x[i];
        a[i*8+1]=-s_y[i];
        a[i*8+2]=-1;
        a[i*8+6]=s_x[i]*d_x[i];
        a[i*8+7]=s_y[i]*d_x[i];
    }
    for (i=4;i<8;i++){
        a[i*8+3]=-s_x[i-4];
        a[i*8+4]=-s_y[i-4];
        a[i*8+5]=-1;
        a[i*8+6]=s_x[i-4]*d_y[i-4];
        a[i*8+7]=s_y[i-4]*d_y[i-4];
    }

    cv::Mat m1=cv::Mat(8,8,CV_32FC1,a);
    float b[8];
    for (i=0;i<4;i++) b[i]=-d_x[i];
    for (i=0;i<4;i++) b[i+4]=-d_y[i];
    cv::Mat m3=cv::Mat(8,1,CV_32FC1,b);
    cv::Mat m2=(m1.inv())*m3;

    for (i=0;i<8;i++) p_trans[i]=m2.at<float>(i,0);
}

bool testApp::look_for_leds(unsigned char *data){
    int i,j,i1,j1,confidence,confidence_temp;
    int x0,y0,x1,y1,x2,y2,x3,y3;
    confidence=0;
    for (j=0+1;j<120-1;j++){
        for (i=0+1;i<160-1;i++){
            if (data[j*320+i]==255){
                confidence_temp=0;
                for (j1=j-1;j1<=j+1;j1++){
                    for (i1=i-1;i1<=i+1;i1++){
                        if (data[j1*320+i1]==255){
                            confidence_temp+=(2-abs(j-j1)-abs(i-i1));
                        }
                    }
                }
                if (confidence_temp>confidence){
                    confidence=confidence_temp;
                    x0=i;
                    y0=j;
                }
            }
        }
    }
    if (confidence==0) return false;
    confidence=0;
    for (j=120+1;j<240-1;j++){
        for (i=0+1;i<160-1;i++){
            if (data[j*320+i]==255){
                confidence_temp=0;
                for (j1=j-1;j1<=j+1;j1++){
                    for (i1=i-1;i1<=i+1;i1++){
                        if (data[j1*320+i1]==255){
                            confidence_temp+=(2-abs(j-j1)-abs(i-i1));
                        }
                    }
                }
                if (confidence_temp>confidence){
                    confidence=confidence_temp;
                    x1=i;
                    y1=j;
                }
            }
        }
    }
    if (confidence==0) return false;
    confidence=0;
    for (j=120+1;j<240-1;j++){
        for (i=160+1;i<320-1;i++){
            if (data[j*320+i]==255){
                confidence_temp=0;
                for (j1=j-1;j1<=j+1;j1++){
                    for (i1=i-1;i1<=i+1;i1++){
                        if (data[j1*320+i1]==255){
                            confidence_temp+=(2-abs(j-j1)-abs(i-i1));
                        }
                    }
                }
                if (confidence_temp>confidence){
                    confidence=confidence_temp;
                    x2=i;
                    y2=j;
                }
            }
        }
    }
    if (confidence==0) return false;
    confidence=0;
    for (j=0+1;j<120-1;j++){
        for (i=160+1;i<320-1;i++){
            if (data[j*320+i]==255){
                confidence_temp=0;
                for (j1=j-1;j1<=j+1;j1++){
                    for (i1=i-1;i1<=i+1;i1++){
                        if (data[j1*320+i1]==255){
                            confidence_temp+=(2-abs(j-j1)-abs(i-i1));
                        }
                    }
                }
                if (confidence_temp>confidence){
                    confidence=confidence_temp;
                    x3=i;
                    y3=j;
                }
            }
        }
    }
    if (confidence==0) return false;

    camera_corner_x[0]=x0;
    camera_corner_y[0]=y0;
    camera_corner_x[1]=x1;
    camera_corner_y[1]=y1;
    camera_corner_x[2]=x2;
    camera_corner_y[2]=y2;
    camera_corner_x[3]=x3;
    camera_corner_y[3]=y3;
    return true;

}

#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <ctime>
#include <vector>
#include <string>
#include <set>
//#include <opencv2\highgui.h>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2\cv.h>
#include <opencv2/opencv.hpp>

#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

// #define SOCKET_ENABLE
// #define USE_CAPTURE_DEV

using namespace std;

//initial min and max HSV filter values.
//these will be changed using trackbars
int H_MIN[2] = {0, 0};
int H_MAX[2] = {180, 180};
int S_MIN[2] = {0, 0};
int S_MAX[2] = {255, 255};
int V_MIN[2] = {0, 0};
int V_MAX[2] = {255, 255};
//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
//max number of objects to be detected in frame
const int MAX_NUM_OBJECTS = 15;
//minimum and maximum object area
const int MIN_OBJECT_AREA = 3 * 3;
const int MAX_OBJECT_AREA = 200 * 200;
//names that will appear at the top of each window

class Socket
{
private:
    int sockfd, portno;
    struct sockaddr_in serv_addr;
    struct hostent *server;

public:
    Socket()
    {
        sockfd = -1;
    }

    Socket(const char *hostname, const int &port)
    {
        connect(hostname, port);
    }
    
    Socket(const string &hostname, const int &port)
    {
        connect(hostname, port);
    }

    ~Socket()
    {
        if(sockfd < 0)
            return;
        if(close(sockfd) < 0)
        {
            ::perror("ERROR closing");
            ::exit(-1);
        }
    }
    
    void connect(const char *hostname, const int &port)
    {
        portno = port;
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0) 
        {
            ::perror("ERROR opening socket");
            ::exit(-1);
        }

        if ((server = gethostbyname(hostname)) == NULL)
        {
            fprintf(stderr, "ERROR, no such host");
            ::exit(-1);
        }

        bzero((char*) &serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        bcopy((char*) server->h_addr, (char*) &serv_addr.sin_addr.s_addr, server->h_length);
        serv_addr.sin_port = htons(portno);

        if (::connect(sockfd,(struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) 
        {
            ::perror("ERROR connecting");
            ::exit(-1);
        }
    }
    
    void connect(const string &hostname, const int &port)
    {
        connect(hostname.c_str(), port);
    }

    int write(char *buffer, int sz)
    {
        int n;

        if ((n = ::write(sockfd, buffer, sz)) < 0)
        {
            ::perror("ERROR writing to socket");
            ::exit(-1);
        }

        return n;
    }
    
    int read(char *buffer, int sz)
    {
        int n;

        if ((n = ::read(sockfd, buffer, sz)) < 0)
        {
            ::perror("ERROR writing to socket");
            ::exit(-1);
        }

        return n;
    }
    
};

class Player
{
public:
    cv::Point A, B;
    double areaA, areaB;

    Player()
    {
        A = B = cv::Point(0, 0);
    }
};

using namespace cv;

void on_mouse(int e, int x, int y, int d, void *ptr)
{
    if (e == EVENT_LBUTTONDOWN)
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
}

void on_trackbar(int, void*)
{//This function gets called whenever a
 // trackbar position is changed
}

string intToString(int number)
{
    std::stringstream ss;
    ss << number;
    return ss.str();
}

void createTrackbars()
{
    //create window for trackbars

    cv::namedWindow("P1 trackbar", 0);
    cv::namedWindow("P2 trackbar", 0);
    //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
    //the max value the trackbar can move (eg. H_HIGH),
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    //                                  ---->    ---->     ---->
    cv::createTrackbar("H_MIN", "P1 trackbar", &H_MIN[0], H_MAX[0], on_trackbar);
    cv::createTrackbar("H_MAX", "P1 trackbar", &H_MAX[0], H_MAX[0], on_trackbar);
    cv::createTrackbar("S_MIN", "P1 trackbar", &S_MIN[0], S_MAX[0], on_trackbar);
    cv::createTrackbar("S_MAX", "P1 trackbar", &S_MAX[0], S_MAX[0], on_trackbar);
    cv::createTrackbar("V_MIN", "P1 trackbar", &V_MIN[0], V_MAX[0], on_trackbar);
    cv::createTrackbar("V_MAX", "P1 trackbar", &V_MAX[0], V_MAX[0], on_trackbar);

    cv::createTrackbar("H_MIN", "P2 trackbar", &H_MIN[1], H_MAX[1], on_trackbar);
    cv::createTrackbar("H_MAX", "P2 trackbar", &H_MAX[1], H_MAX[1], on_trackbar);
    cv::createTrackbar("S_MIN", "P2 trackbar", &S_MIN[1], S_MAX[1], on_trackbar);
    cv::createTrackbar("S_MAX", "P2 trackbar", &S_MAX[1], S_MAX[1], on_trackbar);
    cv::createTrackbar("V_MIN", "P2 trackbar", &V_MIN[1], V_MAX[1], on_trackbar);
    cv::createTrackbar("V_MAX", "P2 trackbar", &V_MAX[1], V_MAX[1], on_trackbar);
}

void drawObject(int x, int y, cv::Mat &frame, Scalar color)
{

    //use some of the openCV drawing functions to draw crosshairs
    //on your tracked image!

    //UPDATE:JUNE 18TH, 2013
    //added 'if' and 'else' statements to prevent
    //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

    int range = 5;
//     std::cout << x << " " << y << std::endl;
    circle(frame, Point(x, y), range, color, 2);
//     if (y - range > 0)
//         line(frame, Point(x, y), Point(x, y - range), color, 2);
//     else
//         line(frame, Point(x, y), Point(x, 0), color, 2);
//     if (y + range < FRAME_HEIGHT)
//         line(frame, Point(x, y), Point(x, y + range), color, 2);
//     else
//         line(frame, Point(x, y), Point(x, FRAME_HEIGHT), color, 2);
//     if (x - range > 0)
//         line(frame, Point(x, y), Point(x - range, y), color, 2);
//     else
//         line(frame, Point(x, y), Point(0, y), color, 2);
//     if (x + range < FRAME_WIDTH)
//         line(frame, Point(x, y), Point(x + range, y), color, 2);
//     else
//         line(frame, Point(x, y), Point(FRAME_WIDTH, y), color, 2);

    putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, color, 2);
    //cout << "x,y: " << x << ", " << y;
}

void morphOps(cv::Mat &thresh)
{
    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle

    cv::Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
    //dilate with larger element so make sure object is nicely visible
    cv::Mat dilateElement = getStructuringElement(MORPH_RECT, Size(5, 5));

    erode(thresh, thresh, erodeElement);
    erode(thresh, thresh, erodeElement);

    dilate(thresh, thresh, dilateElement);
    dilate(thresh, thresh, dilateElement);
}

void trackFilteredObject(cv::Mat threshold, cv::Mat &cameraFeed, Scalar color, Player &player)
{
    int x, y;
    cv::Mat temp;
    threshold.copyTo(temp);
    //these two vectors needed for output of findContours
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    //use moments method to find our filtered object
    double refArea = 0;
    bool objectFound = false;

    if (hierarchy.size() == 0)
        return;

    int numObjects = hierarchy.size();

    if (numObjects > MAX_NUM_OBJECTS)
    {
        putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, color, 2);
        return;
    }

    for (int index = 0; index >= 0; index = hierarchy[index][0])
    {
        Moments moment = moments((cv::Mat) contours[index]);
        double area = moment.m00;

        if (area >= MIN_OBJECT_AREA && area <= MAX_OBJECT_AREA)
        {
            x = moment.m10 / area;
            y = moment.m01 / area;
            objectFound = true;
        }
        else
            objectFound = false;
        //let user know you found an object
        if (objectFound == true)
        {
            putText(cameraFeed, "Tracking Object", Point(0, 50), 2, 1, color, 2);
            drawObject(x, y, cameraFeed, color);
            
            if(player.A.x == 0 && player.A.y == 0)
            {
                player.A = Point(x, y);
                player.areaA = area;
            }

            player.B = Point(x, y);
            player.areaB = area;
            if(player.A == player.B)
                continue;
        }
    }

    if(player.areaA > player.areaB)
    {
        swap(player.A, player.B);
        swap(player.areaA, player.areaB);
    }

//     std::cout << "Points stored: " << player.A << " " << player.B << "\n";
}

double dist(const cv::Point &p1, const cv::Point &p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + 
           (p1.y - p2.y) * (p1.y - p2.y);
}

double dotProd2d(const cv::Point &a, const cv::Point &b)
{
    return a.x * b.x + a.y * b.y;
}

double vectorLength(const cv::Point &a)
{
    return sqrt(a.x * a.x + a.y * a.y);
}

//not efficient, but it works
int checkBoundsBot(const Player &p, const std::set<int> pts[])
{
    int val, line;

    val = sqrt(p.areaA) / 1.8;
    for(int i = -1; i <= 1; i++)
    {
        line = p.A.y + i * val;
        if(line < 0 || line >= FRAME_HEIGHT)
            return 1;
        for(int j = -1; j <= 1; j++)
        {
            if(pts[line].find(p.A.x + i * val) != pts[line].end())
                return 1;
        }
    }

    val = sqrt(p.areaB) / 0.8;
    for(int i = -1; i <= 1; i++)
    {
        line = p.B.y + i * val;
        if(line < 0 || line >= FRAME_HEIGHT)
            return 1;
        for(int j = -1; j <= 1; j++)
        {
            if(pts[line].find(p.B.x + i * val) != pts[line].end())
                return 1;
        }
    }

    return 0;
}

void filterOutOfAreaPoints(std::set<int> outOfArea[], const cv::Mat &map)
{
    for(int i = 0; i < map.rows; i++)
    {
        for(int j = 0; j < map.cols; j++)
        {
            if(map.at<int>(i, j) == 0)
                outOfArea[i].insert(j);
            else
                break;
        }
        for(int j = map.cols - 1; j >= 0; j--)
        {
            if(map.at<int>(i, j) == 0)
                outOfArea[i].insert(j);
            else
                break;
        }
    }
}

int main(int argc, char* argv[])
{
    if(argc != 3)
    {
        printf("Usage: %s hostname port\n", argv[0]);
        exit(1);
    }

    bool trackObjects = true;
    bool useMorphOps = true;

    cv::Mat cameraFeed, imgHSV, threshold[2], thresholdArea, thresholdRobots;
    createTrackbars(); //create slider bars for HSV filtering

    thresholdRobots = cv::Mat(FRAME_HEIGHT, 2 * FRAME_WIDTH, CV_8U);

    #ifdef USE_CAPTURE_DEV
    cv::VideoCapture capture;
//     capture.open("rtmp://172.16.254.63/live/live"); // remote webcam
    capture.open(0); // webcam
//     set height and width of capture frame
    capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);
    #endif

    #ifdef SOCKET_ENABLE
    Socket soc;
    soc.connect(argv[1], atoi(argv[2]));
    #endif
    struct timespec start, end;
    int rotationSign = 0, n, initialArea = 0;
    char buff[2]; // buffer for I/O
    std::set<int> outOfArea[FRAME_HEIGHT];

//     sleep(15);
    #ifdef SOCKET_ENABLE
    buff[0] = 's';
    n = soc.write(buff, 1);
    if(n != 1)
    {
        std::cerr << "Socket write movement error!\n";
        exit(-2);
    }
    #endif

    while (1)
    {
        #ifdef SOCKET_ENABLE
        n = soc.read(buff, 1);
        if(n != 1)
        {
            std::cerr << "Socket read byte error!\n";
            exit(-2);
        }
        #endif
        if(rotationSign != 0)
        {
            if(buff[0] == 0)
                rotationSign = 0;
        }

        Player player1, player2;

        clock_gettime(CLOCK_MONOTONIC, &start);
        //store image to matrix
        #ifdef USE_CAPTURE_DEV
        capture.read(cameraFeed);
        #else
        cameraFeed = cv::imread("/home/linux/Desktop/FIC/basic_area_3_robot.png");
        #endif

        //convert frame from BGR to HSV colorspace
        cv::cvtColor(cameraFeed, imgHSV, COLOR_BGR2HSV);
        //filter HSV image between values and store filtered image to
        //threshold matrix
        inRange(imgHSV, 
                cv::Scalar(H_MIN[0], S_MIN[0], V_MIN[0]), 
                cv::Scalar(H_MAX[0], S_MAX[0], V_MAX[0]), 
                threshold[0]);
        inRange(imgHSV, 
                cv::Scalar(H_MIN[1], S_MIN[1], V_MIN[1]), 
                cv::Scalar(H_MAX[1], S_MAX[1], V_MAX[1]), 
                threshold[1]);
        if(!initialArea)
        {
            inRange(imgHSV, 
                    cv::Scalar(0, 0, 0), 
                    cv::Scalar(180, 255, 40), 
                    thresholdArea);
            initialArea = 1;
            filterOutOfAreaPoints(outOfArea, thresholdArea);
        }

        //perform morphological operations on thresholded image to eliminate noise
        //and emphasize the filtered object(s)
        if (useMorphOps)
        {
            morphOps(threshold[0]);
            morphOps(threshold[1]);
        }
        //pass in thresholded frame to our object tracking function
        //this function will return the x and y coordinates of the
        //filtered object
        if (trackObjects)
        {
            trackFilteredObject(threshold[0], cameraFeed, Scalar(0, 255, 0), player1);
            trackFilteredObject(threshold[1], cameraFeed, Scalar(255, 0, 0), player2);
        }

        //show frames
//         cv::imshow("HSV", imgHSV);
        threshold[0].copyTo(thresholdRobots.rowRange(0, FRAME_HEIGHT).colRange(0, FRAME_WIDTH));
        threshold[1].copyTo(thresholdRobots.rowRange(0, FRAME_HEIGHT).colRange(FRAME_WIDTH, 2 * FRAME_WIDTH));
        cv::imshow("Threshold", thresholdRobots);
        cv::imshow("Area", thresholdArea);
        cv::imshow("Original Image", cameraFeed);

//         cv::Point p;
//         cv::setMouseCallback("Original Image", on_mouse, &p);

        cv::Point origin, dirHead, enemy = player2.A;
        if(dist(player1.A, enemy) > dist(player1.B, enemy))
        {
            origin = player1.A;
            dirHead = player1.B;
        }
        else
        {
            origin = player1.B;
            dirHead = player1.A;
        }

        int danger = checkBoundsBot(player1, outOfArea);
        static int dangerTime = 0;
        if(danger > 0)
        {
//             std::cout << "MOVE IT!!!\n";
            dangerTime++;
            if(dangerTime == 8)
            {
//                 std::cout << "I'M OUTTA HERE!!!\n";
                if(dirHead == player1.A) // move back
                {
                    buff[0] = 'b';
                    #ifdef SOCKET_ENABLE
                    n = soc.write(buff, 1);
                    if(n != 1)
                    {
                        std::cerr << "Socket write movement error!\n";
                        exit(-2);
                    }
                    #endif
                }
                else if(dirHead == player1.B)
                {
                    buff[0] = 'f';
                    #ifdef SOCKET_ENABLE
                    n = soc.write(buff, 1);
                    if(n != 1)
                    {
                        std::cerr << "Socket write movement error!\n";
                        exit(-2);
                    }
                    #endif
                }
            }
        }
        else
        {

            cv::Point ourBot, enemyBot;

            ourBot = cv::Point(dirHead.x - origin.x, dirHead.y - origin.y);
            enemyBot = cv::Point(enemy.x - origin.x, enemy.y - origin.y);
            if(ourBot.x * enemyBot.y - ourBot.y * enemyBot.x > 0) // sort of cross product in 2d
                rotationSign = +1; //rotate to right
            else
                rotationSign = -1; //rotate to left
    //         std::cout << rotationSign << "\n";
            double cosine;
            cosine = (dotProd2d(ourBot, enemyBot) / (vectorLength(ourBot) * vectorLength(enemyBot)));
            if(cosine >= 0.9962) // +- ~5 degrees
                rotationSign = 0;

            if(!rotationSign) // move f/b
            {
                if(dirHead == player1.A)
                    buff[0] = 'f';
                else if(dirHead == player1.B)
                    buff[0] = 'b';
                #ifdef SOCKET_ENABLE
                n = soc.write(buff, 1);
                if(n != 1)
                {
                    std::cerr << "Socket write movement error!\n";
                    exit(-2);
                }
                #endif
            }
            else
            {
                double angle = acos(cosine);

                if(rotationSign == -1)
                    buff[0] = 'l';
                else
                    buff[0] = 'r';
                buff[1] = (char) (((int) round(angle)) & 0x7F);
                #ifdef SOCKET_ENABLE
                n = soc.write(buff, 2);
                if(n != 2)
                {
                    std::cerr << "Socket write angle error!\n";
                    exit(-2);
                }
                #endif
            }
        }

        char key;
        key = cv::waitKey(30);
        if(key == 27)
            break;

        clock_gettime(CLOCK_MONOTONIC, &end);
        double millis = (1e3 * (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e6 );
//         std::cout << "FPS: " << 1000.0 / millis << "\n";
        cameraFeed.release();
    }

    return 0;
}

/*
** MR_Final.c
** COMP4510 - Yanco
** 
** DESCRIPTION:
**      Mobile RObotics 2 Final Project
**      W.A.S.T.E
**      Wireless Automatic Supplementary Trash Evicter
**      --------------------------------
**      Robot capable of automatically taking your trash can out to the
**      curb before the trash truck comes to pick it up and bring it
**      back once it has been picked up.
**
** Author:
**      Sam Laderoute
**
** History:
**      19Oct2021   Sam Laderoute   Created Lab4_Vision_p1.c
**      21Oct2021   Sam Laderoute   Added Comments
**      23Oct2021   Sam Laderoute   If color not visible at start, search for it
**      23Oct2021   Sam Laderoute   Converted to Lab4_Vision_p2.c
**                                  Added Gripper Control and implemented part2 of lab
**                                  Drops off picked up color block at similar colored block
**      25Oct2021   Sam Laderoute   Now requires seeing blob for multiple frames before action
**                                  minimizes detection failure
**      04Nov2021   Sam Laderoute   Converted to MR_Final.c
**                                  Renamed Object struct to Blob
**                                  Blob size now based on real Area
**                                  Blob location now based on blob centroid
**                                  Blob now stores confidence level (unused)
**      16Nov2021   Sam Laderoute   Created Loaction struct to hold destinations and obstacles
**                                  Created Robot struct to handle robot pos and internal sensing
**                                  Added functions to move robot precisly through the world
**                                  
**
*/

/*
** NOTES
**
** Link to useful docs:
** https://www.kipr.org/doc/group__motor.html?fbclid=IwAR2uX9eOVENJE8vOZK1iirCafg8F4ECy8SwURf0x6tBJQY4kRpMALidsRIE#gafce5cf37833e487343ece5256fe66d37
**      "There are approximately 1500 ticks per motor revolution"
**
** - use size of blob for logic, check min size?
** - use confidence level, min conf?
**
** Nov 12 2021 - planning strats
** CANT MOVE FORWARD WITH BARREL, so all mvoement while holding barrel must be backwards/ towing
** maybe just always move backwards and turn around to hitch. essentially robot is just backwards now.
**
** 1. use internal map locations to approach barrel location from desired barrel side. (avoiding obstacles) -- barrel orientation is determined by robot, so we can assume we know it already
** 2. turn 180 to prepare for hitch. use rgb camerea to ensure we are aligned facing the barrel (in center). and close enough to it.
** 3. activate manipulator and grab barrel.
** 4. use internal map to approach desired barrel location.
** 5. at drop off spot, drive through the location so we are on other side, turn 180, drive through again, then deactivate manipulator.
**      __OR__  intially approach the drop off from the backside, drive through, then deactivate the manipulator
** 6. return home. using internal map.
**
** We should store the barrels current location as an obstacle in the map.
** do obstacle avoidance
**
**
*/

#include <kipr/wombat.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#define RED 0
#define PURPLE 1
#define WIDTH 159
#define HEIGHT 119
#define PI 3.14159265

/*
** Struct:    Blob
** ----------------------
** Holds a blob
**
** x: screen x coordinate
** y: screen y coordinate
** size: size of blob
** confidence: The confidence, between 0.0 and 1.0, that given object is significant.
*/
struct Blob {
    int x;
    int y;
    int size;
    double confidence;
};

/*
** Struct:    Location
** ----------------------
** Stores location and orientation of obstacles and destinations
**
** x: world x coordinate cm
** y: world y coordinate cm
** theta: world angle of rotation in radians
*/
struct Location {
    double x;
    double y;
    double theta;
};

/*
** Struct:    Robot
** ----------------------
** x: world x coordinate cm
** y: world y coordinate cm
** theta: world angle of rotation in radians
** radius: radius of robot in cm
** obstacles: stores loactions of seen obstacles
*/
struct Robot {
    double x;
    double y;
    double theta;
    double radius;
    struct Location obstacles[10];
};

void print_robot(struct Robot *r) {
    printf("############\n");
    printf("ROBOT STATE\n");
    printf("pos: (%f, %f)\n", r->x, r->y);
    printf("angle: %f (%fdeg)\n", r->theta, 180*r->theta/PI);
    printf("############\n");
}


double rad2deg(double rad) {
	return (rad*180)/PI;
}
double abs_f(double v) {
	if (v < 0){
    	return v*-1.0;
    }
    return v;
}

/*
** Function:    robot_drive
** ----------------------
** moves the robot forward or backward a precise amount
** alters robot's stored position
**
** r: the robot
** speed: speed of wheels
** cm: distance to move in cm (pos or neg)
**
** returns: void
*/
void robot_drive(struct Robot *r, int speed, double cm) {
    // proboably need to do some manipulation on cm to convert from cm's
    // to motor ticks. Probably based on wheel size? and maybe speed.
    // "There are approximately 1500 ticks per motor revolution"
    int N = 2000; // num ticks in 1 full rotation of wheels
    double circumfrence = 22.0; // cm
    int ticks = (int)((N/circumfrence)*cm); // ~ 68ticks per cm
    printf("moving  speed:%d  ticks:%d  cm:%f\n", speed, ticks, cm);
    move_relative_position(0, speed, ticks);
    move_relative_position(1, speed, ticks);
    //speed is ticks/sec
    //msleep((int)(1000*speed/ticks));
	bmd(0);
    bmd(1);
    
    r->x += cm*cos(r->theta);
    r->y += cm*sin(r->theta);
    return;
}

/*
** Function:    robot_turn
** ----------------------
** turns the robot a precise amount
** alters robot's stored theta
** if isRelative is false then it treats the angle in world coordinate frame.
**
** r: the robot
** speed: speed of wheels
** angle: turning angle in radians
** isRelative: if isRealtive then angle is added to current angle, else
**             it is in world coordinate frame
**
** returns: void
*/
void robot_turn(struct Robot *r, int speed, double d_angle, bool isRelative) {
    if (!isRelative) {
        d_angle = (double)((int)(180*d_angle/PI) % 360);
        d_angle *= PI/180;
        if (abs_f(d_angle) > PI) { // take shortest turn
            d_angle += (d_angle > 0) ? -PI : PI;
            d_angle *= -1;
        }
        d_angle -= r->theta;
    }

    // proboably need to do some manipulation on d_angle to convert from radians
    // to motor ticks. probably based on wheel size? maybe speed.
    // "There are approximately 1500 ticks per motor revolution"
    int Ldir = (d_angle < 0) ? 1 : -1;
    int Rdir = -Ldir;
    double N = 2000.0; // N is total num ticks in 1 full rotation of wheels
    //printf("TICK %f\n", 2.0*N*abs(d_angle)/PI);
    int ticks = (int)(2.0*N*abs_f(d_angle)/PI); 
    printf("turning %f  speed:%d  ticks:%d  Ldir:%d  Rdir:%d\n",rad2deg(d_angle), speed, ticks, Ldir, Rdir);
    move_relative_position(0, speed, ticks*Ldir);
    move_relative_position(1, speed, ticks*Rdir);
    
    //speed is ticks/sec
    //msleep((int)(1000*speed/ticks));
    bmd(0);
    bmd(1);

    r->theta += d_angle;
    return;
}

/*
** Function:    compare
** ----------------------
** compares two [Blob]s (blobs) by size
**
** s1: blob1
** s2: blob2
**
** returns: 1 if [s1] is larger, -1 if [s2] is larger, 0 if equal
*/
int compare(const void *s1, const void *s2) {
    struct Blob *b1 = (struct Blob *)s1;
    struct Blob *b2 = (struct Blob *)s2;
    int diff =  b1->size - b2->size;
    if (diff > 1) return -1;
    else if (diff < 1) return 1;
    else return 0;
}

/*
** Function:    print_arr
** ----------------------
** prints out an array of blobs to the terminal
**
** A: the array of blobs to be printed
** len: the length of the array
**
** returns: void
*/
void print_arr(struct Blob *A, int len) {
    int i = 0;
    printf("[");
    while (i < len) {
        printf("(%d, %d, %d), ", A[i].x, A[i].y, A[i].size);
        i++;
    }
    printf("]");
    return;
}

/*
** Function:    group_blobs
** ----------------------
** Takes an array of blobs (x, y, size) and combines any that are close together.
**      If two blobs are within [threshold] pixels of each other they are combined.
**
** arr: the array of blobs (passed by reference)
** len: the length of the array
** threshold: max combine distance in pixels
**
** returns: void
*/
void group_blobs(struct Blob *arr, int len, double threshold) {
    struct Blob dummy;
    dummy.x = -1;
    dummy.y = -1;
    dummy.size = -1;

    // compare all blobs to all other blobs
    int i = len-1;
    while ( i > 0) {
        int j = 0;
        bool breakj = false;
        while (j < len) {
            if (i == j) break;  // ignore self
            // if (min(arr[i].confidence, arr[j].confidence) < 0.6) break; // dont combine if confidence of on is too low
            double dist = sqrt(pow(arr[i].x - arr[j].x, 2) + pow(arr[i].y - arr[j].y, 2));
            if (dist < threshold) {  // if blobs are close enough, combine them
                struct Blob new_b;    // new_b is combined blob
                new_b.x = (arr[i].x + arr[j].x) / 2;
                new_b.y = (arr[i].y + arr[j].y) / 2;
                new_b.size = arr[i].size + arr[j].size;

                arr[j] = new_b;
                arr[i] = dummy; // replace one of the original blobs with dummy (-1, -1, -1)
                breakj = true;
            }
            if (breakj) break;
            j++;
        }
        i--;
    }
}

/*
** Function:    halt
** ----------------------
** stops the robot
**
** returns: void
*/
void halt() {
	motor(0, 0);
    motor(1, 0);
}

/*
** Function:    turn
** ----------------------
** turns the robot in place for [mdur] millisecs at [speed].
**      if [mdur] = -1 then duration is ignored and the robot
**      will continue to spin until otherwise directed
**
** dir: the direction to turn. 1 = left, -1 = right
** mdur: the number of milliseconds to turn for
** speed: the speed at which the wheels turn (in range 0-100)
**
** returns: void
*/
void turn(int dir, int mdur, int speed) {
    motor(0, dir*-speed);
    motor(1, dir*speed);
    if (mdur != -1) {
    	msleep(mdur);
    	halt();
    }
}

/*
** Function:    drive
** ----------------------
** drives the robot forward for [mdur] millisecs at [speed].
**
** mdur: the number of milliseconds to drive for
** speed: the speed at which the wheels turn (range 0-100)
**
** returns: void
*/
void drive(int mdur, int speed){
    motor(0, speed);
    motor(1, speed);
    msleep(mdur);
    halt();
}

/*
** Function:    turn_to_blob
** ----------------------
** turns the robot towards the given blob.
**      goldilocks zone is determined by [goldilocks].
**      ex. [goldilocks] = 0.2, then goldilocks zone = middle 20% of screen.
**      if blob is too far left of goldilocks zone turn left to face.
**      if blob is too far right of goldilocks zone turn right to face.
**      if the blob is within the goldilocks zone - done turning.
**
** b: the blob we are turning towards (x, y, size)
** goldilocks: ratio of screen that accepts that we are facing blob. (range 0.0-1.0)
**
** returns: false if blob is within goldilocks zone. true if still turning.
*/
bool turn_to_blob(struct Blob b, double goldilocks) {
    if (b.x < WIDTH/2 - WIDTH*goldilocks/2) { // blob is too far left
        turn(1, -1, 8); //turn(1, 50, 50);  // turn left
        return true;
    } else if (b.x > WIDTH/2 + WIDTH*goldilocks/2) { // blob is too far right
        turn(-1, -1, 8); //turn(-1, 50, 50);  // turn right
        return true;
    } else {  // within goldilocks zone
        halt();
        return false;
    }
}

/*
** Function:    seek_color
** ----------------------
** Takes an image of the environment. Determines where
**      the blobs of color are. Compiles them into a list.
**      Combines close blobs. Then turns to face the largest,
**      and drives forward until no blobs are found.
**      If no color blob initially seen, searches for color blob.
**      
** channel: int that determines what color channel we are seeking (what color?)_
**
** returns: void when arrived at desired color
*/
void seek_color(int channel){
    // printf("Searching for Color [channel: %d]\n", channel);
    bool color_found = false;
    int color_count = 0; // counts consecutive frames blob appears
    int count = 0; // counts connsecutive frames w/ no blobs
    // Opens the connection to the black camera
    if (camera_open_black() == 0) {
        printf("camera failure!\n");
    }

    while(true) {
        // take snapshot
        count++;
        if (camera_update() == 0) {
        	printf("snapshot failed!");
        }

        // SEARCH FOR PARTICULAR COLOR
        int n_blobs = get_object_count(channel);
        if(n_blobs > 0) {  // does the camera see at least 1 blob?
            count = 0; // rest num of tries we take imgs looking
            if (color_found == false) {
                color_count++;
                if (color_count > 5) {  // if blob exists for more than 5 consecutive frames
                    // printf("Found Color [channel: %d]\n", channel);
                    color_found = true;
                    color_count = 0;
                }
            }
            // build array of Blobs
            int i = 0;
            struct Blob A[n_blobs];
            while (i < n_blobs) {
                struct Blob b;
                b.x = get_object_centroid_x(channel, i);
                b.y = get_object_centroid_y(channel, i);
                b.size = get_object_area(channel, i);
                b.confidence = get_object_confidence(channel, i);
                A[i] = b;
                i++;
            }
            // array built!
            // combine blobs that are within x pixels of each other
            group_blobs(A, n_blobs, 5);
            // sort array: largest to smallest blobs
            qsort(A, n_blobs, sizeof(struct Blob), compare);

            // TRACK ONCE FOUND
            // turn towars given blob So it is at least centered within the given x(0-1) of the screen
            bool turning = true;
            turning = turn_to_blob(A[0], 0.05);

            // APPROACH
            if (!turning)
            	drive(200, 50);
        } else { // no blobs seen!
            color_count = 0;
            if (color_found == false) {
                // if we have not yet seen the color turn a bit to search
                if (count == 5) {
                    turn(-1, 200, 50);
                    msleep(150);
                    count = 0;
                }
            } else { // cant see the color blob
                // make sure blob is really gone for at least 5 frames!
                if (count == 5) {
                    // (must be too close/at the blob) - STOP and return
                    halt();
                    // printf("Arrived at Color [channel: %d]\n", channel);
                    return;
                }
            }
        }
    }

    // Closes the connection to the camera
    camera_close();
}

/*
** Function:    open_gripper
** ----------------------
** Controls servro to open gripper
**
** returns: void
*/
void open_gripper() {
    enable_servo(1);
    set_servo_position(1, 2000);
    msleep(1000);
    disable_servo(1);
}

/*
** Function:    close_gripper
** ----------------------
** Controls servro to close gripper
**
** returns: void
*/
void close_gripper() {
    enable_servo(1);
    set_servo_position(1, 600);
    msleep(1000);
    disable_servo(1);
}

int main(int argc, char* argv[]) {
    printf("Hello World\n");

    int color_seek = RED;
    enable_servo(0);
    set_servo_position(0, 850);

    struct Robot r;
    r.x=0;
    r.y=0;
    r.theta=PI/2; // starts facing 'upwards' in world frame

    
    printf("case1\n");
    robot_turn(&r, 300, PI/2, true); // SHOULD TURN 90deg LEFT
    robot_drive(&r, 300, 10); // SHOULD DRIVE FORWARD 10cm
    print_robot(&r); // SHOULD BE (-10, 0), 3.14 (180deg); (facing 'left')
    msleep(4000);

    printf("case2\n");
    robot_turn(&r, 300, PI/2, false); // SHOULD TURN TO FACE 'upwards' (90deg relative)
    robot_turn(&r, 300, -PI/6, true); // SHOULD TURN 30deg RIGHT (
    robot_drive(&r, 300, 20); // SHOULD DRIVE FORWARDS 20cm
    print_robot(&r); // SHOULD BE (0, 17.3), 1.04 (60deg); (facing /^ this way)
    msleep(4000);

    printf("case3\n");
    robot_turn(&r, 300, 3*PI/2, false); // SHOULD TURN TO FACE 'downwards'
    robot_drive(&r, 300, -10); // SHOULD DRIVE BACKWARDS 10cm
    print_robot(&r); // SHOULD BE (0, 7.3), -0.78 (-90deg); (facing 'down')
    
    
    /*
    // PART 1 LAB 4
    seek_color(color_seek);

    msleep(500);

    // PART 2 LAB 4
    // GRAB BLOCK
    open_gripper();
    drive(1200, 50);
    close_gripper();
    drive(1500, -50);
    // FIND SIMILAR COLOR
    seek_color(color_seek);
    // PLACE TOGETHER
    open_gripper();
    drive(1500, -50);
    close_gripper();
    */

    disable_servo(0);
    printf("Goodbye\n");
    return 0;
}
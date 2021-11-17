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
**                                  Added function to grab trash located in front of robot (iffy)
**      17Nov2021   Sam Laderoute   Now using IR dist sensor instead of blob size b/c blob size freakin sucks
**                                  Removed unused functions, cleaned and commented code tiny bit
**
*/

/*
** NOTES
**
** Link to useful docs:
** https://www.kipr.org/doc/group__motor.html?fbclid=IwAR2uX9eOVENJE8vOZK1iirCafg8F4ECy8SwURf0x6tBJQY4kRpMALidsRIE#gafce5cf37833e487343ece5256fe66d37
**      "There are approximately 1500 ticks per motor revolution" (actually 2k)
**
** WTH. SOMETIMES THE MOTORS JUST DONT ACTIVATE? I DONT GET IT. BOTH MOTOTRS GET THE SAME EXACT INSTRUCTION, BUT ONLY ONE OF THEM MOVES?
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

/*
** Function:    print_robot
** ----------------------
** prints status of given robot
**
** r: the robot
**
** returns: void
*/
void print_robot(struct Robot *r) {
    printf("############\n");
    printf("ROBOT STATE\n");
    printf("pos: (%f, %f)\n", r->x, r->y);
    printf("angle: %f (%fdeg)\n", r->theta, 180*r->theta/PI);
    printf("############\n");
}

/*
** Function:    rad2deg
** ----------------------
** converts radians to degrees
**
** rad: angle in radians
**
** returns: angle in degrees
*/
double rad2deg(double rad) {
	return (rad*180)/PI;
}

/*
** Function:    abs_f
** ----------------------
** absolute value that works with floats
**
** v: value (float)
**
** returns: absolute value of value
*/
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
    // "There are approximately 1500 ticks per motor revolution" -- actually it 2k
    int N = 2000; // num ticks in 1 full rotation of wheels
    double circumfrence = 22.0; // cm
    int ticks = (int)((N/circumfrence)*cm); // ~ 90ticks per cm
    //printf("moving  speed:%d  ticks:%d  cm:%f\n", speed, ticks, cm);
    move_relative_position(0, speed, ticks);
    move_relative_position(1, speed, ticks);
	
    // wait for movement to complete
    while(!get_motor_done(0) || !get_motor_done(1)){
    }

    // update robot pos
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
        d_angle = (double)((int)((180*5*d_angle)/(PI*5)) % 360); // *5 is to increase fidelty of cast
        d_angle *= PI/180;
        if (abs_f(d_angle) > PI) { // take shortest turn
            d_angle += (d_angle > 0) ? -PI : PI;
            d_angle *= -1;
        }
        d_angle -= r->theta;
    }

    // "There are approximately 1500 ticks per motor revolution" -- actually it 2k
    int Ldir = (d_angle < 0) ? 1 : -1;
    int Rdir = -Ldir;
    double N = 2000.0 * 1.35; // N is total num ticks in 1 full rotation of robot ### idk why i need constant 1.35 (i made up)
    int ticks = (int)(N*abs_f(d_angle)/PI);
    //printf("turning %f  speed:%d  ticks:%d  Ldir:%d  Rdir:%d\n",rad2deg(d_angle), speed, ticks, Ldir, Rdir);
    move_relative_position(0, speed, ticks*Ldir);
    move_relative_position(1, speed, ticks*Rdir);
    
    // wait for movement to complete
	while(!get_motor_done(0) || !get_motor_done(1)){
    }

    // update robot angle
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
bool turn_to_blob(struct Robot *r, struct Blob b, double goldilocks) {
    if (b.x < WIDTH/2 - WIDTH*goldilocks/2) { // blob is too far left
        robot_turn(r, 100, 0.03, true); // turn left
        return true;
    } else if (b.x > WIDTH/2 + WIDTH*goldilocks/2) { // blob is too far right
        robot_turn(r, 100, -0.03, true);  // turn right
        return true;
    } else {  // within goldilocks zone
        //halt();
        return false;
    }
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
    set_servo_position(1, 1500);
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
    set_servo_position(1, 1000);
    msleep(1000);
    disable_servo(1);
}

/*
** Function:    approach_color
** ----------------------
** We expect the robot to be looking at this color when the func is called
** The robot then turns to face it and moves closer to it within some thresh
**
** r: the robot we are giving the command to
** channel: the color we are approaching
**
** returns: false if color not found, true once color is approached
*/
bool approach_color(struct Robot *r, int channel) {
    camera_open_black();

    int ir_thresh = 2600;
    bool color_found = false;
    int color_count = 0; // counts consecutive frames blob appears
    int empty_count = 0; // counts connsecutive frames w/ no blobs
    
    bool done = false;
    bool turning = true;
    while(!done) {
        empty_count++;
        if (camera_update() == 0) {
        	printf("snapshot failed!");
            continue;
        }

        // SEARCH FOR PARTICULAR COLOR
        int n_blobs = get_object_count(channel);
        if(n_blobs > 0) {  // does the camera see at least 1 blob?
            empty_count = 0; // reset num of empty frames
            if (color_found == false) { // if we havent seen color for last 5 frames
                color_count++;  // incrase num frames we've consecutively seen this color
                if (color_count > 5) {  // if blob exists for more than 5 consecutive frames
                    color_found = true;
                }
            }
            if (color_found == true) { // we've seen the color for at least 5 frames
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

                // TURN TO FACE THE BLOB
                turning = turn_to_blob(r, A[0], 0.05); // place blob in middle 20% of screen
            }
        } else { // no blobs seen this snapshot!
            color_count = 0; // reset num of frames we've connsecutively seen the color
            if (empty_count > 10) { // haven't seen any sign of color in last 10 frames
                camera_close();
                return false; // DIDNT SEE THE COLOR :(
            }
        }
        if (!turning && analog(0) < ir_thresh) {
        	robot_drive(r, 100, 1);
        }
        if (!turning && analog(0) >= ir_thresh) {
            done = true;
        }
    }
    camera_close();
    return true; // now at the color
}

/*
** Function:    grab_trash
** ----------------------
** We assume robot is facing trash. This has robot align to trash.
** Open gripper, approach, close gripper (grasp) the backup a bit.
**
** r: the robot we are giving command to
**
** returns: void
*/
void grab_trash(struct Robot *r) {
    // Find color of trash can
    // Turn to face trash more precisely
    if (approach_color(r, RED)) {
        open_gripper();
        // approach
        robot_drive(r, 300, 4.5);
        robot_drive(r, 100, 3);
        close_gripper();
        robot_drive(r, 300, -7.5);
    } else {
    	printf("trash not found");
    }
}

int main(int argc, char* argv[]) {
    printf("Hello World\n");
    enable_servo(0);
    set_servo_position(0, 800);

    struct Robot r;
    r.x=0;
    r.y=0;
    r.theta=PI/2; // starts facing 'upwards' in world frame
	
    printf("A: movement test\nB:Gripper test\n");
    bool choosing = true;
    int method = 1;
    while(choosing) {
        if (a_button_clicked()) {
            method = 1;
            choosing = false;
            break;
        }
        if (b_button_clicked()) {
            method = 2;
            choosing = false;
            break;
        }
    }
    switch(method) {
        case 1:
            // MOVEMENT TEST CASE
            printf("case1\n");
            robot_turn(&r, 300, PI/2, true); // SHOULD TURN 90deg LEFT
            robot_drive(&r, 300, 5); // SHOULD DRIVE FORWARD 5cm
            print_robot(&r); // SHOULD BE (-5, 0), 3.14 (180deg); (facing 'left')
            msleep(4000);

            printf("case2\n");
            robot_turn(&r, 300, PI/2, false); // SHOULD TURN TO FACE 'upwards' (90deg relative)
            robot_turn(&r, 300, -PI/6, true); // SHOULD TURN 30deg RIGHT (
            robot_drive(&r, 300, 10); // SHOULD DRIVE FORWARDS 10cm
            print_robot(&r); // SHOULD BE (0, 8.66), 1.04 (60deg); (facing /^ this way)
            msleep(4000);

            printf("case3\n");
            robot_turn(&r, 300, 3*PI/2, false); // SHOULD TURN TO FACE 'downwards'
            robot_turn(&r, 300, PI/2, false); // SHOULD TURN TO FACE 'upwards'
            robot_drive(&r, 300, -10); // SHOULD DRIVE BACKWARDS 10cm
            print_robot(&r); // SHOULD BE (0, -1.33), -0.78 (-90deg); (facing 'down')
            break;
        case 2:
            grab_trash(&r);
    		print_robot(&r);
            break;
        default:
            printf("pass");
    }
            
    
    /*
    // SIZE OF BLOB TEST
    camera_open_black();
    while(1){
    	camera_update();
        int n_blobs = get_object_count(RED);
        if(n_blobs > 0) {
            printf("%d\n", get_object_area(RED, 0));
        }
    }
    */

    disable_servo(0);
    printf("Goodbye\n");
    return 0;
}
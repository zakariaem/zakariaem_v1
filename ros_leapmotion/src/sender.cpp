#include <stdio.h>
#include <stdlib.h>
#include "LeapC.h"
#include "ExampleConnection.h"
#include "ros/ros.h"
#include "leapmsg.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

ros::Publisher leap_pub;

static LEAP_CONNECTION* connectionHandle;


// convertion from leap variable to ros geometry_msgs
geometry_msgs::Point LeapVectorToPoint(LEAP_VECTOR vector) {
	geometry_msgs::Point point;
	point.x = vector.x;
	point.y = vector.y;
	point.z = vector.z;
	return point;
}

geometry_msgs::Vector3 LeapVectorToVector3(LEAP_VECTOR lvector) {
	geometry_msgs::Vector3 vector3;
	vector3.x = lvector.x;
	vector3.y = lvector.y;
	vector3.z = lvector.z;
	return vector3;
}

/** Callback for when the connection opens. */
static void OnConnect(){
  printf("Connected.\n");
}

/** Callback for when a device is found. */
static void OnDevice(const LEAP_DEVICE_INFO *props){
  printf("Found device %s.\n", props->serial);
}

/** Callback for when a frame of tracking data is available. */
static void OnFrame(const LEAP_TRACKING_EVENT *frame){
  printf("Frame %lli with %i hands.\n", (long long int)frame->info.frame_id, frame->nHands);
  
  leapmotion::leapmsg leapmsg;

  for(uint32_t h = 0; h < frame->nHands; h++){
    LEAP_HAND* hand = &frame->pHands[h];
	if (hand->type == eLeapHandType_Right) {
		// we send information only about the right hand
		
		//print for debug
		/*printf("    Hand id %i is a %s hand with position (%f, %f, %f).\n",
			hand->id,
			(hand->type == eLeapHandType_Left ? "left" : "right"),
			hand->palm.position.x,
			hand->palm.position.y,
			hand->palm.position.z);
			*/

		leapmsg.header.frame_id = "leapmsg";
		leapmsg.header.stamp = ros::Time::now();
		leapmsg.hands_count = frame->nHands;

		leapmsg.palm_normal = LeapVectorToVector3(hand->palm.normal);
		leapmsg.hand_direction = LeapVectorToVector3(hand->palm.direction);
		leapmsg.palmpos = LeapVectorToPoint(hand->palm.position);

		//computation of the elbow -> wrist vector
		geometry_msgs::Vector3 arm_direction;
		arm_direction.x = hand->arm.next_joint.x - hand->arm.prev_joint.x;
		arm_direction.y = hand->arm.next_joint.y - hand->arm.prev_joint.y;
		arm_direction.z = hand->arm.next_joint.z - hand->arm.prev_joint.z;
		float n;
		n = sqrt(arm_direction.x * arm_direction.x + arm_direction.y * arm_direction.y + arm_direction.z * arm_direction.z);
		arm_direction.x = arm_direction.x / n;
		arm_direction.y = arm_direction.y / n;
		arm_direction.z = arm_direction.z / n;

		//fill the leap message
		leapmsg.arm_direction = arm_direction;
		leapmsg.wrist = LeapVectorToPoint(hand->arm.next_joint);
		leapmsg.elbow = LeapVectorToPoint(hand->arm.prev_joint);

		leapmsg.thumb_metacarpal = LeapVectorToPoint(hand->thumb.metacarpal.prev_joint);
		leapmsg.thumb_proximal = LeapVectorToPoint(hand->thumb.proximal.prev_joint);
		leapmsg.thumb_intermediate = LeapVectorToPoint(hand->thumb.intermediate.prev_joint);
		leapmsg.thumb_distal = LeapVectorToPoint(hand->thumb.distal.prev_joint);
		leapmsg.thumb_tip = LeapVectorToPoint(hand->thumb.distal.next_joint);

		leapmsg.index_metacarpal = LeapVectorToPoint(hand->index.metacarpal.prev_joint);
		leapmsg.index_proximal = LeapVectorToPoint(hand->index.proximal.prev_joint);
		leapmsg.index_intermediate = LeapVectorToPoint(hand->index.intermediate.prev_joint);
		leapmsg.index_distal = LeapVectorToPoint(hand->index.distal.prev_joint);
		leapmsg.index_tip = LeapVectorToPoint(hand->index.distal.next_joint);

		leapmsg.middle_metacarpal = LeapVectorToPoint(hand->middle.metacarpal.prev_joint);
		leapmsg.middle_proximal = LeapVectorToPoint(hand->middle.proximal.prev_joint);
		leapmsg.middle_intermediate = LeapVectorToPoint(hand->middle.intermediate.prev_joint);
		leapmsg.middle_distal = LeapVectorToPoint(hand->middle.distal.prev_joint);
		leapmsg.middle_tip = LeapVectorToPoint(hand->middle.distal.next_joint);

		leapmsg.ring_metacarpal = LeapVectorToPoint(hand->ring.metacarpal.prev_joint);
		leapmsg.ring_proximal = LeapVectorToPoint(hand->ring.proximal.prev_joint);
		leapmsg.ring_intermediate = LeapVectorToPoint(hand->ring.intermediate.prev_joint);
		leapmsg.ring_distal = LeapVectorToPoint(hand->ring.distal.prev_joint);
		leapmsg.ring_tip = LeapVectorToPoint(hand->ring.distal.next_joint);

		leapmsg.pinky_metacarpal = LeapVectorToPoint(hand->pinky.metacarpal.prev_joint);
		leapmsg.pinky_proximal = LeapVectorToPoint(hand->pinky.proximal.prev_joint);
		leapmsg.pinky_intermediate = LeapVectorToPoint(hand->pinky.intermediate.prev_joint);
		leapmsg.pinky_distal = LeapVectorToPoint(hand->pinky.distal.prev_joint);
		leapmsg.pinky_tip = LeapVectorToPoint(hand->pinky.distal.next_joint);

		//publish on the ros topic
		leap_pub.publish(leapmsg);
	}
  }

}

static void OnImage(const LEAP_IMAGE_EVENT *image){
  printf("Image %lli  => Left: %d x %d (bpp=%d), Right: %d x %d (bpp=%d)\n",
      (long long int)image->info.frame_id,
      image->image[0].properties.width,image->image[0].properties.height,image->image[0].properties.bpp*8,
      image->image[1].properties.width,image->image[1].properties.height,image->image[1].properties.bpp*8);
}

static void OnLogMessage(const eLeapLogSeverity severity, const int64_t timestamp,
                         const char* message) {
  const char* severity_str;
  switch(severity) {
    case eLeapLogSeverity_Critical:
      severity_str = "Critical";
      break;
    case eLeapLogSeverity_Warning:
      severity_str = "Warning";
      break;
    case eLeapLogSeverity_Information:
      severity_str = "Info";
      break;
    default:
      severity_str = "";
      break;
  }
  printf("[%s][%lli] %s\n", severity_str, (long long int)timestamp, message);
}

static void* allocate(uint32_t size, eLeapAllocatorType typeHint, void* state) {
  void* ptr = malloc(size);
  return ptr;
}

static void deallocate(void* ptr, void* state) {
  if (!ptr)
    return;
  free(ptr);
}

void OnPointMappingChange(const LEAP_POINT_MAPPING_CHANGE_EVENT *change){
  if (!connectionHandle)
    return;

  uint64_t size = 0;
  if (LeapGetPointMappingSize(*connectionHandle, &size) != eLeapRS_Success || !size)
    return;

  LEAP_POINT_MAPPING* pointMapping = (LEAP_POINT_MAPPING*)malloc(size);
  if (!pointMapping)
    return;

  if (LeapGetPointMapping(*connectionHandle, pointMapping, &size) == eLeapRS_Success &&
      pointMapping->nPoints > 0) {
    printf("Managing %u points as of frame %lld at %lld\n", pointMapping->nPoints, (long long int)pointMapping->frame_id, (long long int)pointMapping->timestamp);
  }
  free(pointMapping);
}

void OnHeadPose(const LEAP_HEAD_POSE_EVENT *event) {
  printf("Head pose:\n");
  printf("    Head position (%f, %f, %f).\n",
    event->head_position.x,
    event->head_position.y,
    event->head_position.z);
  printf("    Head orientation (%f, %f, %f, %f).\n",
    event->head_orientation.w,
    event->head_orientation.x,
    event->head_orientation.y,
    event->head_orientation.z);
 }

int main(int argc, char** argv) {

	ros::init(argc, argv, "leapmotion");

	ros::NodeHandle n;
	leap_pub = n.advertise<leapmotion::leapmsg>("Leapmotion_raw", 1000);

  //Set callback function pointers
  ConnectionCallbacks.on_connection          = &OnConnect;
  ConnectionCallbacks.on_device_found        = &OnDevice;
  ConnectionCallbacks.on_frame               = &OnFrame;
  ConnectionCallbacks.on_image               = &OnImage;
  ConnectionCallbacks.on_point_mapping_change = &OnPointMappingChange;
  ConnectionCallbacks.on_log_message         = &OnLogMessage;
  ConnectionCallbacks.on_head_pose           = &OnHeadPose;


  connectionHandle = OpenConnection();
  {
    LEAP_ALLOCATOR allocator = { allocate, deallocate, NULL };
    LeapSetAllocator(*connectionHandle, &allocator);
  }
  LeapSetPolicyFlags(*connectionHandle, eLeapPolicyFlag_Images | eLeapPolicyFlag_MapPoints, 0);

  printf("Press Enter to exit program.\n");
  getchar();

  DestroyConnection();

  return 0;
}

#include <string>
#include <vector>
#include <map>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <leapmsg.h>
#include <math.h>

#ifndef HAND_ESTIMATION_H
#define HAND_ESTIMATION_H

// bones vector are normalized
// joint vector are not normalized
// wrist vector is the palm normal

using namespace ros_zakariaem;
using namespace geometry_msgs;
using namespace std;

namespace Hand_model{ 
float forearm_size = 200;

Vector3 cross_product(Vector3 u, Vector3 v){
	Vector3 w;
	w.x=u.y*v.z-u.z*v.y;
	w.y=u.z*v.x-u.x*v.z;
	w.z=u.x*v.y-u.y*v.x;
	return w;
}

float scalar_product(Vector3 u, Vector3 v){
	float w;
	w=u.x*v.x+u.y*v.y+u.z*v.z;
	return w;
}


float norm(Vector3 u){
	float n;
	n=sqrt(u.x*u.x+u.y*u.y+u.z*u.z);
	return n;
}

Vector3 normalize(Vector3 u){
	Vector3 n;
	n.x=u.x/norm(u);
	n.y=u.y/norm(u);
	n.z=u.z/norm(u);
	return n;
}

float angle(Vector3 u, Vector3 v){
	float output;
	output=scalar_product(normalize(u),normalize(v));
	output=acos(output);
	return output;
}

struct Bone{
	
	Bone(){}
	Bone(const char* write_name){
		name=write_name;
	}
	void compute_from_base_end(){//vectors are normalized
	center.x=(base.x+end.x)/2;
	center.y=(base.y+end.y)/2;
	center.z=(base.z+end.z)/2;
	vector.x=end.x-base.x;
	vector.y=end.y-base.y;
	vector.z=end.z-base.z;
	size=sqrt(pow(vector.x,2)+pow(vector.y,2)+pow(vector.z,2));
	vector=normalize(vector);
	invertyz();
	}

	void invertyz(){//invert the y and z for the center, end, base point and the vector
		float temp;
		temp=center.y;
		center.y=-center.z;
		center.z=temp;
		temp=base.y;
		base.y=-base.z;
		base.z=temp;
		temp=end.y;
		end.y=-end.z;
		end.z=temp;
		temp=vector.y;
		vector.y=-vector.z;
		vector.z=temp;
	}
	
	Vector3 vector;
	Point center; //not use for palm and wrist
	Point base; 
	Point end; //not used for palm
	float size;
	string name;
};

struct Joint{
	Joint(const char* write_name){
		name=write_name;
	}
	void init(Bone parent_bone, Bone child_bone){
		parent=parent_bone;
		child=child_bone;
		center=child_bone.base;
		vector=normalize(cross_product(parent_bone.vector,child_bone.vector));
		value=angle(parent_bone.vector,child_bone.vector);

	}
	Point center;
	Vector3 vector; //not normalized
	float value;
	string name;
	Bone parent;
	Bone child;
};


class Hand_estimation{
	
	public:

	Hand_estimation():
		forearm("forearm"),
		wrist("wrist"),
		thumb_metacarpal("thumb_metacarpal"),
		thumb_intermediate("thumb_intermediate"),
		thumb_distal("thumb_distal"),
		index_metacarpal("index_metacarpal"),
		index_proximal("index_proximal"),
		index_intermediate("index_intermediate"),
		index_distal("index_distal"),
		middle_metacarpal("middle_metacarpal"),
		middle_proximal("middle_proximal"),
		middle_intermediate("middle_intermediate"),
		middle_distal("middle_distal"),
		ring_metacarpal("ring_metacarpal"),
		ring_proximal("ring_proximal"),
		ring_intermediate("ring_intermediate"),
		ring_distal("ring_distal"),
		pinky_metacarpal("pinky_metacarpal"),
		pinky_proximal("pinky_proximal"),
		pinky_intermediate("pinky_intermediate"),
		pinky_distal("pinky_distal"),

		wrist_flexion("wrist_flexion"),
		wrist_deviation("wrist_deviation"),
		wrist_thumb_metacarpal_flexion("wrist_thumb_metacarpal_flexion"),
		wrist_thumb_metacarpal_abduction("wrist_thumb_metacarpal_abduction"),
		thumb_metacarpal_intermediate("thumb_metacarpal_intermediate"),
		thumb_intermediate_distal("thumb_intermediate_distal"),
		wrist_index_metacarpal("wrist_index_metacarpal"),
		index_metacarpal_proximal("index_metacarpal_proximal"),
		index_proximal_intermediate("index_proximal_intermediate"),
		index_intermediate_distal("index_intermediate_distal"),
		wrist_middle_metacarpal("wrist_middle_metacarpal"),
		middle_metacarpal_proximal("middle_metacarpal_proximal"),
		middle_proximal_intermediate("middle_proximal_intermediate"),
		middle_intermediate_distal("middle_intermediate_distal"),
		wrist_ring_metacarpal("wrist_ring_metacarpal"),
		ring_metacarpal_proximal("ring_metacarpal_proximal"),
		ring_proximal_intermediate("ring_proximal_intermediate"),
		ring_intermediate_distal("ring_intermediate_distal"),
		wrist_pinky_metacarpal("wrist_pinky_metacarpal"),
		pinky_metacarpal_proximal("pinky_metacarpal_proximal"),
		pinky_proximal_intermediate("pinky_proximal_intermediate"),
		pinky_intermediate_distal("pinky_intermediate_distal")
		{empty=true;}

	bool is_empty(){
		return empty;
	}


	void compute_model_from_leap_msg(leapmsg msg){
		load_msg(msg);
		init_bones_from_leap_msg();
		init_joints_from_bones();
		init_lists();
		empty=false;
	}

	void load_msg(leapmsg msg){
		leap_msg=msg;
	}

	void init_bones_from_leap_msg(){
		//palm is one point -> center=base=end
		//
		//load base point of each Bone	
		thumb_metacarpal.base=leap_msg.thumb_metacarpal;
		thumb_intermediate.base=leap_msg.thumb_intermediate;
		thumb_distal.base=leap_msg.thumb_distal;
		index_metacarpal.base=leap_msg.index_metacarpal;
		index_proximal.base=leap_msg.index_proximal;
		index_intermediate.base=leap_msg.index_intermediate;
		index_distal.base=leap_msg.index_distal;
		middle_metacarpal.base=leap_msg.middle_metacarpal;
		middle_proximal.base=leap_msg.middle_proximal;
		middle_intermediate.base=leap_msg.middle_intermediate;
		middle_distal.base=leap_msg.middle_distal;
		ring_metacarpal.base=leap_msg.ring_metacarpal;
		ring_proximal.base=leap_msg.ring_proximal;
		ring_intermediate.base=leap_msg.ring_intermediate;
		ring_distal.base=leap_msg.ring_distal;
		pinky_metacarpal.base=leap_msg.pinky_metacarpal;
		pinky_proximal.base=leap_msg.pinky_proximal;
		pinky_intermediate.base=leap_msg.pinky_intermediate;
		pinky_distal.base=leap_msg.pinky_distal;
		forearm.base=leap_msg.elbow;
		wrist.base=thumb_metacarpal.base;
		
		//load end point of each Bone (exept forearm)
		forearm.end=leap_msg.wrist;
		wrist.end=pinky_metacarpal.base;
		thumb_metacarpal.end=leap_msg.thumb_intermediate;
		thumb_intermediate.end=leap_msg.thumb_distal;
		thumb_distal.end=leap_msg.thumb_tip;
		index_metacarpal.end=leap_msg.index_proximal;
		index_proximal.end=leap_msg.index_intermediate;
		index_intermediate.end=leap_msg.index_distal;
		index_distal.end=leap_msg.index_tip;
		middle_metacarpal.end=leap_msg.middle_proximal;
		middle_proximal.end=leap_msg.middle_intermediate;
		middle_intermediate.end=leap_msg.middle_distal;
		middle_distal.end=leap_msg.middle_tip;
		ring_metacarpal.end=leap_msg.ring_proximal;
		ring_proximal.end=leap_msg.ring_intermediate;
		ring_intermediate.end=leap_msg.ring_distal;
		ring_distal.end=leap_msg.ring_tip;
		pinky_metacarpal.end=leap_msg.pinky_proximal;
		pinky_proximal.end=leap_msg.pinky_intermediate;
		pinky_intermediate.end=leap_msg.pinky_distal;
		pinky_distal.end=leap_msg.pinky_tip;
	
		//fill the bones datas (vector, center of gravity and size)
		//wrist
		wrist.size=sqrt(pow(wrist.end.x-wrist.base.x,2)+pow(wrist.end.y-wrist.base.y,2)+pow(wrist.end.z-wrist.base.z,2));
		wrist.center=leap_msg.wrist;
		wrist.vector=normalize(leap_msg.palm_normal);
		wrist.invertyz();


		// compute vector, centers, size and invert yz	
		forearm.compute_from_base_end();
		thumb_metacarpal.compute_from_base_end();
		thumb_intermediate.compute_from_base_end();
		thumb_distal.compute_from_base_end();
		index_metacarpal.compute_from_base_end();
		index_proximal.compute_from_base_end();
		index_intermediate.compute_from_base_end();
		index_distal.compute_from_base_end();
		middle_metacarpal.compute_from_base_end();
		middle_proximal.compute_from_base_end();
		middle_intermediate.compute_from_base_end();
		middle_distal.compute_from_base_end();
		ring_metacarpal.compute_from_base_end();
		ring_proximal.compute_from_base_end();
		ring_intermediate.compute_from_base_end();
		ring_distal.compute_from_base_end();
		pinky_metacarpal.compute_from_base_end();
		pinky_proximal.compute_from_base_end();
		pinky_intermediate.compute_from_base_end();
		pinky_distal.compute_from_base_end();	
	}

	void init_joints_from_bones(){

		wrist_flexion.init(forearm,wrist);
		wrist_deviation.init(forearm,wrist);
		//correction of vector and angle for the thumb base and wrist (center of forearm_wrist on center of wrist and not base)
		wrist_flexion.center=wrist.center;
		wrist_deviation.center=wrist.center;
		//computation of the sagittal end frontal plane
		Vector3 sagittal_plane_normal; 
		sagittal_plane_normal=cross_product(wrist.vector,middle_metacarpal.vector);
		sagittal_plane_normal=normalize(sagittal_plane_normal);
		Vector3 frontal_plane_normal=normalize(wrist.vector);
			
		wrist_flexion.vector=sagittal_plane_normal;
		wrist_deviation.vector=frontal_plane_normal;
		wrist_flexion.value=asin(scalar_product(forearm.vector,frontal_plane_normal));
		wrist_deviation.value=asin(scalar_product(forearm.vector,sagittal_plane_normal));

		//thumb base universal joint
		wrist_thumb_metacarpal_flexion.init(wrist,thumb_metacarpal);
		wrist_thumb_metacarpal_abduction.init(wrist,thumb_metacarpal);
		wrist_thumb_metacarpal_flexion.vector=frontal_plane_normal;
		wrist_thumb_metacarpal_abduction.vector=sagittal_plane_normal;
		wrist_thumb_metacarpal_flexion.value=asin(scalar_product(thumb_metacarpal.vector,sagittal_plane_normal));
		wrist_thumb_metacarpal_abduction.value=asin(scalar_product(thumb_metacarpal.vector,frontal_plane_normal));
		
		//metacarpal base
		wrist_index_metacarpal.init(wrist,index_metacarpal);
		wrist_index_metacarpal.vector=normalize(cross_product(wrist.vector,index_metacarpal.vector));
		wrist_middle_metacarpal.init(wrist,middle_metacarpal);
		wrist_middle_metacarpal.vector=normalize(cross_product(wrist.vector,middle_metacarpal.vector));
		wrist_ring_metacarpal.init(wrist,ring_metacarpal);
		wrist_ring_metacarpal.vector=normalize(cross_product(wrist.vector,ring_metacarpal.vector));
		wrist_pinky_metacarpal.init(wrist,pinky_metacarpal);
		wrist_pinky_metacarpal.vector=normalize(cross_product(wrist.vector,pinky_metacarpal.vector));

		//metacarpal ends
		index_metacarpal_proximal.init(index_metacarpal,index_proximal);
		index_metacarpal_proximal.vector=normalize(cross_product(index_metacarpal.vector,wrist.vector));
		middle_metacarpal_proximal.init(middle_metacarpal,middle_proximal);
		middle_metacarpal_proximal.vector=normalize(cross_product(middle_metacarpal.vector,wrist.vector));
		ring_metacarpal_proximal.init(ring_metacarpal,ring_proximal);
		ring_metacarpal_proximal.vector=normalize(cross_product(ring_metacarpal.vector,wrist.vector));
		pinky_metacarpal_proximal.init(pinky_metacarpal,pinky_proximal);
		pinky_metacarpal_proximal.vector=normalize(cross_product(pinky_metacarpal.vector,wrist.vector));
		//
		thumb_metacarpal_intermediate.init(thumb_metacarpal,thumb_intermediate);
		thumb_intermediate_distal.init(thumb_intermediate,thumb_distal);
		
		index_proximal_intermediate.init(index_proximal,index_intermediate);
		index_intermediate_distal.init(index_intermediate,index_distal);
		
		middle_proximal_intermediate.init(middle_proximal,middle_intermediate);
		middle_intermediate_distal.init(middle_intermediate,middle_distal);
		
		ring_proximal_intermediate.init(ring_proximal,ring_intermediate);
		ring_intermediate_distal.init(ring_intermediate,ring_distal);
		
		pinky_proximal_intermediate.init(pinky_proximal,pinky_intermediate);
		pinky_intermediate_distal.init(pinky_intermediate,pinky_distal);
		
		//joint coordinates of wrist-proximal are not exported as they are constant
		joint_coordinates[0]=wrist_flexion.value;
		joint_coordinates[1]=wrist_deviation.value;
		joint_coordinates[2]=wrist_thumb_metacarpal_flexion.value;
		joint_coordinates[3]=wrist_thumb_metacarpal_abduction.value;
		joint_coordinates[4]=thumb_metacarpal_intermediate.value;
		joint_coordinates[5]=thumb_intermediate_distal.value;
		joint_coordinates[6]=index_metacarpal_proximal.value;
		joint_coordinates[7]=index_proximal_intermediate.value;
		joint_coordinates[8]=index_intermediate_distal.value;
		joint_coordinates[9]=middle_metacarpal_proximal.value;
		joint_coordinates[10]=middle_proximal_intermediate.value;
		joint_coordinates[11]=middle_intermediate_distal.value;
		joint_coordinates[12]=ring_metacarpal_proximal.value;
		joint_coordinates[13]=ring_proximal_intermediate.value;
		joint_coordinates[14]=ring_intermediate_distal.value;
		joint_coordinates[15]=pinky_metacarpal_proximal.value;
		joint_coordinates[16]=pinky_proximal_intermediate.value;
		joint_coordinates[17]=pinky_intermediate_distal.value;
	}
	void init_lists(){
		bones.clear();
		joints.clear();

		bones.push_back(forearm);
		bones.push_back(wrist);
		bones.push_back(thumb_metacarpal);
		bones.push_back(thumb_intermediate);
		bones.push_back(thumb_distal);
		bones.push_back(index_metacarpal);
		bones.push_back(index_proximal);
		bones.push_back(index_intermediate);
		bones.push_back(index_distal);
		bones.push_back(middle_metacarpal);
		bones.push_back(middle_proximal);
		bones.push_back(middle_intermediate);
		bones.push_back(middle_distal);
		bones.push_back(ring_metacarpal);
		bones.push_back(ring_proximal);
		bones.push_back(ring_intermediate);
		bones.push_back(ring_distal);
		bones.push_back(pinky_metacarpal);
		bones.push_back(pinky_proximal);
		bones.push_back(pinky_intermediate);
		bones.push_back(pinky_distal);
		
		joints.push_back(wrist_flexion);
		joints.push_back(wrist_deviation);
		joints.push_back(wrist_thumb_metacarpal_flexion);
		joints.push_back(wrist_thumb_metacarpal_abduction);
		joints.push_back(thumb_metacarpal_intermediate);
		joints.push_back(thumb_intermediate_distal);
		joints.push_back(wrist_index_metacarpal);
		joints.push_back(index_metacarpal_proximal);
		joints.push_back(index_proximal_intermediate);
		joints.push_back(index_intermediate_distal);
		joints.push_back(wrist_middle_metacarpal);
		joints.push_back(middle_metacarpal_proximal);
		joints.push_back(middle_proximal_intermediate);
		joints.push_back(middle_intermediate_distal);
		joints.push_back(wrist_ring_metacarpal);
		joints.push_back(ring_metacarpal_proximal);
		joints.push_back(ring_proximal_intermediate);
		joints.push_back(ring_intermediate_distal);
		joints.push_back(wrist_pinky_metacarpal);
		joints.push_back(pinky_metacarpal_proximal);
		joints.push_back(pinky_proximal_intermediate);
		joints.push_back(pinky_intermediate_distal);
		
	}


	leapmsg leap_msg;
	
	Bone forearm;
	Bone wrist;
	Bone thumb_metacarpal;
	Bone thumb_intermediate;
	Bone thumb_distal;
	Bone index_metacarpal;
	Bone index_proximal;
	Bone index_intermediate;
	Bone index_distal;
	Bone middle_metacarpal;
	Bone middle_proximal;
	Bone middle_intermediate;
	Bone middle_distal;
	Bone ring_metacarpal;
	Bone ring_proximal;
	Bone ring_intermediate;
	Bone ring_distal;
	Bone pinky_metacarpal;
	Bone pinky_proximal;
	Bone pinky_intermediate;
	Bone pinky_distal;
	std::list<Bone> bones;

	Joint wrist_flexion;
	Joint wrist_deviation;
	Joint wrist_thumb_metacarpal_flexion;
	Joint wrist_thumb_metacarpal_abduction;
	Joint thumb_metacarpal_intermediate;
	Joint thumb_intermediate_distal;
	Joint wrist_index_metacarpal;
	Joint index_metacarpal_proximal;
	Joint index_proximal_intermediate;
	Joint index_intermediate_distal;
	Joint wrist_middle_metacarpal;
	Joint middle_metacarpal_proximal;
	Joint middle_proximal_intermediate;
	Joint middle_intermediate_distal;
	Joint wrist_ring_metacarpal;
	Joint ring_metacarpal_proximal;
	Joint ring_proximal_intermediate;
	Joint ring_intermediate_distal;
	Joint wrist_pinky_metacarpal;
	Joint pinky_metacarpal_proximal;
	Joint pinky_proximal_intermediate;
	Joint pinky_intermediate_distal;
	std::list<Joint> joints;
	float joint_coordinates[22];

	bool empty;
};

} //end spacename
#endif

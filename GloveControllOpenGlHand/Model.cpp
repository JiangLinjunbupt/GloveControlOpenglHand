
#include "Model.h"
#include <iostream>
#include <fstream>
#define PI 3.1415926

#include "util.h"
#include <Eigen/LU>
Model::Model() {


}

Model::Model(char* file) {
	bvh_.Load(file);
}

Model::~Model() {


}


void Model::set_dof() {
	BVH::Joint* joint = bvh_.GetJoint(0);
	joint->dof[0] = true;  joint->dof[1] = true; joint->dof[2] = true;
	joint->corresponds[0] = 0; joint->corresponds[1] = 1; joint->corresponds[2] = 2;

	joint = bvh_.GetJoint(1);
	joint->dof[0] = false; joint->dof[1] = true; joint->dof[2] = true;
	joint->corresponds[1] = 3; joint->corresponds[2] = 4;


	joint = bvh_.GetJoint(2);
	joint->dof[0] = false; joint->dof[1] = true; joint->dof[2] = false;
	joint->corresponds[1] = 5;

	joint = bvh_.GetJoint(3);
	joint->dof[0] = false; joint->dof[1] = true; joint->dof[2] = false;
	joint->corresponds[1] = 6;

	joint = bvh_.GetJoint(4);
	joint->dof[0] = false; joint->dof[1] = false; joint->dof[2] = false;

	joint = bvh_.GetJoint(5);
	joint->dof[0] = false; joint->dof[1] = true; joint->dof[2] = true;
	joint->corresponds[1] = 7; joint->corresponds[2] = 8;

	joint = bvh_.GetJoint(6);
	joint->dof[0] = false; joint->dof[1] = true; joint->dof[2] = false;
	joint->corresponds[1] = 9;

	joint = bvh_.GetJoint(7);
	joint->dof[0] = false; joint->dof[1] = true; joint->dof[2] = false;
	joint->corresponds[1] = 10;

	joint = bvh_.GetJoint(8);
	joint->dof[0] = false; joint->dof[1] = false; joint->dof[2] = false;

	joint = bvh_.GetJoint(9);
	joint->dof[0] = false; joint->dof[1] = true; joint->dof[2] = true;
	joint->corresponds[1] = 11; joint->corresponds[2] = 12;

	joint = bvh_.GetJoint(10);
	joint->dof[0] = false; joint->dof[1] = true; joint->dof[2] = false;
	joint->corresponds[1] = 13;

	joint = bvh_.GetJoint(11);
	joint->dof[0] = false; joint->dof[1] = true; joint->dof[2] = false;
	joint->corresponds[1] = 14;

	joint = bvh_.GetJoint(12);
	joint->dof[0] = false; joint->dof[1] = false; joint->dof[2] = false;

	joint = bvh_.GetJoint(13);
	joint->dof[0] = false; joint->dof[1] = true; joint->dof[2] = true;
	joint->corresponds[1] = 15; joint->corresponds[2] = 16;

	joint = bvh_.GetJoint(14);
	joint->dof[0] = false; joint->dof[1] = true; joint->dof[2] = false;
	joint->corresponds[1] = 17;

	joint = bvh_.GetJoint(15);
	joint->dof[0] = false; joint->dof[1] = true; joint->dof[2] = false;
	joint->corresponds[1] = 18;

	joint = bvh_.GetJoint(16);
	joint->dof[0] = false; joint->dof[1] = false; joint->dof[2] = false;

	joint = bvh_.GetJoint(17);
	joint->dof[0] = false; joint->dof[1] = true; joint->dof[2] = true;
	joint->corresponds[1] = 19; joint->corresponds[2] = 20;

	joint = bvh_.GetJoint(18);
	joint->dof[0] = false; joint->dof[1] = false; joint->dof[2] = true;
	joint->corresponds[1] = 21;

	joint = bvh_.GetJoint(19);
	joint->dof[0] = false; joint->dof[1] = false; joint->dof[2] = true;
	joint->corresponds[1] = 22;

	joint = bvh_.GetJoint(20);
	joint->dof[0] = false; joint->dof[1] = false; joint->dof[2] = false;


	joint = bvh_.GetJoint(21);
	joint->dof[0] = false; joint->dof[1] = true; joint->dof[2] = true;
	joint->corresponds[1] = 23; joint->corresponds[2] = 24;

	joint = bvh_.GetJoint(22);
	joint->dof[0] = false; joint->dof[1] = false; joint->dof[2] = false;


	// global position is the last three parameters
	corresponds_[0] = 25;
	corresponds_[1] = 26;
	corresponds_[2] = 27;
}

void Model::set_upper_lower_bound() {
	BVH::Joint* joint = bvh_.GetJoint(0);
	joint->upper.x = 60; joint->lower.x = 0;
	joint->upper.y = 180; joint->lower.y = 0;
	joint->upper.z = 90; joint->lower.z = 90;

	joint = bvh_.GetJoint(1);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 90; joint->lower.y = 0;
	joint->upper.z = 20; joint->lower.z = -20;

	joint = bvh_.GetJoint(2);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 90; joint->lower.y = 0;
	joint->upper.z = 0; joint->lower.z = 0;

	joint = bvh_.GetJoint(3);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 90; joint->lower.y = 0;
	joint->upper.z = 0; joint->lower.z = 0;

	joint = bvh_.GetJoint(4);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 0; joint->lower.y = 0;
	joint->upper.z = 0; joint->lower.z = 0;

	joint = bvh_.GetJoint(5);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 90; joint->lower.y = 0;
	joint->upper.z = 20; joint->lower.z = -20;

	joint = bvh_.GetJoint(6);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 90; joint->lower.y = 0;
	joint->upper.z = 0; joint->lower.z = 0;

	joint = bvh_.GetJoint(7);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 90; joint->lower.y = 0;
	joint->upper.z = 0; joint->lower.z = 0;

	joint = bvh_.GetJoint(8);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 0; joint->lower.y = 0;
	joint->upper.z = 0; joint->lower.z = 0;

	joint = bvh_.GetJoint(9);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 90; joint->lower.y = 0;
	joint->upper.z = 20; joint->lower.z = -20;

	joint = bvh_.GetJoint(10);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 90; joint->lower.y = 0;
	joint->upper.z = 0; joint->lower.z = 0;

	joint = bvh_.GetJoint(11);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 90; joint->lower.y = 0;
	joint->upper.z = 0; joint->lower.z = 0;

	joint = bvh_.GetJoint(12);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 0; joint->lower.y = 0;
	joint->upper.z = 0; joint->lower.z = 0;

	joint = bvh_.GetJoint(13);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 90; joint->lower.y = 0;
	joint->upper.z = 20; joint->lower.z = -20;

	joint = bvh_.GetJoint(14);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 90; joint->lower.y = 0;
	joint->upper.z = 0; joint->lower.z = 0;

	joint = bvh_.GetJoint(15);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 90; joint->lower.y = 0;
	joint->upper.z = 0; joint->lower.z = 0;

	joint = bvh_.GetJoint(16);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 0; joint->lower.y = 0;
	joint->upper.z = 0; joint->lower.z = 0;

	joint = bvh_.GetJoint(17);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 60; joint->lower.y = 0;
	joint->upper.z = 0; joint->lower.z = -40;

	joint = bvh_.GetJoint(18);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 0; joint->lower.y = 0;
	joint->upper.z = 0; joint->lower.z = -90;

	joint = bvh_.GetJoint(19);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 0; joint->lower.y = 0;
	joint->upper.z = 0; joint->lower.z = -90;

	joint = bvh_.GetJoint(20);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 0; joint->lower.y = 0;
	joint->upper.z = 0; joint->lower.z = 0;

	joint = bvh_.GetJoint(21);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 0; joint->lower.y = -0;
	joint->upper.z = 10; joint->lower.z = -10;

	joint = bvh_.GetJoint(22);
	joint->upper.x = 0; joint->lower.x = 0;
	joint->upper.y = 0; joint->lower.y = 0;
	joint->upper.z = 0; joint->lower.z = 0;

}

void Model::save_upper_lower_of_angle(string file) {
	ofstream fo(file, ios::out);
	fo << global_position_center_.x << " " << global_position_center_.y << " "
		<< global_position_center_.z;
	Pose upper, lower;

	for (int i = 0; i < get_number_of_joint(); ++i) {
		upper = get_upper_of_angle(i);
		lower = get_lower_of_angle(i);
		bool* dof = get_dof(i);
		if (dof[0] == true) {
			fo << " " << lower.x << " " << upper.x;
		}
		if (dof[1] == true) {
			fo << " " << lower.y << " " << upper.y;
		}
		if (dof[2] == true) {
			fo << " " << lower.z << " " << upper.z;
		}
	}


	fo.close();

}


void Model::compute_local_inverse() {
	int num_joint = bvh_.GetNumJoint();
	for (int i = 0; i < num_joint; i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);
		joint->local_inverse = joint->local.inverse();
	}

}

void Model::init() {
	set_dof();
	compute_init_global_position();       //��һ����������handbone���������ĵ㣩��������Ϊ�ֵ�ȫ��λ�á�
	compute_local_coordinate();
	compute_parent_child_transform();
	compute_local_inverse();
	set_upper_lower_bound();

	this->load_faces(".\\model\\handfaces.txt");
	this->load_vertices(".\\model\\handverts.txt");
	this->load_weight(".\\model\\weight.txt");


	Pose pose(0, 0, 0);
	pose.x = 0; pose.y = 0; pose.z = -200;
	this->set_global_position(pose);
	this->set_global_position_center(pose);

	this->forward_kinematic();
	this->compute_mesh();
}

void Model::transform_matrix(Pose pose, Eigen::MatrixXd& rot) {
	Eigen::MatrixXd x = Eigen::MatrixXd::Identity(4, 4);
	Eigen::MatrixXd y = Eigen::MatrixXd::Identity(4, 4);
	Eigen::MatrixXd z = Eigen::MatrixXd::Identity(4, 4);

	double cx = cos(pose.x / 180 * PI);
	double sx = sin(pose.x / 180 * PI);

	double cy = cos(pose.y / 180 * PI);
	double sy = sin(pose.y / 180 * PI);

	double cz = cos(pose.z / 180 * PI);
	double sz = sin(pose.z / 180 * PI);

	x(1, 1) = cx; x(2, 2) = cx;
	x(1, 2) = -sx; x(2, 1) = sx;

	y(0, 0) = cy; y(0, 2) = sy;
	y(2, 0) = -sy; y(2, 2) = cy;

	z(0, 0) = cz; z(1, 1) = cz;
	z(0, 1) = -sz; z(1, 0) = sz;

	rot = x*y*z;   //��ת˳�� z-y-x
}

void Model::forward_kinematic() {

	int number_joint = bvh_.GetNumJoint();
	compute_global();
	Eigen::MatrixXd I;
	Eigen::MatrixXd t;
	BVH::Joint* joint0 = bvh_.GetJoint(0);
	//Eigen::Matr
	for (int i = 0; i < number_joint; i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);
		t = Eigen::MatrixXd(4, 1);
		t(0, 0) = joint->init_global_position[0];
		t(1, 0) = joint->init_global_position[1];
		t(2, 0) = joint->init_global_position[2];
		t(3, 0) = 1; /////joint->scale*
					 /*Eigen::MatrixXd SCALE = Eigen::MatrixXd::Identity(4, 4);
					 SCALE(0, 0) = joint->scale;
					 SCALE(1, 1) = joint->scale;
					 SCALE(2, 2) = joint->scale;*/

		I = joint->global* (joint->scale*((joint->local_inverse)*t));
		//I = joint->global* (SCALE*((joint->local_inverse)*t));
		joint->global_position[0] = I(0, 0) + global_position_.x;
		joint->global_position[1] = I(1, 0) + global_position_.y;
		joint->global_position[2] = I(2, 0) + global_position_.z;
	}


	for (int i = 0; i < number_joint; i++) {

		BVH::Joint* joint_handbone = bvh_.GetJoint(1);
		BVH::Joint* joint = bvh_.GetJoint(i);
		t = Eigen::MatrixXd(4, 1);
		t(0, 0) = joint_handbone->init_global_position[0];
		t(1, 0) = joint_handbone->init_global_position[1];
		t(2, 0) = joint_handbone->init_global_position[2];
		t(3, 0) = 1; /////joint->scale*
					 /*Eigen::MatrixXd SCALE = Eigen::MatrixXd::Identity(4, 4);
					 SCALE(0, 0) = joint->scale;
					 SCALE(1, 1) = joint->scale;
					 SCALE(2, 2) = joint->scale;*/

		I = joint_handbone->global* (joint_handbone->scale*((joint_handbone->local_inverse)*t));
		//I = joint->global* (SCALE*((joint->local_inverse)*t));
		joint->global_position[0] = joint->global_position[0] - I(0,0);
		joint->global_position[1] = joint->global_position[1] - I(1,0);
		joint->global_position[2] = joint->global_position[2] - I(2,0);
	}



}



void Model::set_global_position(Pose global_position) {
	global_position_.x = global_position.x;
	global_position_.y = global_position.y;
	global_position_.z = global_position.z;
}

void Model::set_global_position_center(Pose global_position) {
	global_position_center_.x = global_position.x;
	global_position_center_.y = global_position.y;
	global_position_center_.z = global_position.z;

}

void Model::compute_global() {
	int number_joint = bvh_.GetNumJoint();
	for (int i = 0; i < number_joint; i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);

		if (i == 2)
		{
			//����ʳָ�䳤
			/*Eigen::MatrixXd longer = Eigen::MatrixXd::Identity(4, 4);
			longer(0, 3) = 100;
			joint->transR = joint->trans*longer*joint->rotation;*/
			joint->transR = joint->trans*joint->rotation;
		}
		else
		{
			joint->transR = joint->trans*joint->rotation;  //rotation �ڼ���bvh�ļ�ʱ���ʼ��Ϊ4*4��λ����   //����任��˳��ӦΪ����С����ת��ƽ��
		}
	}
	BVH::Joint* joint0 = bvh_.GetJoint(0);
	joint0->global = joint0->local;
	for (int i = 0; i < number_joint; i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);
		BVH::Joint* joint_parent = joint->parent;
		if (joint_parent != nullptr) {
			joint->global = joint_parent->global*joint->transR;        //��joint��0������ת�Ļ����ϼ����
		}
	}
	for (int i = 0; i < number_joint; i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);
		joint->global = joint0->rotation*joint->global;           //����Joint��0������ת
	}
}

void Model::set_one_rotation(Pose pose, int index) {
	BVH::Joint* joint = bvh_.GetJoint(index);
	joint->pose = pose;
	transform_matrix(joint->pose, joint->rotation);
}

void Model::set_hand_rotation(Pose pose)
{
	BVH::Joint *joint = bvh_.GetJoint(0);

	Eigen::MatrixXd z0 = Eigen::MatrixXd::Identity(4, 4);

	double cz0 = cos(-90.0 / 180 * PI);
	double sz0 = sin(-90.0 / 180 * PI);

	z0(0, 0) = cz0;z0(1, 1) = cz0;
	z0(0, 1) = -sz0;z0(1, 0) = sz0;

	Eigen::MatrixXd x = Eigen::MatrixXd::Identity(4, 4);
	Eigen::MatrixXd y = Eigen::MatrixXd::Identity(4, 4);
	Eigen::MatrixXd z = Eigen::MatrixXd::Identity(4, 4);

	double cx = cos(pose.x / 180 * PI);
	double sx = sin(pose.x / 180 * PI);

	double cy = cos(-pose.y / 180 * PI);
	double sy = sin(-pose.y / 180 * PI);

	double cz = cos(-pose.z / 180 * PI);
	double sz = sin(-pose.z / 180 * PI);

	x(1, 1) = cx;x(2, 2) = cx;
	x(1, 2) = -sx;x(2, 1) = sx;

	y(0, 0) = cy;y(0, 2) = sy;
	y(2, 0) = -sy;y(2, 2) = cy;

	z(0, 0) = cz;z(1, 1) = cz;
	z(0, 1) = -sz;z(1, 0) = sz;

	joint->rotation = x*y*z*z0; //��ת˳�� z-y-x
}

void Model::set_thumbLower_rotation(Pose pose)
{
	BVH::Joint *joint = bvh_.GetJoint(18);

	Eigen::MatrixXd x = Eigen::MatrixXd::Identity(4, 4);
	Eigen::MatrixXd y = Eigen::MatrixXd::Identity(4, 4);
	Eigen::MatrixXd z = Eigen::MatrixXd::Identity(4, 4);

	double cx = cos(-pose.y / 180 * PI);
	double sx = sin(-pose.y / 180 * PI);

	double cy = cos(pose.x / 180 * PI);
	double sy = sin(pose.x / 180 * PI);

	double cz = cos(pose.z / 180 * PI);
	double sz = sin(pose.z / 180 * PI);

	x(1, 1) = cx;x(2, 2) = cx;
	x(1, 2) = -sx;x(2, 1) = sx;

	y(0, 0) = cy;y(0, 2) = sy;
	y(2, 0) = -sy;y(2, 2) = cy;

	z(0, 0) = cz;z(1, 1) = cz;
	z(0, 1) = -sz;z(1, 0) = sz;

	joint->rotation = x*y*z; //��ת˳�� z-y-x

}

void Model::set_rotation(Pose* pose) {
	int number_joint = bvh_.GetNumJoint();
	for (int i = 0; i < number_joint; i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);
		joint->pose = pose[i];
		transform_matrix(joint->pose, joint->rotation);
	}
}

void Model::compute_parent_child_transform() {
	int number_joint = bvh_.GetNumJoint();
	for (int i = 0; i < number_joint; i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);
		BVH::Joint* joint_parent = joint->parent;
		if (joint->parent != nullptr) {
			joint->trans = joint_parent->local.inverse()* joint->local;
		}
		else {
			joint->trans = joint->local;
		}
	}
}

void Model::compute_local_coordinate() {
	int number_joint = bvh_.GetNumJoint();
	double axisx[3] = { 0,0,0 };
	double axisy[3] = { 0,0,0 };
	double axisz[3] = { 0,0,1 };
	for (int i = 0; i < number_joint; i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);
		BVH::Joint* joint_parent = joint->parent;
		axisx[0] = .0; axisx[1] = .0; axisx[2] = .0;
		axisz[0] = .0; axisz[1] = .0; axisz[2] = 1.0;


		BVH::Joint* joint_child = nullptr;

		if (joint->children.size() == 0) {
			joint->local = joint_parent->local;
			joint->local(0, 3) = joint->init_global_position[0];
			joint->local(1, 3) = joint->init_global_position[1];
			joint->local(2, 3) = joint->init_global_position[2];
			continue;
		}
		if (joint->children.size() >1) {
			if (i == 0)
			{
				joint_child = joint->children[0];
			}
			else
			{
				joint_child = joint->children[1];
			}
			
		}

		if (joint->children.size() == 1) {
			joint_child = joint->children[0];
		}

		axisx[0] = joint_child->init_global_position[0] - joint->init_global_position[0];
		axisx[1] = joint_child->init_global_position[1] - joint->init_global_position[1];
		axisx[2] = joint_child->init_global_position[2] - joint->init_global_position[2];
		normalize(axisx);
		cross_product(axisx, axisz, axisy);
		normalize(axisy);
		cross_product(axisx, axisy, axisz);
		normalize(axisz);
		set_axis(axisx, axisy, axisz, joint->init_global_position, joint->local);
	}
}

void Model::compute_init_global_position() {
	int number_joint = bvh_.GetNumJoint();
	for (int i = 0; i < number_joint; i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);
		BVH::Joint* joint_parent = joint->parent;
		if (joint_parent != nullptr) {
			joint->init_global_position[0] = joint_parent->init_global_position[0]
				+ joint->offset[0];

			joint->init_global_position[1] = joint_parent->init_global_position[1]
				+ joint->offset[1];

			joint->init_global_position[2] = joint_parent->init_global_position[2]
				+ joint->offset[2];
		}
	}
}


void Model::set_one_target_position(double target_position[3], int index) {
	BVH::Joint* joint = bvh_.GetJoint(index);
	joint->target_position[0] = target_position[0];
	joint->target_position[1] = target_position[1];
	joint->target_position[2] = target_position[2];
}
void Model::set_target_position(double target_position[NUM_JOINT * 3], bool visiable[NUM_JOINT]) {

	for (int i = 0; i < bvh_.GetNumJoint(); i++) {
		BVH::Joint* joint = bvh_.GetJoint(i);
		joint->target_position[0] = target_position[3 * i];
		joint->target_position[1] = target_position[3 * i + 1];
		joint->target_position[2] = target_position[3 * i + 2];
	}

}

void Model::load_faces(char* file) {
	ifstream f;
	f.open(file, ios::in);

	f >> num_faces_;
	faces_ = Eigen::MatrixXi::Zero(num_faces_, 3);
	for (int i = 0; i < num_faces_; i++) {
		f >> faces_(i, 0) >> faces_(i, 1) >> faces_(i, 2);
		//cout<< faces_(i,0) <<" "<< faces_(i,1) <<" "<< faces_(i,2)<<endl;
	}
	f.close();
	//cout<< faces_;
}

void Model::load_vertices(char* file) {
	ifstream f;
	f.open(file, ios::in);
	//int number;
	f >> num_vertices_;
	vertices_ = Eigen::MatrixXd::Zero(num_vertices_, 3);
	for (int i = 0; i < num_vertices_; i++) {
		f >> vertices_(i, 0) >> vertices_(i, 1) >> vertices_(i, 2);
		vertices_(i, 0) = vertices_(i, 0) - 79.446518;      //����ӵ���handbone�����wrist��ƫ�ƣ�����BVH�ļ��������HandboneΪ��׼�ģ�verticesҲ���Դ�Ϊ��׼�ģ������൱�ڶ����еĵ�ı��׼����
		vertices_(i, 1) = vertices_(i, 1) + 5.274386;
		vertices_(i, 2) = vertices_(i, 2) - 13.494767;
		//cout<< vertices_(i,0)<<" " << vertices_(i,1)<<" "<<vertices_(i,2)<<endl;
	}
	f.close();
	//cout<< vertices_;
}

void Model::load_weight_0(char* file) {
	fstream f;
	f.open(file, ios::in);
	int number = 0;
	f >> num_vertices_ >> number;
	int x = 0, y = 0;
	int num_joint = bvh_.GetNumJoint();
	assert(num_joint != number);
	weight_ = Eigen::MatrixXd::Zero(num_vertices_, number);
	double weight;
	for (int i = 0; i < num_vertices_; i++) {
		for (int j = 0; j<number; j++) {
			f >> weight;
			weight_(i, j) = weight;
		}
	}
	//cout<<weight_;
	f.close();

}

void Model::load_weight(char* file) {
	fstream f;
	f.open(file, ios::in);
	//int number = 0;
	f >> num_weight_;
	int x = 0, y = 0;
	int num_joint = bvh_.GetNumJoint();
	weight_ = Eigen::MatrixXd::Zero(num_vertices_, num_joint);
	double weight;
	for (int i = 0; i < num_weight_; i++) {
		f >> x >> y >> weight;
		y = y + 1;
		if (y == 22)
		{
			y = 0;
		}
		if (y == 23)
		{
			y = 22;
		}
		weight_(x, y) = weight;

	}
	//cout<<weight_;
	f.close();
}

void Model::compute_mesh() {

	int num_joint = bvh_.GetNumJoint();
	compute_global();
	Eigen::MatrixXd t = Eigen::MatrixXd::Zero(4, num_vertices_);
	Eigen::MatrixXd x = Eigen::MatrixXd::Ones(4, num_vertices_);
	x.block(0, 0, 3, num_vertices_) = vertices_.block(0, 0, num_vertices_, 3).transpose();
	BVH::Joint* joint = nullptr;
	for (int i = 0; i < num_joint; i++) {
		joint = bvh_.GetJoint(i);
		Eigen::MatrixXd y = weight_.block(0, i, num_vertices_, 1);// �����ж��� ���� �ùؽڵ��weight
		Eigen::MatrixXd y0 = y.replicate(1, 4);    //�ֱ������ظ�1�飬���ظ�4�飬���Ϊ��num_vertices_��4����ô��С�ľ���
		Eigen::MatrixXd z = joint->scale * joint->global * joint->local_inverse * x;
		t = t + z.cwiseProduct(y0.transpose());
	}
	vertices_update_ = t.transpose();
	//cout<<vertices_update_;

	BVH::Joint* joint_handbone = bvh_.GetJoint(1);
	t = Eigen::MatrixXd(4, 1);
	t(0, 0) = joint_handbone->init_global_position[0];
	t(1, 0) = joint_handbone->init_global_position[1];
	t(2, 0) = joint_handbone->init_global_position[2];
	t(3, 0) = 1; /////joint->scale*
				 /*Eigen::MatrixXd SCALE = Eigen::MatrixXd::Identity(4, 4);
				 SCALE(0, 0) = joint->scale;
				 SCALE(1, 1) = joint->scale;
				 SCALE(2, 2) = joint->scale;*/
	Eigen::MatrixXd I;
	I = joint_handbone->global* (joint_handbone->scale*((joint_handbone->local_inverse)*t));

	for (int i = 0; i < vertices_update_.rows(); i++) {
		vertices_update_(i, 0) += global_position_.x - I(0,0);
		vertices_update_(i, 1) += global_position_.y - I(1,0);
		vertices_update_(i, 2) += global_position_.z - I(2,0);
	}
	//cout<<vertices_update_;
}

/////////////////////////////////////////
/////////////////////////////////////////


void Model::save(char* file) {
	ofstream f;
	f.open(file, ios::out);
	int number_joint = bvh_.GetNumJoint();
	for (int i = 0; i < number_joint; i++) {
		f << bvh_.GetJoint(i)->global_position[0] << " "
			<< bvh_.GetJoint(i)->global_position[1] << " " <<
			bvh_.GetJoint(i)->global_position[2] << endl;
	}
	f.close();
}

void Model::save2(char* file) {
	ofstream f;
	f.open(file, ios::out);
	int number_joint = bvh_.GetNumJoint();
	for (int i = 0; i < number_joint; i++) {
		f << bvh_.GetJoint(i)->init_global_position[0] << " "
			<< bvh_.GetJoint(i)->init_global_position[1] << " " <<
			bvh_.GetJoint(i)->init_global_position[2] << endl;
	}
	f.close();
}

void Model::save_trans(char* file) {
	ofstream f;
	f.open(file, ios::out);
	int number_joint = bvh_.GetNumJoint();
	for (int i = 0; i < number_joint; i++) {
		f << bvh_.GetJoint(i)->trans << endl;
	}
	f.close();

}
void Model::save_local(char* file) {
	ofstream f;
	f.open(file, ios::out);
	int number_joint = bvh_.GetNumJoint();
	for (int i = 0; i < number_joint; i++) {
		f << bvh_.GetJoint(i)->local << endl;
	}
	f.close();
}

void Model::save_global(char* file) {
	ofstream f;
	f.open(file, ios::out);
	int number_joint = bvh_.GetNumJoint();
	for (int i = 0; i < number_joint; i++) {
		f << bvh_.GetJoint(i)->global << endl;
	}
	f.close();
}
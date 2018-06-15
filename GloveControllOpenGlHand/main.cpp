#define NDEBUG
#include "Model.h"
#include <iostream>
#include "GL/freeglut.h"
#include "Viewer.h"
#include <iostream>
#include "myKinect.h"
#include "GloveData.h"
#include "PointCloud.h"
#include "Projection.h"
#include "global.h"
#include "extern.h"

VisData _data;
Config config;
Control control;
/*线程池*/
ThreadPool threadPool;
bool begain_PSO = false;
void MixShowResult(cv::Mat input1, cv::Mat input2);

#pragma region OpenGL

#pragma region  Keybroad_event(show mesh or not)

void menu(int op) {

	switch (op) {
	case 'Q':
	case 'q':
		exit(0);
	}
}

/* executed when a regular key is pressed */
void keyboardDown(unsigned char key, int x, int y) {

	switch (key) {
	case 'q':
		config.show_mesh = true;
		config.show_point = false;
		config.show_skeleton = false;
		break;
	case 'w':
		config.show_mesh = false;
		config.show_point = true;
		config.show_skeleton = true;
		break;
	case 'b':
		begain_PSO = true;
	case 'e':
		begain_PSO = false;
	case  27:   // ESC
		exit(0);
	}
}

/* executed when a regular key is released */
void keyboardUp(unsigned char key, int x, int y) {

}

/* executed when a special key is pressed */
void keyboardSpecialDown(int k, int x, int y) {

}

/* executed when a special key is released */
void keyboardSpecialUp(int k, int x, int y) {

}
#pragma endregion  Keybroad_event(show mesh or not)


/* reshaped window */
void reshape(int width, int height) {

	GLfloat fieldOfView = 90.0f;
	glViewport(0, 0, (GLsizei)width, (GLsizei)height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fieldOfView, (GLfloat)width / (GLfloat)height, 0.1, 500.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

/* executed when button 'button' is put into state 'state' at screen position ('x', 'y') */
void mouseClick(int button, int state, int x, int y) {
	control.mouse_click = 1;
	control.x = x;
	control.y = y;
}

/* executed when the mouse moves to position ('x', 'y') */
/* render the scene */
void draw() {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	gluPerspective(180, 1.5, -1000, 1000);
	glLoadIdentity();
	control.gx = model->get_global_position().x;
	control.gy = model->get_global_position().y;
	control.gz = model->get_global_position().z;
	double r = 200;
	double x = r*sin(control.roty)*cos(control.rotx);
	double y = r*sin(control.roty)*sin(control.rotx);
	double z = r*cos(control.roty);
	//cout<< x <<" "<< y <<" " << z<<endl;
	gluLookAt(x + control.gx, y + control.gy, z + control.gz, control.gx, control.gy, control.gz, 0.0, 1.0, 0.0);//个人理解最开始是看向-z的，之后的角度是在global中心上叠加的，所以要加


	//画点云，必须放最前面，要是放后面会出错。为什么我也没找到~   -------原因：以前glEnd没加(),程序不会报错，但是运行结果不对。；
	if (pointcloud.pointcloud_vector.size() > 0)
	{
		glPointSize(2);
		glBegin(GL_POINTS);
		glColor3d(0.0, 1.0, 0.0);
		//cout << "the pointcloud size : " << pointcloud.pointcloud_vector.size() << endl;
		for (int i = 0;i < pointcloud.pointcloud_vector.size();i++)
		{
			glVertex3f(pointcloud.pointcloud_vector[i].x, pointcloud.pointcloud_vector[i].y, pointcloud.pointcloud_vector[i].z);
		}
		glEnd();
	}
	/* render the scene here */
	glColor3d(1.0,1.0,1.0);
	if (config.show_point) {
		glPointSize(2);
		glBegin(GL_POINTS);
		glColor3d(1.0, 0.0, 0.0);
		for (int i = 0; i < model->vertices_update_.rows(); i++) {
			glVertex3d(model->vertices_update_(i, 0), model->vertices_update_(i, 1), model->vertices_update_(i, 2));
			//cout<< model->vertices_(i,0)<< " " << model->vertices_(i,1) <<" "<< model->vertices_(i,2)<<endl;
		}
		glEnd();
	}

	if (config.show_mesh) {
		if (_data.indices == nullptr) return;
		if (_data.vertices == nullptr) return;
		glColor3d(0.0, 0.0, 1.0);
		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, _data.vertices);
		glEnableClientState(GL_COLOR_ARRAY);
		glColorPointer(3, GL_FLOAT, 0, _data.colors);
		//glDrawElements(GL_TRIANGLE_STRIP, 12, GL_UNSIGNED_BYTE, indices);
		glDrawElements(GL_TRIANGLES, 3 * _data.num_face, GL_UNSIGNED_INT, _data.indices);

		// deactivate vertex arrays after drawing
		glDisableClientState(GL_VERTEX_ARRAY);

	}
	//glEnable(GL_LIGHTING);
	if (config.show_skeleton) {
		for (int i = 0; i < _data.joints.rows(); i++) {
			//画点开始
			glColor3f(1.0, 0.0, 0.0);
			glPushMatrix();
			glTranslatef(_data.joints(i, 0), _data.joints(i, 1), _data.joints(i, 2));
			glutSolidSphere(5, 31, 10);
			glPopMatrix();
			//画点结束，使用push和popmatrix是因为保证每个关节点的偏移都是相对于全局坐标中心点做的变换。

			//画线开始
			if (i != 0) {
				glLineWidth(5);
				glColor3f(0.0, 1.0, 0);
				glBegin(GL_LINES);
				int ii = _data.joints(i, 3);
				glVertex3f(_data.joints(ii, 0), _data.joints(ii, 1), _data.joints(ii, 2));
				glVertex3f(_data.joints(i, 0), _data.joints(i, 1), _data.joints(i, 2));
				glEnd();
			}

			//画线结束
		}
	}

	//画深度图转换成的点云的点


	glFlush();
	glutSwapBuffers();
}


void mouseMotion(int x, int y) {
	control.rotx = (x - control.x)*0.05;
	control.roty = (y - control.y)*0.05;

	//cout<< control.rotx <<" " << control.roty << endl;
	glutPostRedisplay();
}

/* executed when program is idle */
void idle() {
	if (begain_PSO)
	{
		mykinect.Collectdata();
		pointcloud.DepthMatToPointCloud(mykinect.HandsegmentMat);

		glovedata.HandinfParams[24] = pointcloud.PointCloud_center_x;
		glovedata.HandinfParams[25] = pointcloud.PointCloud_center_y;
		glovedata.HandinfParams[26] = pointcloud.PointCloud_center_z;
		glovedata.GetGloveData();


		float *OptimizedParams;
		poseEstimate(mykinect.HandsegmentMat, glovedata.HandinfParams, model->upper_bound, model->lower_bound, OptimizedParams);

		model->GloveParamsConTrollHand(OptimizedParams);
		model->forward_kinematic();
		model->compute_mesh();

		cv::Mat generated_mat = cv::Mat::zeros(424, 512, CV_16UC1);
		projection->set_color_index(model);
		projection->project_3d_to_2d_(model, generated_mat);
		MixShowResult(mykinect.HandsegmentMat, generated_mat);

		_data.set(model->vertices_update_, model->faces_);
		_data.set_color(model->weight_);
		_data.set_skeleton(model);

		delete OptimizedParams;
		glutPostRedisplay();
	}
	else
	{
		mykinect.Collectdata();
		pointcloud.DepthMatToPointCloud(mykinect.HandsegmentMat);

		glovedata.HandinfParams[24] = pointcloud.PointCloud_center_x;
		glovedata.HandinfParams[25] = pointcloud.PointCloud_center_y;
		glovedata.HandinfParams[26] = pointcloud.PointCloud_center_z;
		glovedata.GetGloveData();


		model->GloveParamsConTrollHand(glovedata.HandinfParams);
		model->forward_kinematic();
		model->compute_mesh();

		cv::Mat generated_mat = cv::Mat::zeros(424, 512, CV_16UC1);
		projection->set_color_index(model);
		projection->project_3d_to_2d_(model, generated_mat);
		MixShowResult(mykinect.HandsegmentMat, generated_mat);

		_data.set(model->vertices_update_, model->faces_);
		_data.set_color(model->weight_);
		_data.set_skeleton(model);

		glutPostRedisplay();
	}
}

/* initialize OpenGL settings */
void initGL(int width, int height) {

	reshape(width, height);

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0f);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
}

#pragma endregion 


void main(int argc, char** argv) {


	mykinect.InitializeDefaultSensor();
	model = new Model(".\\model\\HandBase.bvh");
	model->init();

	pointcloud.pointcloud_vector.clear();

	threadPool.setMaxQueueSize(100);
	int hardware_thread = thread::hardware_concurrency();
	if (hardware_thread == 0) { hardware_thread = 2; } //当系统信息无法获取时，函数会返回0
	int threadNum = hardware_thread - 1;
	cout << "threadNum is：" << threadNum << endl;
	threadPool.start(threadNum);  //启动线程池，创建多个线程，从任务队列取任务作为线程入口函数，取任务时，需要判断队列是否非空


	_data.init(model->vertices_.rows(), model->faces_.rows());
	_data.set(model->vertices_update_, model->faces_);
	_data.set_color(model->weight_);
	_data.set_skeleton(model);


#pragma region OpenGL
	glutInit(&argc, argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Interactron");

	// register glut call backs
	glutKeyboardFunc(keyboardDown);
	glutKeyboardUpFunc(keyboardUp);
	glutSpecialFunc(keyboardSpecialDown);
	glutSpecialUpFunc(keyboardSpecialUp);
	glutMouseFunc(mouseClick);
	glutMotionFunc(mouseMotion);
	glutReshapeFunc(reshape);
	glutDisplayFunc(draw);
	glutIdleFunc(idle);
	glutIgnoreKeyRepeat(true); // ignore keys held down

							   // create a sub menu 
	int subMenu = glutCreateMenu(menu);
	glutAddMenuEntry("Do nothing", 0);
	glutAddMenuEntry("Really Quit", 'q');

	// create main "right click" menu
	glutCreateMenu(menu);
	glutAddSubMenu("Sub Menu", subMenu);
	glutAddMenuEntry("Quit", 'q');
	glutAttachMenu(GLUT_RIGHT_BUTTON);

	initGL(800, 600);

	glutMainLoop();

#pragma endregion

}



void MixShowResult(cv::Mat input1, cv::Mat input2)
{
	int height = input2.rows;
	int width = input2.cols;
	cv::Mat colored_input1 = cv::Mat::zeros(height, width, CV_8UC3);
	cv::Mat colored_input2 = cv::Mat::zeros(height, width, CV_8UC3);
	cv::Mat dst;
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (input1.at<ushort>(i, j) != 0)
			{
				colored_input1.at < cv::Vec3b>(i, j)[0] = 0;
				colored_input1.at < cv::Vec3b>(i, j)[1] = 0;
				colored_input1.at < cv::Vec3b>(i, j)[2] = 255;
			}
			else
			{

				colored_input1.at < cv::Vec3b>(i, j)[0] = 255;
				colored_input1.at < cv::Vec3b>(i, j)[1] = 255;
				colored_input1.at < cv::Vec3b>(i, j)[2] = 255;

			}

			if (input2.at<ushort>(i, j) != 0)
			{
				colored_input2.at < cv::Vec3b>(i, j)[0] = 0;
				colored_input2.at < cv::Vec3b>(i, j)[1] = 255;
				colored_input2.at < cv::Vec3b>(i, j)[2] = 0;
			}
			else
			{

				colored_input2.at < cv::Vec3b>(i, j)[0] = 255;
				colored_input2.at < cv::Vec3b>(i, j)[1] = 255;
				colored_input2.at < cv::Vec3b>(i, j)[2] = 255;

			}

		}
	}

	cv::addWeighted(colored_input1, 0.5, colored_input2, 0.5, 0.0, dst);
	cv::imshow("Mixed Result", dst);

}
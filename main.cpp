//2.0   done

#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Windows.h>
#include <time.h>

#define Radius 5
#define m_pi 3.14159265358979323846f
#define radianof(a) ((a*m_pi)/180.0)
#define particle_number 30
#define resample_size particle_number*2/4
#define Ess_mun particle_number/4
#define LANDMARKNUM 75
#define OBR 4
#define RER 4
#define PERTIME 5

float guassian(float ,float );
void laser(int ,int ,int ,int []);
void PrintMatrix(CvMat *Matrix,int Rows,int Cols);

int round(long double x)
{
	if(x > 0.0)
		return (int)(x+0.5);
	else
		return (int)(x-0.5);
}

CvPoint realo[LANDMARKNUM];


struct particle_member
{

	int update[3];
	int N;
	int new_N;
	double w;
	double new_w;
	double *state;
	double *new_state;
	int *path_x;
	int *path_y;
	int *path_d;
	int *new_path_x;
	int *new_path_y;
	int *new_path_d;
	CvPoint coordinate;
	CvPoint path;
	CvPoint map;

	CvMat *sigma_state;
	CvMat *new_sigma_state;
	CvMat *new_u_state;
	CvMat *u_state;
	CvMat *z;
};

struct robot_member
{
	int x,y,d;
	int *path_x;
	int *path_y;
	int *path_d;
	int range_finder[181];
	float obj_x;
	float obj_y;
	CvPoint laser_line;//畫機器人雷射掃描線用
	CvPoint true_path;//存機器人歷史軌跡
	CvPoint coordinate;//存機器人位置(x,y)
	CvPoint dir;//畫機器人方向用的線條所需之座標

	CvMat *z;
};

void OpenCV_Angle_Correct(int particle_num,int particle_N);

IplImage *frame;
IplImage *Image1;
CvScalar Color;
CvSize ImageSize1;
char map_name[] = "landmark3.bmp" ;

int map_x,map_y;

	int i,j,k,l,m;
	int total_time=300;
	char keyboard=0;
	int degree;
	double *p;
	double p0 = 0.000001;
	double p_temp;
	int n_head=0;
	double sum_w=0;
	double ESS=0;
	int index[resample_size] = {0};
	double w_temp[resample_size] = {0};

	char robot_path[]="wwwwwwwwawwwwwawwwwwwdddwwwwwwwdddddwwwwwwawwwwwwdwwwwwwwwwdddwwwwwwddwwdwwwdwwdddwwwwwwddwwwwwwwwww";
	int path_lengh = 0;
	int path_count = 0;

	FILE *fptr;
	FILE *fptr2;
	FILE *fptr3;

	double objx[LANDMARKNUM];
	double objy[LANDMARKNUM];
	CvPoint NEW_RE[LANDMARKNUM];
	double errorX = 0;
	double errorY = 0;
	double errorDis = 0;
	int realcount = 0;
	int error_count = 0;
	float R_f[4] = {3,0,0,3};
	float I_f[4] = {1,0,0,1};
	float sigma_state_init[9]={0.001,0,0,0,0.001,0,0,0,0.001};
	struct particle_member particle[particle_number];
	struct robot_member robot;
	int sensor = 0;
	int presensor = 0;
	int Updatecounter = 0;
	int Updatecounterlimit = 1;


int main()
{
	
	CvMat *I22				= cvCreateMat(2,2,CV_32FC1);
	CvMat *R				= cvCreateMat(2,2,CV_32FC1);
	CvMat *R_inv			= cvCreateMat(2,2,CV_32FC1);
	CvMat *P				= cvCreateMat(3,3,CV_32FC1);
	CvMat *P_inv			= cvCreateMat(3,3,CV_32FC1);
	CvMat *H_landmark		= cvCreateMat(2,2,CV_32FC1);
	CvMat *H_landmark_T		= cvCreateMat(2,2,CV_32FC1);
	CvMat *H_state			= cvCreateMat(2,3,CV_32FC1);
	CvMat *H_state_T		= cvCreateMat(3,2,CV_32FC1);
	CvMat *S				= cvCreateMat(2,2,CV_32FC1);
	CvMat *S_inv			= cvCreateMat(2,2,CV_32FC1);
	CvMat *sigma_landmark	= cvCreateMat(2,2,CV_32FC1);
	CvMat *u_landmark		= cvCreateMat(2,1,CV_32FC1);
	CvMat *K				= cvCreateMat(2,2,CV_32FC1);
	CvMat *L				= cvCreateMat(2,2,CV_32FC1);
	CvMat *L_inv			= cvCreateMat(2,2,CV_32FC1);

	CvMat *temp_M23			= cvCreateMat(2,3,CV_32FC1);
	CvMat *temp_M22			= cvCreateMat(2,2,CV_32FC1);
	CvMat *temp_M21			= cvCreateMat(2,1,CV_32FC1);
	CvMat *temp_M33			= cvCreateMat(3,3,CV_32FC1);
	CvMat *temp_M32			= cvCreateMat(3,2,CV_32FC1);
	CvMat *temp_M31			= cvCreateMat(3,1,CV_32FC1);
	CvMat *temp_M12			= cvCreateMat(1,2,CV_32FC1);
	CvMat *temp_M11			= cvCreateMat(1,1,CV_32FC1);
	CvMat *z_temp			= cvCreateMat(2,1,CV_32FC1);
	CvMat *z_temp_T			= cvCreateMat(1,2,CV_32FC1);

	CvScalar Scalar1;
	
	clock_t start, finish;
	double  run_time=0;
	srand((unsigned)time(NULL));

	//////////////////////////////////////////////////存先前的點
	frame = cvLoadImage(map_name,1);
	map_x = frame->width;
	map_y = frame->height;

	for(int j = 0;j < map_y; j++)
	{
		for(int i = 0;i < map_x; i++)
		{
			Color = cvGet2D(frame,j,i);
			if((Color.val[0] <= 10) && (Color.val[1] <= 10) && (Color.val[2] <= 10) )
			{
				realo[realcount].x = i;
				realo[realcount].y = j;
				////printf("%d , %d\n\n",realo[realcount].x,realo[realcount].y);
				realcount++;
			}
		}
	}

	cvSetData(R,R_f,R->step);
	cvInvert(R,R_inv,CV_LU);
	cvSetData(I22,I_f,I22->step);
	cvSetData(P,sigma_state_init,P->step);
	cvInvert(P,P_inv,CV_LU);
	//robot結構初始化
	robot.x = 100;
	robot.y = 100;
	robot.d = 90;

	robot.z = cvCreateMat(2,1,CV_32FC1);
	///
	
	//particle結構初始化
	for(i=0;i<particle_number;i++)
	{
		particle[i].w = 1/particle_number;
		particle[i].new_w = 0;
		particle[i].N = 0;
		particle[i].new_N = 0;

		particle[i].sigma_state = cvCreateMat(3,3,CV_32FC1);
		particle[i].new_sigma_state = cvCreateMat(3,3,CV_32FC1);
		particle[i].new_u_state = cvCreateMat(3,1,CV_32FC1);
		particle[i].u_state = cvCreateMat(3,1,CV_32FC1);
		particle[i].z = cvCreateMat(2,1,CV_32FC1);
	}
	///
	
	//
	cvNamedWindow("fast_slam2.0",0);
	
	//
	
	//配置記憶體
	robot.path_x = (int *)calloc(300, sizeof(int));
	robot.path_y = (int *)calloc(300, sizeof(int));
	robot.path_d = (int *)calloc(300, sizeof(int));

	for(i=0;i<particle_number;i++)
	{
		particle[i].path_x = (int *)calloc(300, sizeof(int));
		particle[i].path_y = (int *)calloc(300, sizeof(int));
		particle[i].path_d = (int *)calloc(300, sizeof(int));
		particle[i].new_path_x = (int *)calloc(300, sizeof(int));
		particle[i].new_path_y = (int *)calloc(300, sizeof(int));
		particle[i].new_path_d = (int *)calloc(300, sizeof(int));
		particle[i].state = (double *)calloc(9000, sizeof(double));
		particle[i].new_state = (double *)calloc(9000, sizeof(double));
		for(j=0;j<100;j++)
		{
			particle[i].state[j] = 0;
			particle[i].new_state[j] = 0;
		}
	}
	
	//particle_state initial
	for(i=0;i<particle_number;i++)
	{
		particle[i].update[0] = robot.x;
		particle[i].update[1] = robot.y;
		particle[i].update[2] = robot.d;
		particle[i].w = 1.0;
	}
	////////////

	//2.0 initial
	for(i=0;i<particle_number;i++)
	{
		cvSetData(particle[i].sigma_state,sigma_state_init,particle[i].sigma_state->step);
		cvmSet(particle[i].u_state,0,0,robot.x);
		cvmSet(particle[i].u_state,1,0,robot.y);
		cvmSet(particle[i].u_state,2,0,robot.d);
	}
	/////////////
	i=0;
	path_lengh = strlen(robot_path);
	//printf("%d\n",path_lengh);
	start = clock();

	fptr3 = fopen("Landmark_estimations.csv","w");

	if((fptr2 = fopen("Kai_time.csv","w"))==NULL)
	{
		printf("file error!");
		system("pause");
	}
	while(keyboard != 'q')
	{
		Image1 = cvLoadImage(map_name,1);
		map_x = Image1->width;
		map_y = Image1->height;
		cvResizeWindow("fast_slam2.0",map_x*2,map_y*2);
		
		//機器人位置(實際)
		//printf("機器人位置:\t%d\t%d\t%d\n",robot.x,robot.y,robot.d);
		robot.coordinate=cvPoint(robot.x,map_y-robot.y);
		robot.dir=cvPoint(robot.x+round( Radius*cos(radianof(robot.d)) ),map_y-(robot.y+round( Radius*sin(radianof(robot.d))) ));
		if(i>=total_time)
		{
			robot.path_x = (int *)realloc(robot.path_x,(i+100)*sizeof(int));
			robot.path_y = (int *)realloc(robot.path_y,(i+100)*sizeof(int));
			robot.path_d = (int *)realloc(robot.path_d,(i+100)*sizeof(int));
			for(j=0;j<particle_number;j++)
			{
				particle[j].path_x = (int *)realloc(particle[j].path_x,(i+100)*sizeof(int));
				particle[j].path_y = (int *)realloc(particle[j].path_y,(i+100)*sizeof(int));
				particle[j].path_d = (int *)realloc(particle[j].path_d,(i+100)*sizeof(int));
				particle[j].new_path_x = (int *)realloc(particle[j].new_path_x,(i+100)*sizeof(int));
				particle[j].new_path_y = (int *)realloc(particle[j].new_path_y,(i+100)*sizeof(int));
				particle[j].new_path_d = (int *)realloc(particle[j].new_path_d,(i+100)*sizeof(int));
			}
			total_time += 100;
		}
		robot.path_x[i] = robot.x;
		robot.path_y[i] = robot.y;
		robot.path_d[i] = robot.d;
		//

		//雷射掃描
		laser(robot.x,robot.y,robot.d,robot.range_finder);
		//
				
		//粒子更新開始
		for(j=0;j<particle_number;j++)
		{
			particle[j].coordinate = cvPoint(particle[j].update[0],map_y - particle[j].update[1]);
			particle[j].path_x[i] = particle[j].update[0];
			particle[j].path_y[i] = particle[j].update[1];
			particle[j].path_d[i] = particle[j].update[2];
			particle[j].new_path_x[i] = particle[j].update[0];
			particle[j].new_path_y[i] = particle[j].update[1];
			particle[j].new_path_d[i] = particle[j].update[2];
			particle[j].state[0] = (double)particle[j].update[0];
			particle[j].state[1] = (double)particle[j].update[1];
			particle[j].state[2] = (double)particle[j].update[2];
			cvmSet(particle[j].u_state,0,0,(double)particle[j].update[0]);
			cvmSet(particle[j].u_state,1,0,(double)particle[j].update[1]);
			cvmSet(particle[j].u_state,2,0,(double)particle[j].update[2]);
		}
		//更新完成


		/*sensor = 0;
		for(k=0;k<181;k+=1)
		{
			if(robot.range_finder[k] < 100)
			{
				sensor++;
			}
		}*/
		////量測預測
		//if(keyboard == 'a')
		//{
		//	
		//	for(k=180;k>0;k--)
		//{
		//	if(robot.range_finder[k] < 100)
		//	{
		//		cvmSet(robot.z,0,0,robot.range_finder[k]);
		//		cvmSet(robot.z,1,0,k-90);
		//		
		//		for(j=0;j<particle_number;j++)
		//		{
		//			p = (double *)malloc(sizeof(double)*(particle[j].N+5));
		//			for(l=0;l<particle[j].N+1;l++)
		//				p[l] = 0;
		//			for(l=0;l<particle[j].N;l++)
		//			{
		//				//2.0預測
		//				cvmSet( particle[j].z,0,0, sqrt( pow((particle[j].state[2+(l)*6+1] - particle[j].state[0]),2) + pow((particle[j].state[2+(l)*6+2] - particle[j].state[1]),2) ) ); 
		//				cvmSet( particle[j].z,1,0, atan2( (particle[j].state[2+(l)*6+2] - particle[j].state[1]) , (particle[j].state[2+(l)*6+1] - particle[j].state[0]) )*180/m_pi);
		//				
		//				
		//				OpenCV_Angle_Correct(j,l);

		//				cvmSub(robot.z, particle[j].z, z_temp);
		//				cvTranspose(z_temp,z_temp_T);
		//				//PrintMatrix(particle[j].z,2,1);
		//				//system("pause");
		//				//H
		//				cvmSet(H_landmark,0,0,( (particle[j].state[2+(l)*6+1] - particle[j].state[0]) / cvmGet(particle[j].z,0,0) ) );
		//				cvmSet(H_landmark,0,1,( (particle[j].state[2+(l)*6+2] - particle[j].state[1]) / cvmGet(particle[j].z,0,0) ) );
		//				cvmSet(H_landmark,1,0,( (-1) * (particle[j].state[2+(l)*6+2] - particle[j].state[1]) / pow(cvmGet(particle[j].z,0,0),2) ) );
		//				cvmSet(H_landmark,1,1,( (particle[j].state[2+(l)*6+1] - particle[j].state[0]) / pow(cvmGet(particle[j].z,0,0),2) ) );
		//				cvTranspose(H_landmark,H_landmark_T);

		//				//S
		//				cvmSet(sigma_landmark,0,0,particle[j].state[2+l*6+3]);
		//				cvmSet(sigma_landmark,0,1,particle[j].state[2+l*6+4]);
		//				cvmSet(sigma_landmark,1,0,particle[j].state[2+l*6+5]);
		//				cvmSet(sigma_landmark,1,1,particle[j].state[2+l*6+6]);
		//				cvmMul(H_landmark,sigma_landmark,temp_M22);
		//				cvmMul(temp_M22,H_landmark_T,S);
		//				cvmAdd(S,R,S);
		//				cvInvert(S,S_inv,CV_LU);
		//				
		//				/////////////////////////////////////////////////////////////////////////////////////////////////////
		//				//H_state
		//				cvmSet(H_state,0,0,( (-1)*(particle[j].state[2+(l)*6+1] - particle[j].state[0]) / cvmGet(particle[j].z,0,0) ) );
		//				cvmSet(H_state,0,1,( (-1)*(particle[j].state[2+(l)*6+2] - particle[j].state[1]) / cvmGet(particle[j].z,0,0) ) );
		//				cvmSet(H_state,0,2,0 );
		//				cvmSet(H_state,1,0,( (particle[j].state[2+(l)*6+2] - particle[j].state[1]) / pow(cvmGet(particle[j].z,0,0),2) ) );
		//				cvmSet(H_state,1,1,( (-1) * (particle[j].state[2+(l)*6+1] - particle[j].state[0]) / pow(cvmGet(particle[j].z,0,0),2) ) );
		//				cvmSet(H_state,1,2,-1 );
		//				cvTranspose(H_state,H_state_T);
		//				
		//				//sigma_state
		//				//particle[j].sigma_state = cvCloneMat(P);
		//				//PrintMatrix(sigma_landmark,2,2);
		//				//PrintMatrix(H_landmark,2,2);
		//				//PrintMatrix(S,2,2);
		//				cvmMul(H_state_T,S_inv,temp_M32);
		//				cvInvert(particle[j].sigma_state,particle[j].sigma_state,CV_LU);
		//				cvMatMulAdd(temp_M32,H_state,particle[j].sigma_state,particle[j].sigma_state);
		//				cvInvert(particle[j].sigma_state,particle[j].sigma_state,CV_LU);
		//				//PrintMatrix(particle[j].sigma_state,3,3);
		//				//system("pause");
		//				

		//				//u_state
		//				cvmSet(particle[j].u_state,0,0,particle[j].state[0]);
		//				cvmSet(particle[j].u_state,1,0,particle[j].state[1]);
		//				cvmSet(particle[j].u_state,2,0,particle[j].state[2]);

		//				cvmMul(particle[j].sigma_state, H_state_T, temp_M32);
		//				cvmMul(temp_M32, S_inv, temp_M32);
		//				//cvMatMulAdd(temp_M32, z_temp, particle[j].u_state, particle[j].u_state);
		//				cvMatMul(temp_M32, z_temp, temp_M31);
		//				for(m=0;m<3;m++)
		//				{
		//					cvmSet(temp_M31,m,0,cvmGet(temp_M31,m,0));
		//				}
		//				cvmAdd(temp_M31,particle[j].u_state,particle[j].u_state);

		//				if(cvmGet(particle[j].u_state,2,0) >= 360)
		//					cvmSet(particle[j].u_state,2,0,(int)cvmGet(particle[j].u_state,2,0) % 360);
		//				else if(cvmGet(particle[j].u_state,2,0) < 0)
		//					cvmSet(particle[j].u_state,2,0,(int)cvmGet(particle[j].u_state,2,0) % 360);

		//				/*particle[j].state[0] = cvmGet(particle[j].u_state,0,0);
		//				particle[j].state[1] = cvmGet(particle[j].u_state,1,0);
		//				particle[j].state[2] = cvmGet(particle[j].u_state,2,0);*/

		//				//量測
		//				cvmSet( particle[j].z,0,0, sqrt( pow((particle[j].state[2+(l)*6+1] - particle[j].state[0]),2) + pow((particle[j].state[2+(l)*6+2] - particle[j].state[1]),2) ) ); 
		//				cvmSet( particle[j].z,1,0, atan2( (particle[j].state[2+(l)*6+2] - particle[j].state[1]) , (particle[j].state[2+(l)*6+1] - particle[j].state[0]) )*180/m_pi);

		//				OpenCV_Angle_Correct(j,l);


		//				cvmSub(robot.z, particle[j].z, z_temp);
		//				cvTranspose(z_temp,z_temp_T);

		//				cvmSet(H_landmark,0,0,( (particle[j].state[2+(l)*6+1] - particle[j].state[0]) / cvmGet(particle[j].z,0,0) ) );
		//				cvmSet(H_landmark,0,1,( (particle[j].state[2+(l)*6+2] - particle[j].state[1]) / cvmGet(particle[j].z,0,0) ) );
		//				cvmSet(H_landmark,1,0,( (-1) * (particle[j].state[2+(l)*6+2] - particle[j].state[1]) / pow(cvmGet(particle[j].z,0,0),2) ) );
		//				cvmSet(H_landmark,1,1,( (particle[j].state[2+(l)*6+1] - particle[j].state[0]) / pow(cvmGet(particle[j].z,0,0),2) ) );
		//				cvTranspose(H_landmark,H_landmark_T);

		//				//S
		//				cvmSet(sigma_landmark,0,0,particle[j].state[2+l*6+3]);
		//				cvmSet(sigma_landmark,0,1,particle[j].state[2+l*6+4]);
		//				cvmSet(sigma_landmark,1,0,particle[j].state[2+l*6+5]);
		//				cvmSet(sigma_landmark,1,1,particle[j].state[2+l*6+6]);
		//				cvmMul(H_landmark,sigma_landmark,temp_M22);
		//				cvmMul(temp_M22,H_landmark_T,S);
		//				cvmAdd(S,R,S);
		//				cvInvert(S,S_inv,CV_LU);
		//				//////////////////////////////////////////////////////////////////////
		//				
		//				p[l] = 1 / ( sqrt( pow(2*m_pi,2)*fabs(cvDet(S)) ) );
		//				//cvmSub(robot.z, particle[j].z, z_temp);
		//				//cvTranspose(z_temp,z_temp_T);
		//				cvmMul(z_temp_T,S_inv,temp_M12);
		//				cvmMul(temp_M12,z_temp,temp_M11);
		//				p[l] = p[l] * exp( (-0.5) * cvmGet(temp_M11,0,0) );
		//			}
		//			p[l]=p0;

		//			//計算n_head = arg max p[]
		//			for(l=1,n_head=0,p_temp=p[0];l<particle[j].N+1;l++)
		//			{
		//				if(p[l] > p_temp)
		//				{
		//					n_head = l;
		//					p_temp = p[l];
		//				}
		//			}

		//			if(n_head < particle[j].N)	//更新
		//			{
		//			
		//				if(sensor - presensor == 0)
		//				{
		//						break;
		//				}

		//				particle[j].N = particle[j].N;						
		//				
		//				cvmSet( particle[j].z,0,0, sqrt( pow((particle[j].state[2+(n_head)*6+1] - particle[j].state[0]),2) + pow((particle[j].state[2+(n_head)*6+2] - particle[j].state[1]),2) ) ); 
		//				cvmSet( particle[j].z,1,0, atan2( (particle[j].state[2+(n_head)*6+2] - particle[j].state[1]) , (particle[j].state[2+(n_head)*6+1] - particle[j].state[0]) )*180/m_pi);

		//				OpenCV_Angle_Correct(j,n_head);

		//				cvmSub(robot.z, particle[j].z, z_temp);
		//				cvTranspose(z_temp,z_temp_T);

		//				//H
		//				cvmSet(H_landmark,0,0,( (particle[j].state[2+(n_head)*6+1] - particle[j].state[0]) / cvmGet(particle[j].z,0,0) ) );
		//				cvmSet(H_landmark,0,1,( (particle[j].state[2+(n_head)*6+2] - particle[j].state[1]) / cvmGet(particle[j].z,0,0) ) );
		//				cvmSet(H_landmark,1,0,( (-1) * (particle[j].state[2+(n_head)*6+2] - particle[j].state[1]) / pow(cvmGet(particle[j].z,0,0),2) ) );
		//				cvmSet(H_landmark,1,1,( (particle[j].state[2+(n_head)*6+1] - particle[j].state[0]) / pow(cvmGet(particle[j].z,0,0),2) ) );
		//				cvTranspose(H_landmark,H_landmark_T);

		//				//S
		//				cvmSet(sigma_landmark,0,0,particle[j].state[2+(n_head)*6+3]);
		//				cvmSet(sigma_landmark,0,1,particle[j].state[2+(n_head)*6+4]);
		//				cvmSet(sigma_landmark,1,0,particle[j].state[2+(n_head)*6+5]);
		//				cvmSet(sigma_landmark,1,1,particle[j].state[2+(n_head)*6+6]);
		//				cvmMul(H_landmark,sigma_landmark,temp_M22);
		//				cvMatMulAdd(temp_M22,H_landmark_T,R,S);
		//				cvInvert(S,S_inv,CV_LU);

		//				//K
		//				cvmMul(sigma_landmark,H_landmark_T,temp_M22);
		//				cvmMul(temp_M22,S_inv,K);

		//				cvmMul(K,z_temp,temp_M21);
		//				particle[j].state[2+(n_head)*6+1] += cvmGet(temp_M21,0,0)*0.001;
		//				particle[j].state[2+(n_head)*6+2] += cvmGet(temp_M21,1,0)*0.001;

		//				//H_state
		//				cvmSet(H_state,0,0,( (-1)*(particle[j].state[2+(n_head)*6+1] - particle[j].state[0]) / cvmGet(particle[j].z,0,0) ) );
		//				cvmSet(H_state,0,1,( (-1)*(particle[j].state[2+(n_head)*6+2] - particle[j].state[1]) / cvmGet(particle[j].z,0,0) ) );
		//				cvmSet(H_state,0,2,0 );
		//				cvmSet(H_state,1,0,( (particle[j].state[2+(n_head)*6+2] - particle[j].state[1]) / pow(cvmGet(particle[j].z,0,0),2) ) );
		//				cvmSet(H_state,1,1,( (-1) * (particle[j].state[2+(n_head)*6+1] - particle[j].state[0]) / pow(cvmGet(particle[j].z,0,0),2) ) );
		//				cvmSet(H_state,1,2,-1 );
		//				cvTranspose(H_state,H_state_T);

		//				//L
		//				cvmMul(H_state,P,temp_M23);
		//				cvmMul(temp_M23,H_state_T,temp_M22);
		//				cvmAdd(temp_M22,S,L);
		//				cvInvert(L,L_inv,CV_LU);

		//				//w
		//				particle[j].w = particle[j].w * (1 / sqrt(pow(2*m_pi,2)*fabs(cvDet(L)) ) ) ;
		//				//particle[j].w =  (1 / sqrt(pow(2*m_pi,2)*fabs(cvDet(L)) ) ) ;
		//				cvmMul(z_temp_T,L_inv,temp_M12);
		//				cvmMul(temp_M12,z_temp,temp_M11);
		//				particle[j].w = particle[j].w * exp( (-0.5) * cvmGet(temp_M11,0,0) );
		//				//particle[j].w = particle[j].w * p_temp;

		//				//sigma_landmark
		//				cvmMul(K,H_landmark,temp_M22);
		//				cvmSub(I22,temp_M22,temp_M22);
		//				cvmMul(temp_M22,sigma_landmark,sigma_landmark);

		//				particle[j].state[2+n_head*6+3] = cvmGet(sigma_landmark,0,0);
		//				particle[j].state[2+n_head*6+4] = cvmGet(sigma_landmark,0,1);
		//				particle[j].state[2+n_head*6+5] = cvmGet(sigma_landmark,1,0);
		//				particle[j].state[2+n_head*6+6] = cvmGet(sigma_landmark,1,1);
		//			}
		//			else	//新增
		//			{
		//				particle[j].N = particle[j].N + 1;
		//				
		//				//u
		//				particle[j].state[2+(particle[j].N - 1)*6+1] = particle[j].state[0] + robot.range_finder[k]*cos(radianof((double)(k-90+particle[j].state[2])));
		//				particle[j].state[2+(particle[j].N - 1)*6+2] = particle[j].state[1] + robot.range_finder[k]*sin(radianof((double)(k-90+particle[j].state[2])));
		//				
		//				//H
		//				cvmSet(H_landmark,0,0,( robot.range_finder[k]*cos(radianof((double)(k-90+particle[j].state[2]))) / robot.range_finder[k] ) );
		//				cvmSet(H_landmark,0,1,( robot.range_finder[k]*sin(radianof((double)(k-90+particle[j].state[2]))) / robot.range_finder[k] ) );
		//				cvmSet(H_landmark,1,0,( (-1)*robot.range_finder[k]*sin(radianof((double)(k-90+particle[j].state[2]))) / pow((double)robot.range_finder[k],2) ) );
		//				cvmSet(H_landmark,1,1,( robot.range_finder[k]*cos(radianof((double)(k-90+particle[j].state[2]))) / pow((double)robot.range_finder[k],2) ) );
		//				cvTranspose(H_landmark,H_landmark_T);

		//				cvmMul(H_landmark_T,R_inv,temp_M22);
		//				cvmMul(temp_M22,H_landmark,sigma_landmark);
		//				cvInvert(sigma_landmark,sigma_landmark,CV_LU);

		//				particle[j].w *= p0;
		//				particle[j].state[2+(particle[j].N - 1)*6+3] = cvmGet(sigma_landmark,0,0);
		//				particle[j].state[2+(particle[j].N - 1)*6+4] = cvmGet(sigma_landmark,0,1);
		//				particle[j].state[2+(particle[j].N - 1)*6+5] = cvmGet(sigma_landmark,1,0);
		//				particle[j].state[2+(particle[j].N - 1)*6+6] = cvmGet(sigma_landmark,1,1);
		//			}
		//			free(p);
		//		}
		//	}
		//}

		//}else
		//{
		//	
		for(k=0;k<181;k+=1)
		{
			if(robot.range_finder[k] < 100)
			{	
				cvmSet(robot.z,0,0,robot.range_finder[k]);
				cvmSet(robot.z,1,0,k-90);
				for(j=0;j<particle_number;j++)
				{
					p = (double *)malloc(sizeof(double)*(particle[j].N+5));
					for(l=0;l<particle[j].N+1;l++)
						p[l] = 0;
					for(l=0;l<particle[j].N;l++)
					{
						//2.0預測
						cvmSet( particle[j].z,0,0, sqrt( pow((particle[j].state[2+(l)*6+1] - particle[j].state[0]),2) + pow((particle[j].state[2+(l)*6+2] - particle[j].state[1]),2) ) ); 
						cvmSet( particle[j].z,1,0, atan2( (particle[j].state[2+(l)*6+2] - particle[j].state[1]) , (particle[j].state[2+(l)*6+1] - particle[j].state[0]) )*180/m_pi);
						
						
						OpenCV_Angle_Correct(j,l);

						cvmSub(robot.z, particle[j].z, z_temp);
						cvTranspose(z_temp,z_temp_T);
						//PrintMatrix(particle[j].z,2,1);
						//system("pause");
						//H
						cvmSet(H_landmark,0,0,( (particle[j].state[2+(l)*6+1] - particle[j].state[0]) / cvmGet(particle[j].z,0,0) ) );
						cvmSet(H_landmark,0,1,( (particle[j].state[2+(l)*6+2] - particle[j].state[1]) / cvmGet(particle[j].z,0,0) ) );
						cvmSet(H_landmark,1,0,( (-1) * (particle[j].state[2+(l)*6+2] - particle[j].state[1]) / pow(cvmGet(particle[j].z,0,0),2) ) );
						cvmSet(H_landmark,1,1,( (particle[j].state[2+(l)*6+1] - particle[j].state[0]) / pow(cvmGet(particle[j].z,0,0),2) ) );
						cvTranspose(H_landmark,H_landmark_T);

						//S
						cvmSet(sigma_landmark,0,0,particle[j].state[2+l*6+3]);
						cvmSet(sigma_landmark,0,1,particle[j].state[2+l*6+4]);
						cvmSet(sigma_landmark,1,0,particle[j].state[2+l*6+5]);
						cvmSet(sigma_landmark,1,1,particle[j].state[2+l*6+6]);
						cvmMul(H_landmark,sigma_landmark,temp_M22);
						cvmMul(temp_M22,H_landmark_T,S);
						cvmAdd(S,R,S);
						cvInvert(S,S_inv,CV_LU);
						
						///////////////////////////////////////////////////////////////////////////////////////////////////////
						//H_state
						cvmSet(H_state,0,0,( (-1)*(particle[j].state[2+(l)*6+1] - particle[j].state[0]) / cvmGet(particle[j].z,0,0) ) );
						cvmSet(H_state,0,1,( (-1)*(particle[j].state[2+(l)*6+2] - particle[j].state[1]) / cvmGet(particle[j].z,0,0) ) );
						cvmSet(H_state,0,2,0 );
						cvmSet(H_state,1,0,( (particle[j].state[2+(l)*6+2] - particle[j].state[1]) / pow(cvmGet(particle[j].z,0,0),2) ) );
						cvmSet(H_state,1,1,( (-1) * (particle[j].state[2+(l)*6+1] - particle[j].state[0]) / pow(cvmGet(particle[j].z,0,0),2) ) );
						cvmSet(H_state,1,2,-1 );
						cvTranspose(H_state,H_state_T);
						
						//sigma_state
						//particle[j].sigma_state = cvCloneMat(P);
						//PrintMatrix(sigma_landmark,2,2);
						//PrintMatrix(H_landmark,2,2);
						//PrintMatrix(S,2,2);
						cvmMul(H_state_T,S_inv,temp_M32);
						cvInvert(particle[j].sigma_state,particle[j].sigma_state,CV_LU);
						cvMatMulAdd(temp_M32,H_state,particle[j].sigma_state,particle[j].sigma_state);
						cvInvert(particle[j].sigma_state,particle[j].sigma_state,CV_LU);
						//PrintMatrix(particle[j].sigma_state,3,3);
						//system("pause");
						

						//u_state
						cvmSet(particle[j].u_state,0,0,particle[j].state[0]);
						cvmSet(particle[j].u_state,1,0,particle[j].state[1]);
						cvmSet(particle[j].u_state,2,0,particle[j].state[2]);

						cvmMul(particle[j].sigma_state, H_state_T, temp_M32);
						cvmMul(temp_M32, S_inv, temp_M32);
						//cvMatMulAdd(temp_M32, z_temp, particle[j].u_state, particle[j].u_state);
						cvMatMul(temp_M32, z_temp, temp_M31);
						for(m=0;m<3;m++)
						{
							cvmSet(temp_M31,m,0,cvmGet(temp_M31,m,0));
						}
						cvmAdd(temp_M31,particle[j].u_state,particle[j].u_state);

						if(cvmGet(particle[j].u_state,2,0) >= 360)
							cvmSet(particle[j].u_state,2,0,(int)cvmGet(particle[j].u_state,2,0) % 360);
						else if(cvmGet(particle[j].u_state,2,0) < 0)
							cvmSet(particle[j].u_state,2,0,(int)cvmGet(particle[j].u_state,2,0) % 360);

						/*particle[j].state[0] = cvmGet(particle[j].u_state,0,0);
						particle[j].state[1] = cvmGet(particle[j].u_state,1,0);
						particle[j].state[2] = cvmGet(particle[j].u_state,2,0);*/

						//量測
						cvmSet( particle[j].z,0,0, sqrt( pow((particle[j].state[2+(l)*6+1] - particle[j].state[0]),2) + pow((particle[j].state[2+(l)*6+2] - particle[j].state[1]),2) ) ); 
						cvmSet( particle[j].z,1,0, atan2( (particle[j].state[2+(l)*6+2] - particle[j].state[1]) , (particle[j].state[2+(l)*6+1] - particle[j].state[0]) )*180/m_pi);

						OpenCV_Angle_Correct(j,l);


						cvmSub(robot.z, particle[j].z, z_temp);
						cvTranspose(z_temp,z_temp_T);

						cvmSet(H_landmark,0,0,( (particle[j].state[2+(l)*6+1] - particle[j].state[0]) / cvmGet(particle[j].z,0,0) ) );
						cvmSet(H_landmark,0,1,( (particle[j].state[2+(l)*6+2] - particle[j].state[1]) / cvmGet(particle[j].z,0,0) ) );
						cvmSet(H_landmark,1,0,( (-1) * (particle[j].state[2+(l)*6+2] - particle[j].state[1]) / pow(cvmGet(particle[j].z,0,0),2) ) );
						cvmSet(H_landmark,1,1,( (particle[j].state[2+(l)*6+1] - particle[j].state[0]) / pow(cvmGet(particle[j].z,0,0),2) ) );
						cvTranspose(H_landmark,H_landmark_T);

						//S
						cvmSet(sigma_landmark,0,0,particle[j].state[2+l*6+3]);
						cvmSet(sigma_landmark,0,1,particle[j].state[2+l*6+4]);
						cvmSet(sigma_landmark,1,0,particle[j].state[2+l*6+5]);
						cvmSet(sigma_landmark,1,1,particle[j].state[2+l*6+6]);
						cvmMul(H_landmark,sigma_landmark,temp_M22);
						cvmMul(temp_M22,H_landmark_T,S);
						cvmAdd(S,R,S);
						cvInvert(S,S_inv,CV_LU);
						////////////////////////////////////////////////////////////////////////
						
						p[l] = 1 / ( sqrt( pow(2*m_pi,2)*fabs(cvDet(S)) ) );
						cvmSub(robot.z, particle[j].z, z_temp);
						cvTranspose(z_temp,z_temp_T);
						cvmMul(z_temp_T,S_inv,temp_M12);
						cvmMul(temp_M12,z_temp,temp_M11);
						p[l] = p[l] * exp( (-0.5) * cvmGet(temp_M11,0,0) );
					}
					p[l]=p0;

					//計算n_head = arg max p[]
					for(l=1,n_head=0,p_temp=p[0];l<particle[j].N+1;l++)
					{
						if(p[l] > p_temp)
						{
							n_head = l;
							p_temp = p[l];
						}
					}

					if(n_head < particle[j].N)	//更新
					{
						
						/*if(sensor - presensor == 0)
						{
								break;	
						}*/

						particle[j].N = particle[j].N;						
						
						cvmSet( particle[j].z,0,0, sqrt( pow((particle[j].state[2+(n_head)*6+1] - particle[j].state[0]),2) + pow((particle[j].state[2+(n_head)*6+2] - particle[j].state[1]),2) ) ); 
						cvmSet( particle[j].z,1,0, atan2( (particle[j].state[2+(n_head)*6+2] - particle[j].state[1]) , (particle[j].state[2+(n_head)*6+1] - particle[j].state[0]) )*180/m_pi);

						OpenCV_Angle_Correct(j,n_head);

						cvmSub(robot.z, particle[j].z, z_temp);
						cvTranspose(z_temp,z_temp_T);

						//H
						cvmSet(H_landmark,0,0,( (particle[j].state[2+(n_head)*6+1] - particle[j].state[0]) / cvmGet(particle[j].z,0,0) ) );
						cvmSet(H_landmark,0,1,( (particle[j].state[2+(n_head)*6+2] - particle[j].state[1]) / cvmGet(particle[j].z,0,0) ) );
						cvmSet(H_landmark,1,0,( (-1) * (particle[j].state[2+(n_head)*6+2] - particle[j].state[1]) / pow(cvmGet(particle[j].z,0,0),2) ) );
						cvmSet(H_landmark,1,1,( (particle[j].state[2+(n_head)*6+1] - particle[j].state[0]) / pow(cvmGet(particle[j].z,0,0),2) ) );
						cvTranspose(H_landmark,H_landmark_T);

						//S
						cvmSet(sigma_landmark,0,0,particle[j].state[2+(n_head)*6+3]);
						cvmSet(sigma_landmark,0,1,particle[j].state[2+(n_head)*6+4]);
						cvmSet(sigma_landmark,1,0,particle[j].state[2+(n_head)*6+5]);
						cvmSet(sigma_landmark,1,1,particle[j].state[2+(n_head)*6+6]);
						cvmMul(H_landmark,sigma_landmark,temp_M22);
						cvMatMulAdd(temp_M22,H_landmark_T,R,S);
						cvInvert(S,S_inv,CV_LU);

						//K
						cvmMul(sigma_landmark,H_landmark_T,temp_M22);
						cvmMul(temp_M22,S_inv,K);

						cvmMul(K,z_temp,temp_M21);
						particle[j].state[2+(n_head)*6+1] += cvmGet(temp_M21,0,0)*0.001;
						particle[j].state[2+(n_head)*6+2] += cvmGet(temp_M21,1,0)*0.001;

						//H_state
						cvmSet(H_state,0,0,( (-1)*(particle[j].state[2+(n_head)*6+1] - particle[j].state[0]) / cvmGet(particle[j].z,0,0) ) );
						cvmSet(H_state,0,1,( (-1)*(particle[j].state[2+(n_head)*6+2] - particle[j].state[1]) / cvmGet(particle[j].z,0,0) ) );
						cvmSet(H_state,0,2,0 );
						cvmSet(H_state,1,0,( (particle[j].state[2+(n_head)*6+2] - particle[j].state[1]) / pow(cvmGet(particle[j].z,0,0),2) ) );
						cvmSet(H_state,1,1,( (-1) * (particle[j].state[2+(n_head)*6+1] - particle[j].state[0]) / pow(cvmGet(particle[j].z,0,0),2) ) );
						cvmSet(H_state,1,2,-1 );
						cvTranspose(H_state,H_state_T);

						//L
						cvmMul(H_state,P,temp_M23);
						cvmMul(temp_M23,H_state_T,temp_M22);
						cvmAdd(temp_M22,S,L);
						cvInvert(L,L_inv,CV_LU);

						//w
						particle[j].w = particle[j].w * (1 / sqrt(pow(2*m_pi,2)*fabs(cvDet(L)) ) ) ;
						//particle[j].w =  (1 / sqrt(pow(2*m_pi,2)*fabs(cvDet(L)) ) ) ;
						cvmMul(z_temp_T,L_inv,temp_M12);
						cvmMul(temp_M12,z_temp,temp_M11);
						particle[j].w = particle[j].w * exp( (-0.5) * cvmGet(temp_M11,0,0) );
						//particle[j].w = particle[j].w * p_temp;

						//sigma_landmark
						cvmMul(K,H_landmark,temp_M22);
						cvmSub(I22,temp_M22,temp_M22);
						cvmMul(temp_M22,sigma_landmark,sigma_landmark);

						particle[j].state[2+n_head*6+3] = cvmGet(sigma_landmark,0,0);
						particle[j].state[2+n_head*6+4] = cvmGet(sigma_landmark,0,1);
						particle[j].state[2+n_head*6+5] = cvmGet(sigma_landmark,1,0);
						particle[j].state[2+n_head*6+6] = cvmGet(sigma_landmark,1,1);

						


					}
					else	//新增
					{
						particle[j].N = particle[j].N + 1;
						
						//u
						particle[j].state[2+(particle[j].N - 1)*6+1] = particle[j].state[0] + robot.range_finder[k]*cos(radianof((double)(k-90+particle[j].state[2])));
						particle[j].state[2+(particle[j].N - 1)*6+2] = particle[j].state[1] + robot.range_finder[k]*sin(radianof((double)(k-90+particle[j].state[2])));
						
						//H
						cvmSet(H_landmark,0,0,( robot.range_finder[k]*cos(radianof((double)(k-90+particle[j].state[2]))) / robot.range_finder[k] ) );
						cvmSet(H_landmark,0,1,( robot.range_finder[k]*sin(radianof((double)(k-90+particle[j].state[2]))) / robot.range_finder[k] ) );
						cvmSet(H_landmark,1,0,( (-1)*robot.range_finder[k]*sin(radianof((double)(k-90+particle[j].state[2]))) / pow((double)robot.range_finder[k],2) ) );
						cvmSet(H_landmark,1,1,( robot.range_finder[k]*cos(radianof((double)(k-90+particle[j].state[2]))) / pow((double)robot.range_finder[k],2) ) );
						cvTranspose(H_landmark,H_landmark_T);

						cvmMul(H_landmark_T,R_inv,temp_M22);
						cvmMul(temp_M22,H_landmark,sigma_landmark);
						cvInvert(sigma_landmark,sigma_landmark,CV_LU);

						particle[j].w *= p0;
						particle[j].state[2+(particle[j].N - 1)*6+3] = cvmGet(sigma_landmark,0,0);
						particle[j].state[2+(particle[j].N - 1)*6+4] = cvmGet(sigma_landmark,0,1);
						particle[j].state[2+(particle[j].N - 1)*6+5] = cvmGet(sigma_landmark,1,0);
						particle[j].state[2+(particle[j].N - 1)*6+6] = cvmGet(sigma_landmark,1,1);
					}
					free(p);
					//printf("Updatecounter = %d\n\n",Updatecounter);
				}
			}
		}
		//}
		//presensor = sensor;

		//權重正規化
		sum_w = 0;
		for(j=0;j<particle_number;j++)
			sum_w += particle[j].w;

		for(j=0;j<particle_number;j++)
		{
			particle[j].w = particle[j].w / sum_w;
		}
		//計算ESS
		sum_w = 0;
		ESS = 0;
		for(j=0;j<particle_number;j++)
			sum_w += pow((particle_number * particle[j].w - 1),2);
		sum_w /= particle_number;
		ESS = particle_number / (1 + sum_w);
		//Updatecounterlimit = 10 - ESS/3;
		

		//printf("ESS = %Lf\n",ESS);
		if(ESS <= Ess_mun)  //是否重新取樣
		{
			for(j=0;j<particle_number;j++)
			{
				for(m=0;m<resample_size;m++)
				{
					index[m] = rand()%particle_number;
					w_temp[m] = particle[index[m]].w;
					for(l=0;l<m;l++)
					{
						if(index[m] == index[l])
						{
							m--;
							break;
						}
					}
				}

				//找出權重最大的
				sum_w = w_temp[0];
				k = 0;
				for(m=1;m<resample_size;m++)
				{
					if(w_temp[m] >= sum_w)
					{
						sum_w = w_temp[m];
						k = m;
					}
				}

				for(m=0;m< (3+ particle[index[k]].N *6 );m++)
					particle[j].new_state[m] = particle[index[k]].state[m];

				for(m=0;m<i;m++)
				{
					particle[j].new_path_x[m] = particle[index[k]].path_x[m];
					particle[j].new_path_y[m] = particle[index[k]].path_y[m];
					particle[j].new_path_d[m] = particle[index[k]].path_d[m];
				}
				particle[j].new_N = particle[index[k]].N;
				particle[j].new_w = particle[index[k]].w;
				particle[j].new_sigma_state = cvCloneMat(particle[index[k]].sigma_state);
				particle[j].new_u_state = cvCloneMat(particle[index[k]].u_state);
			}
			for(j=0;j<particle_number;j++)
			{
				for(m=0;m< (3+ particle[j].new_N *6 );m++)
					particle[j].state[m] = particle[j].new_state[m];
				
				for(m=0;m<i;m++)
				{
					particle[j].path_x[m] = particle[j].new_path_x[m];
					particle[j].path_y[m] = particle[j].new_path_y[m];
					particle[j].path_d[m] = particle[j].new_path_d[m];
				}
				particle[j].N = particle[j].new_N;
				particle[j].w = particle[j].new_w;
				particle[j].sigma_state = cvCloneMat(particle[j].new_sigma_state);
				particle[j].u_state = cvCloneMat(particle[j].new_u_state);
			}
			//權重正規化
			sum_w = 0;
			for(j=0;j<particle_number;j++)
				sum_w += particle[j].w;

			for(j=0;j<particle_number;j++)
				particle[j].w = particle[j].w / sum_w;
		}

		///////////////////////////////畫圖區/////////////////////////////////////////////////////
		//ImageSize1 = cvSize(map_x,map_y);
		////Image1 = cvCreateImage(ImageSize1, IPL_DEPTH_8U, 3);
		//Image1 = cvLoadImage("map.bmp",1);


		//畫機器人(實際)
		Color=CV_RGB(150,150,0);
		cvCircle(Image1,robot.coordinate,Radius,Color,1,CV_AA,0);
		cvLine(Image1,robot.coordinate,robot.dir,Color,1,CV_AA,0);
		//
		//畫機器人的雷射
		Color=CV_RGB(0,0,255);
		for(j=0;j<181;j++)
		{
			degree = robot.d - 90 + j;
			robot.laser_line = cvPoint(robot.x+round( robot.range_finder[j]*cos(radianof(degree)) ),map_y-(robot.y+round( robot.range_finder[j]*sin(radianof(degree)) ) ) );
			if(j == 0 || j == 180)
			{
				cvLine(Image1,robot.coordinate,robot.laser_line,Color,1,CV_AA,0);
			}else
			{
				cvCircle(Image1,robot.laser_line,1,Color,-1,CV_AA,0);
			}
		}

		//
		//畫機器人軌跡(真實)
		Color=CV_RGB(255,0,255);
		for(j=0;j<=i;j++)
		{
			robot.true_path = cvPoint( robot.path_x[j],(map_y - robot.path_y[j]));
			cvCircle(Image1,robot.true_path,1,Color,-1,CV_AA,0);	
		}
		//
		//畫粒子
		Color=CV_RGB(0,255,0);
		for(j=0;j<particle_number;j++)
		{
			cvCircle(Image1,particle[j].coordinate,2,Color,-1,CV_AA,0);
		}
		//找出權重最高
		sum_w = particle[0].w;
		k = 0;
		for(j=1;j<particle_number;j++)
		{
			if(particle[j].w > sum_w)
			{
				k = j;
				sum_w = particle[j].w;
			}
		}
		//PrintMatrix(particle[k].sigma_state,3,3);
		////畫粒子歷史軌跡
		Color=CV_RGB(100,100,100);
		for(j=0;j<=i;j++)
		{
			particle[k].path = cvPoint(particle[k].path_x[j],map_y - particle[k].path_y[j]);
			cvCircle(Image1,particle[k].path,1,Color,-1,CV_AA,0);
		}
		Color=CV_RGB(255,0,0);
		//printf("partical pose = %.5f\t%.5f\t%.5f\n",particle[k].state[0],particle[k].state[1],particle[k].state[2]);
		//printf("partical[%d] landmark number = %d\n",k,particle[k].N);
		for(j=0;j<particle[k].N;j++)
		{
			//printf("landmark[%d] pose = %Lf,%Lf\n",j+1,particle[k].state[3+j*6],map_y - particle[k].state[4+j*6]);
			particle[k].map = cvPoint(round(particle[k].state[2+j*6+1]) , map_y - round(particle[k].state[2+j*6+2]));
			cvCircle(Image1,particle[k].map,3,Color,1,CV_AA,0);
			if(particle[k].N >= 15)
				fprintf(fptr3,"%.3f,%.3f\n",particle[k].state[2+j*6+1],map_y - particle[k].state[2+j*6+2]);
		}

		cvShowImage("fast_slam2.0",Image1);
		if(!(i%PERTIME))
		{
			if(path_count >= path_lengh)
			{
				if((fptr = fopen("path2_0.csv","w"))==NULL)
					printf("file error!");
				else
				{
					finish = clock();
					run_time = (double)(finish - start) / CLOCKS_PER_SEC;
					/*fprintf(fptr,"j,robot_x,robot_y,robot_d,p_x,p_y,p_d\n");
					for(j=0;j<=i;j+=5)
					{
						fprintf(fptr,"%d,%d,%d,%d,%d,%d,%d\n",j,robot.path_x[j],robot.path_y[j],robot.path_d[j],particle[k].path_x[j],particle[k].path_y[j],particle[k].path_d[j]);
					}*/
					fprintf(fptr,"run_time,%.5f\n\n",run_time);
					fprintf(fptr,"R_Dx,R_Dy,R_Ddis\n");
					for(j = 0 ; j <= i ; j += PERTIME)
					{
						errorX += abs(particle[k].path_x[j] - robot.path_x[j]);
						errorY += abs(particle[k].path_y[j] - robot.path_y[j]);
						errorDis += sqrt(long double(particle[k].path_x[j] - robot.path_x[j])*(particle[k].path_x[j] - robot.path_x[j])+(particle[k].path_y[j] - robot.path_y[j])*(particle[k].path_y[j] - robot.path_y[j]));
					}
					errorX = errorX/(j/PERTIME+1);
					errorY = errorY/(j/PERTIME+1);
					errorDis = errorDis/(j/PERTIME+1);
					fprintf(fptr,"%.3f,%.3f,%.3f\n",errorX,errorY,errorDis);

					//fprintf(fptr,"\nlandmark pos:\ni,x,y\n");
					for(j=0;j<particle[k].N;j++)
					{
						//fprintf(fptr,"%d,%.3f,%.3f\n",j,particle[k].state[j*6+3],map_y - particle[k].state[j*6+4]);
						objx[j] = particle[k].state[j*6+3];
						objy[j] = map_y - particle[k].state[j*6+4];
					}

					for(int i = 0; i < LANDMARKNUM;i++)
					{
						for(int j = 0; j < LANDMARKNUM;j++)
						{
							if( abs(objx[i] - realo[j].x) < RER)
							{
								if( abs(objy[i] - realo[j].y) < RER)
								{
									NEW_RE[i].x = realo[j].x;
									NEW_RE[i].y = realo[j].y;
									break;
								}
							}
						}
					}
					errorX = 0;
					errorY = 0;
					errorDis = 0; 
					fprintf(fptr,"dx,dy,dDis\n");
					for(int i = 0;i < LANDMARKNUM;i++)
					{
						errorDis = sqrt((objx[i] - NEW_RE[i].x)*(objx[i] - NEW_RE[i].x) + (objy[i] - NEW_RE[i].y)*(objy[i] - NEW_RE[i].y));
						errorX = abs(objx[i] - NEW_RE[i].x);
						errorY = abs(objy[i] - NEW_RE[i].y);
						fprintf(fptr,"%f,%f,%f\n",errorX,errorY,errorDis);
			
					}
					errorX = 0;
					errorY = 0;
					errorDis = 0; 
					for(int i = 0;i < LANDMARKNUM;i++)
					{
						errorDis += sqrt((objx[i] - NEW_RE[i].x)*(objx[i] - NEW_RE[i].x) + (objy[i] - NEW_RE[i].y)*(objy[i] - NEW_RE[i].y));
						errorX += abs(objx[i] - NEW_RE[i].x);
						errorY += abs(objy[i] - NEW_RE[i].y);
						error_count++;
					}
					errorX = errorX / error_count;
					errorY = errorY / error_count;
					errorDis = errorDis / error_count;
		
					fprintf(fptr,"avedx,avedy,avedDis\n");
					fprintf(fptr,"%f,%f,%f\n",errorX,errorY,errorDis);
							
				}
				fclose(fptr);

				//if((fptr2 = fopen("particle_state.csv","w"))==NULL)
				//	printf("file error!");
				//else
				//{
				//	for(j=0;j<(3+ particle[k].N *6 );j++)
				//	{
				//		fprintf(fptr2,"%.3f,",particle[k].state[j]);
				//	}
				//}
				fclose(fptr2);
				system("pause");
			}
			else
			{
				keyboard = robot_path[path_count++];
				for(j=0;j<particle_number;j++)
				{
					particle[j].sigma_state = cvCloneMat(P);
				}

				finish = clock();
				run_time = (double)(finish - start) / CLOCKS_PER_SEC;
				fprintf(fptr2,"run_time,%.5f\n",run_time);
			}
			
		}
		else
			keyboard = 'x';
		cvWaitKey(10);
		system("cls");
		cvReleaseImage(&Image1);
		//機器人移動
		switch (keyboard)
		{
			case 'w':
				robot.x = robot.x+ round(10*cos(radianof(robot.d)) );//+ 1.5*guassian(0,0.1) );
				robot.y = robot.y+ round(10*sin(radianof(robot.d)) );//+ 1.5*guassian(0,0.1) );
				for(j=0;j<particle_number;j++)
				{
					particle[j].update[0] =round( particle[j].state[0]+ 10*cos(radianof(particle[j].state[2])) + guassian(0,1)) ;
					particle[j].update[1] =round( particle[j].state[1]+ 10*sin(radianof(particle[j].state[2])) + guassian(0,1)) ;
					particle[j].update[2] = round( particle[j].state[2] );
				}
				break;
			case 's':
				robot.x = robot.x-round( 10*cos(radianof(robot.d)) );
				robot.y = robot.y-round( 10*sin(radianof(robot.d)) );
				for(j=0;j<particle_number;j++)
				{
					particle[j].update[0] =round( particle[j].state[0]- 10*cos(radianof(particle[j].state[2])) + guassian(0,1) );
					particle[j].update[1] =round( particle[j].state[1]- 10*sin(radianof(particle[j].state[2])) + guassian(0,1) );
					particle[j].update[2] = round( particle[j].state[2] );
				}
				break;
			case 'a':
				robot.d = robot.d + 15 ;//+ round(guassian(0,0.1));
				if(robot.d>=360)
					robot.d -= 360;
				for(j=0;j<particle_number;j++)
				{
					particle[j].update[0] = round( particle[j].state[0] );
					particle[j].update[1] = round( particle[j].state[1] );
					particle[j].update[2] = round(particle[j].state[2] + 15 + guassian(0,1));
					if(particle[j].update[2]>=360)
						particle[j].update[2] -= 360;
				}
				break;
			case 'd':
				robot.d = robot.d - 15;
				if(robot.d < 0)
					robot.d += 360;
				for(j=0;j<particle_number;j++)
				{
					particle[j].update[0] = round( particle[j].state[0] );
					particle[j].update[1] = round( particle[j].state[1] );
					particle[j].update[2] = round(particle[j].state[2] - 15 + guassian(0,1));
					if(particle[j].update[2] < 0)
						particle[j].update[2] += 360;
				}
				break;
			default:
				for(j=0;j<particle_number;j++)
				{
					particle[j].update[0] = round( particle[j].state[0] );
					particle[j].update[1] = round( particle[j].state[1] );
					particle[j].update[2] = round( particle[j].state[2] );
				}
				break;
		}

		i++;
	}


	cvDestroyWindow("fast_slam2.0");
	return 0;
}


void laser(int x, int y, int d, int range_finder[])
{
	int degree;
	int i=0,j=0;
	int obj_x=0,obj_y=0;
	CvScalar Scalar1;
	//雷射掃描儀
	for(i=0;i<181;i++)
	{
		range_finder[i] = 100;
		for(j=1;j<=100;j++)
		{
			degree = d-90+i;
			obj_x = x + round(j*cos(radianof(degree)));
			obj_y = y + round(j*sin(radianof(degree)));
			
			if( (obj_x <= 0)||(obj_y<=0)||(obj_x > 1000)||(obj_y >1000) )
			{
				range_finder[i] = 100;
				break;
			}
			Scalar1 = cvGet2D(Image1,map_y-obj_y,obj_x );
			if( (Scalar1.val[0] <= 10) &&
				(Scalar1.val[1] <= 10) &&
				(Scalar1.val[2] <= 10) )
			{
				range_finder[i] = j;
				break;
			}
		}
	}

}

void PrintMatrix(CvMat *Matrix,int Rows,int Cols)
{
    int i,j;
	for(i=0;i<Rows;i++)
    {
        for(j=0;j<Cols;j++)
        {
            printf("%.6f ",cvGet2D(Matrix,i,j).val[0]);
        }
        printf("\n");
    }
}

void OpenCV_Angle_Correct(int particle_num,int particle_N)
{
	
	if( (cvmGet(particle[particle_num].z,1,0)) < 0 )
	{
		cvmSet( particle[particle_num].z,1,0,cvmGet(particle[particle_num].z,1,0) + 360);
	}
	

	if( (cvmGet(particle[particle_num].z,1,0) - particle[particle_num].state[2]) > 180 )
	{
		cvmSet( particle[particle_num].z,1,0,cvmGet(particle[particle_num].z,1,0)-360-particle[particle_num].state[2] );
	}
	else if( (cvmGet(particle[particle_num].z,1,0) - particle[particle_num].state[2]) < -180 )
	{
		cvmSet( particle[particle_num].z,1,0, cvmGet(particle[particle_num].z,1,0)+360 - particle[particle_num].state[2]);
	}
	else
		cvmSet( particle[particle_num].z,1,0, cvmGet(particle[particle_num].z,1,0) - particle[particle_num].state[2]);

	
}


float guassian(float mean,float variance)
{
	//float mean = 0	;	//平均值
	//float variance = 1;	//變異數
	float Gaussian0,Gaussian_rand,u,v;

	//srand((unsigned)time(NULL));//每次都產生不同亂數

	//for(;;)
	//{
	//	frand = rand()/((float)RAND_MAX+1);
	//	if(frand != 0)
	//		break;
	//}
	//printf("%f\n",frand);
	//Gaussian0 = (float)sqrt( -2 * (float)log( frand ) ) * (float)sin( 2 * PI * frand );

	u = rand() / (float)RAND_MAX;
    v = rand() / (float)RAND_MAX;

    Gaussian0 = sqrt(-2 * log(u)) * cos(2 * m_pi * v);

	Gaussian_rand = Gaussian0 * sqrt(variance) + mean;

	//printf("%f\n",Gaussian_rand);

	return Gaussian_rand;
}


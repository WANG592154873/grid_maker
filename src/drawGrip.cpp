#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <math.h>

using namespace std;

class CMatrix
{
    int m_Rows;
    int m_Cols;
    float *m_P;
public:
    CMatrix(int i=0,int j=0, int isI=0);
    ~CMatrix();
    void setElement(int row, int col, float value);
    float getElement(int row, int col);
    int getColSize(){return this->m_Cols;}
    int getRowSize(){return this->m_Rows;}
    void Show();
    CMatrix operator*(CMatrix &rc);
    CMatrix& operator=(const CMatrix &rc);
    CMatrix (CMatrix &rc);
};
CMatrix::CMatrix(int i,int j, int isI)
{
    int m,n;
    m_P=0;
    if(i<0||j<0)
        return;
    m_Rows=i;
    m_Cols=j;
    m_P=new float[m_Rows*m_Cols];
    for(m=0;m<i;m++)
        for(n=0;n<j;n++)
            if(isI == 1 && m == n)
                m_P[m*m_Cols+n]=1;
            else
                m_P[m*m_Cols+n]=0;
}
void CMatrix::setElement(int row, int col, float value){
    m_P[row*m_Cols + col] = value;
}
float CMatrix::getElement(int row, int col)
{
    return m_P[row*m_Cols + col];
}

void CMatrix::Show()
{
    int i,j;
    for(i=0;i<m_Rows;i++)
    {
        for(j=0;j<m_Cols;j++)
                std::cout<<"  "<<m_P[i*m_Cols+j];
        std::cout<<std::endl;
    }
}
CMatrix CMatrix::operator*(CMatrix &rc)
{
    int i,j,k;
    CMatrix m(0,0,0);
    if(m_Cols!=rc.m_Rows)
        return m;
    CMatrix r(m_Rows,rc.m_Cols);
    for(i=0;i<m_Rows;i++)
        for(j=0;j<rc.m_Cols;j++)
            for(k=0;k<m_Cols;k++)
                r.m_P[i*rc.m_Cols+j]+=m_P[i*m_Cols+k]*rc.m_P[k*rc.m_Cols+j];
    return r;
}

CMatrix& CMatrix::operator=(const CMatrix &rc)
{
    if(m_P!=0)
        delete m_P;
    m_Rows=rc.m_Rows;
    m_Cols=rc.m_Cols;
    m_P=new float[m_Rows*m_Cols];
    for(int i=0;i<m_Rows*m_Cols;i++)
            m_P[i]=rc.m_P[i];
    return *this;
}

CMatrix::CMatrix (CMatrix &rc)
{
    int i,j;
    m_Rows=rc.m_Rows;
    m_Cols=rc.m_Cols;
    m_P=new float[m_Rows*m_Cols];
    for(i=0;i<m_Rows;i++)
        for(j=0;j<m_Cols;j++)
            m_P[i*m_Cols+j]=rc.m_P[i*m_Cols+j];
}
CMatrix::~CMatrix ()
{
    if(m_P!=0)
        delete []m_P;
}

void setCubeList( const geometry_msgs::Point point, CMatrix &map, visualization_msgs::Marker &cubeList, float grid_size = 1.0, float grid_position_z = - 0.01)
{

    int i = point.x / grid_size;
    int j = point.y / grid_size;
    int row = point.x >= 0 ? map.getRowSize() / 2 + i : map.getRowSize() / 2 + i - 1;
    int col = point.y >= 0 ? map.getColSize() / 2 - j - 1: map.getColSize() / 2 - j;
    if( row >= map.getRowSize() || col >= map.getColSize()){
        ROS_INFO("Warning : out of range");
        return;
    }
    if(map.getElement(row, col) != 1){
        map.setElement(row, col, 1);
        geometry_msgs::Point p;
        p.x = point.x >= 0 ? (float)i * grid_size + grid_size / 2.0 : (float)i * grid_size - grid_size / 2.0;
        p.y = point.y >= 0 ? (float)j * grid_size + grid_size / 2.0 : (float)j * grid_size - grid_size / 2.0;
        p.z =  grid_position_z;
        cubeList.points.push_back(p);
    }
}
int main( int argc, char** argv )
{
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(10);

    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker pointList;
    visualization_msgs::Marker cubeList;


    pointList.header.frame_id = "/odom";
    pointList.header.stamp = ros::Time::now();
    pointList.ns = "basic_shapes";
    pointList.id = 1;
    pointList.type = visualization_msgs::Marker::POINTS; //visualization_msgs::Marker::LINE_LIST
    pointList.action = visualization_msgs::Marker::ADD;
    pointList.scale.x = 0.005;
    pointList.scale.y = 0.005;
    pointList.color.b = 0.0f;
    pointList.color.r = 1.0f;
    pointList.color.g = 0.5f;
    pointList.color.a = 1.0;

    cubeList.header.frame_id = "/odom";
    cubeList.header.stamp = ros::Time::now();
    cubeList.ns = "basic_shapes";
    cubeList.id = 2;
    cubeList.type = visualization_msgs::Marker::CUBE_LIST; // Show grid
    cubeList.action = visualization_msgs::Marker::ADD;
    cubeList.scale.x = 0.1;
    cubeList.scale.y = 0.1;
    cubeList.scale.z = 0.01;
    cubeList.color.b = 0.5f;
    cubeList.color.r = 0.5f;
    cubeList.color.g = 1.0f;
    cubeList.color.a = 1.0;

    CMatrix grid_map(100,100); // 100 * 100 
    int grid_size = 0.1; // 10 CM

    // set pointList
    string path = "/home/exbot/My.txt";
    ifstream file(path.c_str());
    string str_line;
    while(getline(file, str_line)){
    	geometry_msgs::Point p;        	
    	int nPos = str_line.find_last_of(',');
    	p.x = atof(str_line.substr(0, nPos).c_str());
    	p.y = atof(str_line.substr(nPos+1, sizeof(str_line)).c_str());
    	cout<<"("<<p.x<<", "<<p.y<<")"<<endl;
    	pointList.points.push_back(p);
        setCubeList(p, grid_map, cubeList, grid_size);
	}
    markerArray.markers.push_back(pointList);
    markerArray.markers.push_back(cubeList);

	while (ros::ok())
    {
        // Publish the marker
        marker_pub.publish(markerArray);
        r.sleep();
    }
    return 0;
}
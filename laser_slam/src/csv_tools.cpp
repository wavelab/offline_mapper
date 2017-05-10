#include <laser_slam/csv_tools.h>
using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr  load_scan(const char* file_name)
{
	cout << "opening scan file: " << file_name << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ point;

    ifstream csv;
    string sx, sy, sz;
    csv.open(file_name, ios::in); 
    int c_timeout = 10;
    int c = 0;
    
    cloud_out->is_dense = true;

    while(csv.good() && !csv.eof())
    {
		c=0;
		while(sx.empty()){
        	getline(csv, sx, ' ');
        	c++;
        	if(c>c_timeout)
        		break;
        }
        c=0;
        while(sy.empty()){
        	getline(csv, sy, ' ');
        	c++;
        	if(c>c_timeout)
        		break;
        }
        
        getline(csv, sz);
        
        if(!sx.empty() && !sy.empty() && !sz.empty())
        {

	        point.x = atof(sx.c_str());
	        point.y = atof(sy.c_str());
	        point.z = atof(sz.c_str());

	        if( (point.x*point.x + point.y*point.y + point.z*point.z ) > 2)
	        {
	             cloud_out->points.push_back(point);
	        }
	        sx.clear();
	        sy.clear();
	        sz.clear();
	       	//cout <<  "point" << point.x << "," << point.y << "," << point.z <<endl;;
	    }
	
    }

    csv.close();
    return cloud_out;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr  load_scan_ply(const char* file_name)
{
	cout << "opening scan file: " << file_name << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ point;

    ifstream csv;
    string sx, sy, sz,dummy;
    csv.open(file_name, ios::in); 
    int c_timeout = 10;
    int c = 0;
    
    cloud_out->is_dense = true;

    while(csv.good() && !csv.eof())
    {

		c=0;
		while(sx.empty()){
        	getline(csv, sx, ' ');
        	c++;
        	if(c>c_timeout)
        		break;
        }
        //cout<<sx<<endl;
        c=0;
        while(sy.empty()){
        	getline(csv, sy, ' ');
        	c++;
        	if(c>c_timeout)
        		break;
        }
        //cout<<sy<<endl;

        c=0;
        while(sz.empty()){
            getline(csv, sz, ' ');
            c++;
            if(c>c_timeout)
                break;
        }
        //cout<<sz<<endl;
        
        getline(csv, dummy);
        
        if(!sx.empty() && !sy.empty() && !sz.empty())
        {

	        point.x = atof(sx.c_str());
	        point.y = atof(sy.c_str());
	        point.z = atof(sz.c_str());

	        if( (point.x*point.x + point.y*point.y + point.z*point.z ) > 2)
	        {
	             cloud_out->points.push_back(point);
	        }
	        sx.clear();
	        sy.clear();
	        sz.clear();
	       	//cout <<  "point" << point.x << "," << point.y << "," << point.z <<endl;;
	    }
	
    }

    csv.close();
    return cloud_out;
}

Eigen::Affine3d load_pose(const char* file_name)
{
	cout << "opening pose file: " << file_name << endl;
	Eigen::Affine3d pose_out;

	ifstream pose_file;
    string l1, l2, l3, l4;
    pose_file.open(file_name, ios::in); 
    int line_counter = 0;

    while(pose_file.good() && !pose_file.eof() && line_counter<4)
    {
    	getline(pose_file, l1, ' ');
        getline(pose_file, l2, ' ');
        getline(pose_file, l3, ' ');
        getline(pose_file, l4);

        pose_out.matrix()(line_counter,0) = atof(l1.c_str());
        pose_out.matrix()(line_counter,1) = atof(l2.c_str());
        pose_out.matrix()(line_counter,2) = atof(l3.c_str());
        pose_out.matrix()(line_counter,3) = atof(l4.c_str());
        line_counter++;
    }

    return pose_out;
}

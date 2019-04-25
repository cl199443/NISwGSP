//
//  Parameter.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "Parameter.h"
#include "direct.h"
#include "io64.h"
vector<string> getImageFileFullNamesInDir(const string & dir_name) {
    //DIR *dir;
	intptr_t dir;
	struct _finddata_t fd;
   // struct dirent *ent;
	//ent = new struct dirent();
    vector<string> result;
    
    const vector<string> image_formats = {
        ".bmp", ".dib",
        ".jpeg", ".jpg", ".jpe", ".JPG",
        ".jp2",
        ".png", ".PNG"
        ".pbm", ".pgm", ".ppm",
        ".sr", ".ras",
        ".tiff", ".tif" };
	//opendir
	string p;
	p.assign(dir_name+"*");
	//dir_name.append("*");
	dir = _findfirst(p.c_str(), &fd); //return the file handle
	//cout << "the file handle is "<<dir << endl; //see the handle value 
	//cout << "dir path  is " << dir_name<<endl; //see the folder path

    if(dir != -1)
	{
       // while((ent = readdir (dir)) != NULL) {

		while ( _findnext(dir, &fd) != -1) 
		{
			//读取文件夹下的待拼接图片
            string file = string(fd.name);
			 
            for(int i = 0; i < image_formats.size(); ++i) {
                if(file.length() > image_formats[i].length() &&
                   image_formats[i].compare(file.substr(file.length() - image_formats[i].length(),
                                                        image_formats[i].length())) == 0) {
                    result.emplace_back(file);
                }
            }
        }
        //closedir(dir);
		_findclose(dir);
    } else
	{
        printError("F(getImageFileFullNamesInDir) could not open directory");
    }
    return result; //store the names of all to_stitched images
}

bool isFileExist (const string & name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

Parameter::Parameter(const string & _file_name) {
    
    file_name = _file_name;
    file_dir = "./input-42-data/" + _file_name + "/";
    result_dir = "./input-42-data/0_results/" + _file_name + "-result/";
    
   // mkdir("./input-42-data/0_results/", S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
	
	_mkdir("./input-42-data/0_results/");
	_mkdir(result_dir.c_str());// , S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
#ifndef NDEBUG
    debug_dir = "./input-42-data/1_debugs/" + _file_name + "-result/";
	_mkdir("./input-42-data/1_debugs/");//  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
	_mkdir(debug_dir.c_str());//  S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);
#endif
    
    stitching_parse_file_name = file_dir + _file_name + "-STITCH-GRAPH.txt";
    
	//cout << "this is a debug point!" << endl;
    image_file_full_names = getImageFileFullNamesInDir(file_dir);
	// 获取文件夹下待拼接的图像名称，以string类型存储在vector容器内
    /*** configure ***/
    grid_size = GRID_SIZE;
    down_sample_image_size = DOWN_SAMPLE_IMAGE_SIZE;
    
    /*** by file ***/

    if(isFileExist(stitching_parse_file_name)) {
        const InputParser input_parser(stitching_parse_file_name);
        
        global_homography_max_inliers_dist   = input_parser.get<double>("*global_homography_max_inliers_dist"  , &GLOBAL_HOMOGRAPHY_MAX_INLIERS_DIST);
        local_homogrpahy_max_inliers_dist    = input_parser.get<double>( "*local_homogrpahy_max_inliers_dist"  , &LOCAL_HOMOGRAPHY_MAX_INLIERS_DIST);
        local_homography_min_features_count  = input_parser.get<   int>( "*local_homography_min_features_count", &LOCAL_HOMOGRAPHY_MIN_FEATURES_COUNT);
        
        images_count                 = input_parser.get<   int>("images_count");
        center_image_index           = input_parser.get<   int>("center_image_index");
        center_image_rotation_angle  = input_parser.get<double>("center_image_rotation_angle");
        
        /*** check ***/
        
        assert(image_file_full_names.size() == images_count);
        assert(center_image_index >= 0 && center_image_index < images_count);
        /*************/
        
        images_match_graph_manually.resize(images_count);
        for(int i = 0; i < images_count; ++i) {
            images_match_graph_manually[i].resize(images_count, false);
            vector<int> labels = input_parser.getVec<int>("matching_graph_image_edges-"+to_string(i), false);
            for(int j = 0; j < labels.size(); ++j) {
                images_match_graph_manually[i][labels[j]] = true;
            }
        }
        
        /*** check ***/
        queue<int> que;
        vector<bool> label(images_count, false);
        que.push(center_image_index);
        while(que.empty() == false) {
            int n = que.front();
            que.pop();
            label[n] = true;
            for(int i = 0; i < images_count; ++i) {
                if(!label[i] && (images_match_graph_manually[n][i] || images_match_graph_manually[i][n])) {
                    que.push(i);
                }
            }
        }
        assert(std::all_of(label.begin(), label.end(), [](bool i){return i;}));
        
        /*************/
        
#ifndef NDEBUG
        cout << "center_image_index = " << center_image_index << endl;
        cout << "center_image_rotation_angle = " << center_image_rotation_angle << endl;
        cout << "images_count = " << images_count << endl;
        cout << "images_match_graph_manually = " << endl;
        for(int i = 0; i < images_match_graph_manually.size(); ++i) {
            for(int j = 0; j < images_match_graph_manually[i].size(); ++j) {
                cout << images_match_graph_manually[i][j] << " ";
            }
            cout << endl;
        }
#endif
    }
}

const vector<vector<bool> > & Parameter::getImagesMatchGraph() const {
    if(images_match_graph_manually.empty()) {
        printError("F(getImagesMatchGraph) image match graph verification [2] didn't be implemented yet");
        return images_match_graph_automatically; /* TODO */
    }
    return images_match_graph_manually;
}

const vector<pair<int, int> > & Parameter::getImagesMatchGraphPairList() const {
     if(images_match_graph_pair_list.empty()) {
        const vector<vector<bool> > & images_match_graph = getImagesMatchGraph(); //待拼接图像的相邻匹配矩阵
        for(int i = 0; i < images_match_graph.size(); ++i) {
            for(int j = 0; j < images_match_graph[i].size(); ++j) {
                if(images_match_graph[i][j]) {
                    images_match_graph_pair_list.emplace_back(i, j);
                }
            }
        }
    }
    return images_match_graph_pair_list;    //提取出的有重叠区域的相邻待拼接图像
}
//
//  MeshGrid.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "MeshGrid.h"

const int GRID_VERTEX_SIZE = 4;

MeshGrid::MeshGrid(const int _cols, const int _rows) : Mesh2D(_cols, _rows) {
    
}
/*
网格划分为nh*nw，每个网格大小为lh*lw，网点顶点“形式”位置为(0,0),(0,1),```,(0,nw),(1,0),```,(nh,nw)，对应编号为0，1，2，3，4，···，nh*nw（XY轴式递进）
*/
const vector<Point2> & MeshGrid::getVertices() const {  //返回网格划分的全部顶点坐标
    if(vertices.empty()) {
        const int memory = (nh + 1) * (nw + 1);
        vertices.reserve(memory);
        for(int h = 0; h <= nh; ++h) {
            for(int w = 0; w <= nw; ++w) {
                vertices.emplace_back(w * lw, h * lh);  //网格顶点实际坐标,XY轴式递进
            }
        }
        assert(memory == vertices.size());
    }
    return vertices;
}
/*
		nw = _cols / GRID_SIZE + (_cols % GRID_SIZE != 0);  //宽度上，网格的区间数目
		nh = _rows / GRID_SIZE + (_rows % GRID_SIZE != 0);  //高度上，网格的区间数目
		lw = _cols / (double)nw;
		lh = _rows / (double)nh;						 //根据网格区间数目，更新网格区间大小
*/
const vector<Edge> & MeshGrid::getEdges() const {  //返回边的二个顶点编号，横向编号
    if(edges.empty()) {
        const vector<Point2i> nexts = { Point2i(1, 0), Point2i(0, 1) };
        const int memory = DIMENSION_2D * nh * nw + nh + nw;
        edges.reserve(memory);
        for(int h = 0; h <= nh; ++h)	    //高度上的网格区间数目
		{
            for(int w = 0; w <= nw; ++w)  //宽度上的网格区间数目
			{
                const Point2i p1(w, h);  //锁定一个网格顶点编号
                for(int n = 0; n < nexts.size(); ++n) 
				{
                    const Point2i p2 = p1 + nexts[n];   //以当前网格顶点为参考对象，边的扩展方向为先右侧，再下侧
                    if(p2.x >= 0 && p2.y >= 0 && p2.x <= nw && p2.y <= nh) 
					{
                        edges.emplace_back(p1.x + p1.y * (nw + 1),
                                           p2.x + p2.y * (nw + 1));  //每条边以二个顶点的编号做首尾标记
                    }
                }
            }
        }
        assert(memory == edges.size());
    }
    return edges;
}
const vector<Indices> & MeshGrid:: getPolygonsIndices() const {  //返回网格的四个顶点编号，
    if(polygons_indices.empty()) {
        const Point2i nexts[GRID_VERTEX_SIZE] = {
            Point2i(0, 0), Point2i(1, 0), Point2i(1, 1), Point2i(0, 1)  //对应每个网格的四个顶点
        };
        const int memory = nh * nw;
        polygons_indices.resize(memory);
        int index = 0;
        for(int h = 0; h < nh; ++h) {
            for(int w = 0; w < nw; ++w) {
                const Point2i p1(w, h);
                polygons_indices[index].indices.reserve(GRID_VERTEX_SIZE);
                for(int n = 0; n < GRID_VERTEX_SIZE; ++n) {
                    const Point2i p2 = p1 + nexts[n];  //扩展顺序为当前网格顶点，右侧，右下角，下侧，即以当前网格顶点为参考对象，进行顺时钟录入网格四个顶点编号信息
                    polygons_indices[index].indices.emplace_back(p2.x + p2.y * (nw + 1));
                }
                ++index;
            }
        }
        assert(memory == polygons_indices.size());
    }
    return polygons_indices;
}
const vector<Indices> & MeshGrid::getPolygonsNeighbors() const {  //返回网格的相邻网格编号（每个网格以左上角的网格顶点为代表）
    if(polygons_neighbors.empty()) {
        const vector<Point2i> nexts = {
            Point2i(1, 0), Point2i(0, 1), Point2i(-1, 0), Point2i(0, -1)
        };
        const int memory = nh * nw;
        polygons_neighbors.resize(memory);
        int index = 0;
        for(int h = 0; h < nh; ++h) {
            for(int w = 0; w < nw; ++w) {
                const Point2i p1(w, h);
                for(int n = 0; n < nexts.size(); ++n) {
                    const Point2i p2 = p1 + nexts[n];                   //以当前网格顶点为参考对象，扩展方式为右侧、下侧、左侧、上侧
                    if(p2.x >= 0 && p2.y >= 0 && p2.x < nw && p2.y < nh) {
                        polygons_neighbors[index].indices.emplace_back(p2.x + p2.y * nw);   //获取相关网格顶点的编号，每个网格以左上角的网格顶点为代表
                    }
                }
                ++index;
            }
        }
        assert(memory == polygons_neighbors.size());
    }
    return polygons_neighbors;
}
const vector<Indices> & MeshGrid::getPolygonsEdges() const {  //返回每个网格的四条边的编号
    if(polygons_edges.empty()) {
        const vector<int> nexts = {
            0, 1, 3, nw * 2 + 1  
        };
		/*
		     //0，2，4，6
		    //1，3，5，7，8
		   //9，11，13，15（以1*4网格划分为例）
		*/
        const int memory = nh * nw;
        polygons_edges.resize(memory);
        int index = 0, e_index = 0;;  //index为网格数目标记变量
        for(int h = 0; h < nh; ++h) 
		{
            for(int w = 0; w < nw; ++w) 
			{
                for(int n = 0; n < nexts.size(); ++n)
				{
                    polygons_edges[index].indices.emplace_back(e_index + nexts[n]);   //以当前顶点为参考对象，边的编号顺序为正右侧、正下侧、正下右侧、正右下侧
                }
                polygons_edges[index].indices.back() = polygons_edges[index].indices.back() - (h == nh-1) * w;
                ++index;      //网格数目索引＋1
                e_index += 2;//参考编号+2，自行找规律
            }
            polygons_edges[index - 1].indices[2] = polygons_edges[index - 1].indices[2] - 1;
            e_index += 1;
        }
        assert(memory == polygons_edges.size());
    }
    return polygons_edges;
}

const vector<Indices> & MeshGrid:: getVertexStructures() const { //返回共享该点的其他边的另一顶点编号
    if(vertex_structures.empty()) {
        const vector<Point2i> nexts = {
            Point2i(1, 0), Point2i(0, 1), Point2i(-1, 0), Point2i(0, -1)
        };
        const int memory = (nh + 1) * (nw + 1);
        vertex_structures.resize(memory);
        int index = 0;
        for(int h = 0; h <= nh; ++h) {
            for(int w = 0; w <= nw; ++w) {
                Point2i p1(w, h);
                for(int n = 0; n < nexts.size(); ++n) {
                    Point2i p2 = p1 + nexts[n];      //以当前网格顶点为参考对象，扩展方式为右侧、下侧、左侧、上侧
                    if(p2.x >= 0 && p2.y >= 0 && p2.x <= nw && p2.y <= nh) {   
                        vertex_structures[index].indices.emplace_back(p2.x + p2.y * (nw + 1));
                    }
                }
                ++index;
            }
        }
        assert(memory == vertex_structures.size());
    }
    return vertex_structures;
}
const vector<Indices> & MeshGrid::getEdgeStructures() const {    //返回共享该边的网格编号
    if(edge_structures.empty()) {
        const vector<Point2i>         nexts = { Point2i(1,  0), Point2i( 0, 1) };
        const vector<Point2i> grid_neighbor = { Point2i(0, -1), Point2i(-1, 0) };
        const int memory = DIMENSION_2D * nh * nw + nh + nw;
        edge_structures.resize(memory);
        int index = 0;
        for(int h = 0; h <= nh; ++h)
		{
            for(int w = 0; w <= nw; ++w)
			{
                Point2i p1(w, h);
                for(int n = 0; n < nexts.size(); ++n) 
				{
                    Point2i p2 = p1 + nexts[n];  //以当前网格顶点为参考对象，先考虑右侧边，再考虑下侧边
                    if(p2.x >= 0 && p2.y >= 0 && p2.x <= nw && p2.y <= nh)
					{
                        for(int j = 0; j < grid_neighbor.size(); ++j) 
						{
                            Point2i p3 = p1 + grid_neighbor[n] * j;   //考察右侧边时，先计算本网格，再计算上侧网格；考察下侧边时，先计算本网格，再计算左侧网格；每个网格都以左上角的网格顶点作为编号标记
                            if(p3.x >= 0 && p3.y >= 0 && p3.x < nw && p3.y < nh) 
							{
                                edge_structures[index].indices.emplace_back(p3.x + p3.y * nw);
                            }
                        }
                        ++index;
                    }
                }
            }
        }
        assert(memory == edge_structures.size());
    }
    return edge_structures;
}

const vector<Indices> & MeshGrid::getTriangulationIndices() const {      //返回网格划分三角的顶点编号
    if(triangulation_indices.empty()) {
        triangulation_indices.emplace_back(0, 1, 2);
        triangulation_indices.emplace_back(0, 2, 3); //顺时钟，四个顶点为0，1，2，3
    }
    return triangulation_indices;
}

const int & MeshGrid::getPolygonVerticesCount() const {
    return GRID_VERTEX_SIZE;							  //返回网格顶点数目
}

const vector<int> & MeshGrid::getBoundaryVertexIndices() const {	 //返回边缘的所有顶点编号，顺时针
    if(boundary_vertex_indices.empty()) {
        const int memory = DIMENSION_2D * (nw + nh) + 1;
        boundary_vertex_indices.reserve(memory);
        for(int tw = 0; tw < nw; ++tw) {
            boundary_vertex_indices.emplace_back(tw);    //录入图像的最上边的网格顶点编号
        }
        const int right_bottom = nw * nh + nw + nh;
        for(int rh = nw; rh < right_bottom; rh += (nw + 1)) {
            boundary_vertex_indices.emplace_back(rh);        //录入图像的最右边的网格顶点编号
        }
        const int left_bottom = nh * (nw + 1);
        for(int bw = right_bottom; bw > left_bottom; --bw) {    
            boundary_vertex_indices.emplace_back(bw);       //录入图像的最下边的网格顶点编号
        }
        for(int lh = left_bottom; lh >= 0; lh -= (nw + 1)) {
            boundary_vertex_indices.emplace_back(lh);         //录入图像的最左边的网格顶点编号
        }
        assert(memory == boundary_vertex_indices.size());
    }
    return boundary_vertex_indices;
}

const vector<int> & MeshGrid::getBoundaryEdgeIndices() const {    //返回图像边缘的所有边的编号，上下左右
    if(boundary_edge_indices.empty()) {
        const int memory = DIMENSION_2D * (nh + nw);
        boundary_edge_indices.reserve(memory);
        const int bottom_shift = DIMENSION_2D * nh * nw + nh;
        for(int w = 0; w < nw; ++w) {
            boundary_edge_indices.emplace_back(2 * w);
            boundary_edge_indices.emplace_back(bottom_shift + w);
        }
        const int dh = 2 * nw + 1;
        for(int h = 0; h < nh; ++h) {
            int tmp = h * dh;
            boundary_edge_indices.emplace_back(tmp + 1);
            boundary_edge_indices.emplace_back(tmp + dh - 1);
        }
        assert(memory == boundary_edge_indices.size());
    }
    return boundary_edge_indices;
}

template <typename T>
InterpolateVertex MeshGrid::getInterpolateVertexTemplate(const Point_<T> & _p) const {	  //获取特征点_p所在的网格编号以及_p坐标用网格顶点的线性插值表示
    const vector<Point2> & vertices = getVertices();       //网格顶点坐标
    const vector<Indices> & grids = getPolygonsIndices(); //网格的顶点编号
    
    const int grid_index = getGridIndexOfPoint(_p);   //获取特征点_p所在的网格编号
    
    const Indices & g = grids[grid_index];			//读取特征点_p所在网格的四个顶点编号
    
    const vector<int> diagonal_indices = {2, 3, 0, 1}; /* 0 1    2 3
                                                              ->
                                                          3 2    1 0 */
    assert(g.indices.size() == GRID_VERTEX_SIZE);
    
    vector<double> weights(GRID_VERTEX_SIZE);  //四个权值
    double sum_inv = 0;
    for(int i = 0; i < diagonal_indices.size(); ++i)
	{
        Point2 tmp(_p.x - vertices[g.indices[diagonal_indices[i]]].x,
                   _p.y - vertices[g.indices[diagonal_indices[i]]].y);  //网格顶点线性插值公式
        weights[i] = fabs(tmp.x * tmp.y);
        sum_inv += weights[i];
    }
    sum_inv = 1. / sum_inv;
    for(int i = 0; i < GRID_VERTEX_SIZE; ++i) {
        weights[i] = weights[i] * sum_inv;  //权值归一化
    }
    return InterpolateVertex(grid_index, weights);  //返回权值向量和特征点所在网格编号
}

InterpolateVertex MeshGrid::getInterpolateVertex(const Point_<float> & _p) const {
    return getInterpolateVertexTemplate(_p);
}
InterpolateVertex MeshGrid::getInterpolateVertex(const Point_<double> & _p) const {
    return getInterpolateVertexTemplate(_p);
}

template InterpolateVertex MeshGrid::getInterpolateVertexTemplate< float>(const Point_< float> & _p) const;
template InterpolateVertex MeshGrid::getInterpolateVertexTemplate<double>(const Point_<double> & _p) const;

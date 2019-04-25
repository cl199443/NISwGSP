//
//  MeshGrid.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitiching__MeshGrid__
#define __UglyMan_Stitiching__MeshGrid__

#include "Mesh2D.h"

class MeshGrid : public Mesh2D {
public:
    MeshGrid(const int _cols, const int _rows);
    const vector<Point2> & getVertices() const;                                               //获取所有网格顶点坐标
    const vector<Edge>   & getEdges() const;												 //获取网格中所有边信息，并用首尾二个网格顶点编号表示边
    const vector<Indices> & getPolygonsIndices() const;										//获取所有网格的四个顶点编号，以左上角的网格顶点编号作为该网格的编号
    const vector<Indices> & getPolygonsNeighbors() const;								   //获取网格的相邻网格编号，以相邻网格的左上角网格顶点编号作为其编号
    const vector<Indices> & getPolygonsEdges() const;									  //获取所有网格的四条边的编号，编号规则见注释
    const vector<Indices> & getVertexStructures() const;                                 //获取与当前网格顶点公用一条边的另一个网格顶点编号（其邻居节点编号）
    const vector<Indices> & getEdgeStructures() const;									//获取公用当前边的所有网格编号
    const vector<Indices> & getTriangulationIndices() const;                           //获取网格的三角顶点划分
    const int & getPolygonVerticesCount() const;                                      //一个网格的组成边数目(方形网格四条)     
    const vector<int> & getBoundaryVertexIndices() const;                            //获取图像边缘所有网格顶点的编号
    const vector<int> & getBoundaryEdgeIndices() const;                             //获取图像边缘所有边的编号
    
    InterpolateVertex getInterpolateVertex(const Point_<float> & _p) const;       //float:获取特征点_p所在网格的编号，并计算_p与该网格四个顶点的线性插值权重
    InterpolateVertex getInterpolateVertex(const Point_<double> & _p) const;     //double:获取特征点_p所在网格的编号，并计算_p与该网格四个顶点的线性插值权重    
    
    template <typename T>
    InterpolateVertex getInterpolateVertexTemplate(const Point_<T> & _p) const;
private:
    
};

#endif
/*

图像的网格划分规模为nh*nw（高*宽【Y*X】），每个网格大小为lh*lw；网点顶点的“形式”位置为(0,0),(0,1),```(0,nw),(1,0),```,(nh,nw)，对应编号为0，1，2，3，4，···，nh*nw（XY轴式递进）
以3*4的网格划分为例：网格顶点和边的编号规则为：

                   Y/X      0		1		2		3		4

					0       0---e0--1---e2--2---e4--3---e6--4 
					        |       |		|		|		|
							e1		e3		e5		e7      e8
							|		|		|		|		|
					1       5--e9---6--e11--7--e13--8--e15--9
					        |		|		|		|		|
							e10	   e12	   e14		e16	   e17
							|		|		|		|		|
					2      10--e18--11------12------13------14
					        |		|		|		|		|
						   e19		|		|		|	   e26
							|		|		|		|		|
					3       15------16-------16-----17--e30-18


			网格的编号以该网格左上角的网格顶点编号为代表；
*/

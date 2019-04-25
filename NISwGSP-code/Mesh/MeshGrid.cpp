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
���񻮷�Ϊnh*nw��ÿ�������СΪlh*lw�����㶥�㡰��ʽ��λ��Ϊ(0,0),(0,1),```,(0,nw),(1,0),```,(nh,nw)����Ӧ���Ϊ0��1��2��3��4����������nh*nw��XY��ʽ�ݽ���
*/
const vector<Point2> & MeshGrid::getVertices() const {  //�������񻮷ֵ�ȫ����������
    if(vertices.empty()) {
        const int memory = (nh + 1) * (nw + 1);
        vertices.reserve(memory);
        for(int h = 0; h <= nh; ++h) {
            for(int w = 0; w <= nw; ++w) {
                vertices.emplace_back(w * lw, h * lh);  //���񶥵�ʵ������,XY��ʽ�ݽ�
            }
        }
        assert(memory == vertices.size());
    }
    return vertices;
}
/*
		nw = _cols / GRID_SIZE + (_cols % GRID_SIZE != 0);  //����ϣ������������Ŀ
		nh = _rows / GRID_SIZE + (_rows % GRID_SIZE != 0);  //�߶��ϣ������������Ŀ
		lw = _cols / (double)nw;
		lh = _rows / (double)nh;						 //��������������Ŀ���������������С
*/
const vector<Edge> & MeshGrid::getEdges() const {  //���رߵĶ��������ţ�������
    if(edges.empty()) {
        const vector<Point2i> nexts = { Point2i(1, 0), Point2i(0, 1) };
        const int memory = DIMENSION_2D * nh * nw + nh + nw;
        edges.reserve(memory);
        for(int h = 0; h <= nh; ++h)	    //�߶��ϵ�����������Ŀ
		{
            for(int w = 0; w <= nw; ++w)  //����ϵ�����������Ŀ
			{
                const Point2i p1(w, h);  //����һ�����񶥵���
                for(int n = 0; n < nexts.size(); ++n) 
				{
                    const Point2i p2 = p1 + nexts[n];   //�Ե�ǰ���񶥵�Ϊ�ο����󣬱ߵ���չ����Ϊ���Ҳ࣬���²�
                    if(p2.x >= 0 && p2.y >= 0 && p2.x <= nw && p2.y <= nh) 
					{
                        edges.emplace_back(p1.x + p1.y * (nw + 1),
                                           p2.x + p2.y * (nw + 1));  //ÿ�����Զ�������ı������β���
                    }
                }
            }
        }
        assert(memory == edges.size());
    }
    return edges;
}
const vector<Indices> & MeshGrid:: getPolygonsIndices() const {  //����������ĸ������ţ�
    if(polygons_indices.empty()) {
        const Point2i nexts[GRID_VERTEX_SIZE] = {
            Point2i(0, 0), Point2i(1, 0), Point2i(1, 1), Point2i(0, 1)  //��Ӧÿ��������ĸ�����
        };
        const int memory = nh * nw;
        polygons_indices.resize(memory);
        int index = 0;
        for(int h = 0; h < nh; ++h) {
            for(int w = 0; w < nw; ++w) {
                const Point2i p1(w, h);
                polygons_indices[index].indices.reserve(GRID_VERTEX_SIZE);
                for(int n = 0; n < GRID_VERTEX_SIZE; ++n) {
                    const Point2i p2 = p1 + nexts[n];  //��չ˳��Ϊ��ǰ���񶥵㣬�Ҳ࣬���½ǣ��²࣬���Ե�ǰ���񶥵�Ϊ�ο����󣬽���˳ʱ��¼�������ĸ���������Ϣ
                    polygons_indices[index].indices.emplace_back(p2.x + p2.y * (nw + 1));
                }
                ++index;
            }
        }
        assert(memory == polygons_indices.size());
    }
    return polygons_indices;
}
const vector<Indices> & MeshGrid::getPolygonsNeighbors() const {  //������������������ţ�ÿ�����������Ͻǵ����񶥵�Ϊ����
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
                    const Point2i p2 = p1 + nexts[n];                   //�Ե�ǰ���񶥵�Ϊ�ο�������չ��ʽΪ�Ҳࡢ�²ࡢ��ࡢ�ϲ�
                    if(p2.x >= 0 && p2.y >= 0 && p2.x < nw && p2.y < nh) {
                        polygons_neighbors[index].indices.emplace_back(p2.x + p2.y * nw);   //��ȡ������񶥵�ı�ţ�ÿ�����������Ͻǵ����񶥵�Ϊ����
                    }
                }
                ++index;
            }
        }
        assert(memory == polygons_neighbors.size());
    }
    return polygons_neighbors;
}
const vector<Indices> & MeshGrid::getPolygonsEdges() const {  //����ÿ������������ߵı��
    if(polygons_edges.empty()) {
        const vector<int> nexts = {
            0, 1, 3, nw * 2 + 1  
        };
		/*
		     //0��2��4��6
		    //1��3��5��7��8
		   //9��11��13��15����1*4���񻮷�Ϊ����
		*/
        const int memory = nh * nw;
        polygons_edges.resize(memory);
        int index = 0, e_index = 0;;  //indexΪ������Ŀ��Ǳ���
        for(int h = 0; h < nh; ++h) 
		{
            for(int w = 0; w < nw; ++w) 
			{
                for(int n = 0; n < nexts.size(); ++n)
				{
                    polygons_edges[index].indices.emplace_back(e_index + nexts[n]);   //�Ե�ǰ����Ϊ�ο����󣬱ߵı��˳��Ϊ���Ҳࡢ���²ࡢ�����Ҳࡢ�����²�
                }
                polygons_edges[index].indices.back() = polygons_edges[index].indices.back() - (h == nh-1) * w;
                ++index;      //������Ŀ������1
                e_index += 2;//�ο����+2�������ҹ���
            }
            polygons_edges[index - 1].indices[2] = polygons_edges[index - 1].indices[2] - 1;
            e_index += 1;
        }
        assert(memory == polygons_edges.size());
    }
    return polygons_edges;
}

const vector<Indices> & MeshGrid:: getVertexStructures() const { //���ع���õ�������ߵ���һ������
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
                    Point2i p2 = p1 + nexts[n];      //�Ե�ǰ���񶥵�Ϊ�ο�������չ��ʽΪ�Ҳࡢ�²ࡢ��ࡢ�ϲ�
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
const vector<Indices> & MeshGrid::getEdgeStructures() const {    //���ع���ñߵ�������
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
                    Point2i p2 = p1 + nexts[n];  //�Ե�ǰ���񶥵�Ϊ�ο������ȿ����Ҳ�ߣ��ٿ����²��
                    if(p2.x >= 0 && p2.y >= 0 && p2.x <= nw && p2.y <= nh)
					{
                        for(int j = 0; j < grid_neighbor.size(); ++j) 
						{
                            Point2i p3 = p1 + grid_neighbor[n] * j;   //�����Ҳ��ʱ���ȼ��㱾�����ټ����ϲ����񣻿����²��ʱ���ȼ��㱾�����ټ����������ÿ�����������Ͻǵ����񶥵���Ϊ��ű��
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

const vector<Indices> & MeshGrid::getTriangulationIndices() const {      //�������񻮷����ǵĶ�����
    if(triangulation_indices.empty()) {
        triangulation_indices.emplace_back(0, 1, 2);
        triangulation_indices.emplace_back(0, 2, 3); //˳ʱ�ӣ��ĸ�����Ϊ0��1��2��3
    }
    return triangulation_indices;
}

const int & MeshGrid::getPolygonVerticesCount() const {
    return GRID_VERTEX_SIZE;							  //�������񶥵���Ŀ
}

const vector<int> & MeshGrid::getBoundaryVertexIndices() const {	 //���ر�Ե�����ж����ţ�˳ʱ��
    if(boundary_vertex_indices.empty()) {
        const int memory = DIMENSION_2D * (nw + nh) + 1;
        boundary_vertex_indices.reserve(memory);
        for(int tw = 0; tw < nw; ++tw) {
            boundary_vertex_indices.emplace_back(tw);    //¼��ͼ������ϱߵ����񶥵���
        }
        const int right_bottom = nw * nh + nw + nh;
        for(int rh = nw; rh < right_bottom; rh += (nw + 1)) {
            boundary_vertex_indices.emplace_back(rh);        //¼��ͼ������ұߵ����񶥵���
        }
        const int left_bottom = nh * (nw + 1);
        for(int bw = right_bottom; bw > left_bottom; --bw) {    
            boundary_vertex_indices.emplace_back(bw);       //¼��ͼ������±ߵ����񶥵���
        }
        for(int lh = left_bottom; lh >= 0; lh -= (nw + 1)) {
            boundary_vertex_indices.emplace_back(lh);         //¼��ͼ�������ߵ����񶥵���
        }
        assert(memory == boundary_vertex_indices.size());
    }
    return boundary_vertex_indices;
}

const vector<int> & MeshGrid::getBoundaryEdgeIndices() const {    //����ͼ���Ե�����бߵı�ţ���������
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
InterpolateVertex MeshGrid::getInterpolateVertexTemplate(const Point_<T> & _p) const {	  //��ȡ������_p���ڵ��������Լ�_p���������񶥵�����Բ�ֵ��ʾ
    const vector<Point2> & vertices = getVertices();       //���񶥵�����
    const vector<Indices> & grids = getPolygonsIndices(); //����Ķ�����
    
    const int grid_index = getGridIndexOfPoint(_p);   //��ȡ������_p���ڵ�������
    
    const Indices & g = grids[grid_index];			//��ȡ������_p����������ĸ�������
    
    const vector<int> diagonal_indices = {2, 3, 0, 1}; /* 0 1    2 3
                                                              ->
                                                          3 2    1 0 */
    assert(g.indices.size() == GRID_VERTEX_SIZE);
    
    vector<double> weights(GRID_VERTEX_SIZE);  //�ĸ�Ȩֵ
    double sum_inv = 0;
    for(int i = 0; i < diagonal_indices.size(); ++i)
	{
        Point2 tmp(_p.x - vertices[g.indices[diagonal_indices[i]]].x,
                   _p.y - vertices[g.indices[diagonal_indices[i]]].y);  //���񶥵����Բ�ֵ��ʽ
        weights[i] = fabs(tmp.x * tmp.y);
        sum_inv += weights[i];
    }
    sum_inv = 1. / sum_inv;
    for(int i = 0; i < GRID_VERTEX_SIZE; ++i) {
        weights[i] = weights[i] * sum_inv;  //Ȩֵ��һ��
    }
    return InterpolateVertex(grid_index, weights);  //����Ȩֵ����������������������
}

InterpolateVertex MeshGrid::getInterpolateVertex(const Point_<float> & _p) const {
    return getInterpolateVertexTemplate(_p);
}
InterpolateVertex MeshGrid::getInterpolateVertex(const Point_<double> & _p) const {
    return getInterpolateVertexTemplate(_p);
}

template InterpolateVertex MeshGrid::getInterpolateVertexTemplate< float>(const Point_< float> & _p) const;
template InterpolateVertex MeshGrid::getInterpolateVertexTemplate<double>(const Point_<double> & _p) const;

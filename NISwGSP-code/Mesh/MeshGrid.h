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
    const vector<Point2> & getVertices() const;                                               //��ȡ�������񶥵�����
    const vector<Edge>   & getEdges() const;												 //��ȡ���������б���Ϣ��������β�������񶥵��ű�ʾ��
    const vector<Indices> & getPolygonsIndices() const;										//��ȡ����������ĸ������ţ������Ͻǵ����񶥵�����Ϊ������ı��
    const vector<Indices> & getPolygonsNeighbors() const;								   //��ȡ��������������ţ���������������Ͻ����񶥵�����Ϊ����
    const vector<Indices> & getPolygonsEdges() const;									  //��ȡ��������������ߵı�ţ���Ź����ע��
    const vector<Indices> & getVertexStructures() const;                                 //��ȡ�뵱ǰ���񶥵㹫��һ���ߵ���һ�����񶥵��ţ����ھӽڵ��ţ�
    const vector<Indices> & getEdgeStructures() const;									//��ȡ���õ�ǰ�ߵ�����������
    const vector<Indices> & getTriangulationIndices() const;                           //��ȡ��������Ƕ��㻮��
    const int & getPolygonVerticesCount() const;                                      //һ���������ɱ���Ŀ(������������)     
    const vector<int> & getBoundaryVertexIndices() const;                            //��ȡͼ���Ե�������񶥵�ı��
    const vector<int> & getBoundaryEdgeIndices() const;                             //��ȡͼ���Ե���бߵı��
    
    InterpolateVertex getInterpolateVertex(const Point_<float> & _p) const;       //float:��ȡ������_p��������ı�ţ�������_p��������ĸ���������Բ�ֵȨ��
    InterpolateVertex getInterpolateVertex(const Point_<double> & _p) const;     //double:��ȡ������_p��������ı�ţ�������_p��������ĸ���������Բ�ֵȨ��    
    
    template <typename T>
    InterpolateVertex getInterpolateVertexTemplate(const Point_<T> & _p) const;
private:
    
};

#endif
/*

ͼ������񻮷ֹ�ģΪnh*nw����*��Y*X������ÿ�������СΪlh*lw�����㶥��ġ���ʽ��λ��Ϊ(0,0),(0,1),```(0,nw),(1,0),```,(nh,nw)����Ӧ���Ϊ0��1��2��3��4����������nh*nw��XY��ʽ�ݽ���
��3*4�����񻮷�Ϊ�������񶥵�ͱߵı�Ź���Ϊ��

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


			����ı���Ը��������Ͻǵ����񶥵���Ϊ����
*/

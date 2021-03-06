/*** 
  list of structures of mesh, face, node and any necessary data structure for geometries

  it stores a list of node and a list of triangles (faces)

  it allows non-manifold mesh (it simply is a list of triangles)

  each node knows the faces sharing it

  each node knows its one-ring neighbor

  for fast implementation, everything is public

  each node could be mapped to an external node (for embedded mesh)

  each node could also be mapped to an external tet element if the mesh is only for rendering

  mesh is, in general, for rendering

  it also represents interface (for domain decomposition method)

  Vox is voxel class, it has an center and the pointer to its 8 nodes

  VoxMesh is a mesh consisting of voxel, it keeps a list of voxel on the surface

  Cluster is a shape matching region

  Cluster is also a vox mesh (in order to render, o.w. we can only see nodes) so cluster maintains a surface as well
  
  Cluster needs to have hierarchy relation for multi-grid 

  Cluster needs to have neighbor information

            
  by Yin Yang @ UT Dallas

 ***/

#ifndef MY_MESH_H
#define MY_MESH_H

#include <utility>
#include <vector>
#include <stack>
#include <iostream>
#include <fstream>

#include "Eigen/Dense"
#include "Eigen/Geometry"


//#include "kernel.h"

using namespace std;
using namespace Eigen;

struct Node;
struct DuplicatedNode;
struct Face;
struct Mesh;
struct Cluster;
struct Vox;
struct VoxMesh;
struct Level;
struct Data4Mobile;
struct Data4PC;

typedef enum{
	HSM_DATA_4_MOBILE_TYPE_NONE,
	HSM_DATA_4_MOBILE_TYPE_VX,
	HSM_DATA_4_MOBILE_TYPE_VY,
	HSM_DATA_4_MOBILE_TYPE_VZ,
	HSM_DATA_4_MOBILE_TYPE_FX,
	HSM_DATA_4_MOBILE_TYPE_FY,
	HSM_DATA_4_MOBILE_TYPE_FZ,
	HSM_DATA_4_MOBILE_TYPE_STRING,
	HSM_DATA_4_MOBILE_TYPE_V_SIZE,
	HSM_DATA_4_MOBILE_TYPE_F_SIZE,
	HSM_DATA_4_MOBILE_TYPE_ALL,
	HSM_DATA_4_MOBILE_TYPE_UPDATE
}HSM_DATA_4_MOBILE_TYPE;

struct Node//: public HNPacketLockedMarshallHandler
{
	Node();
	~Node(){}

	Vector3d coordinate;
	Vector3d displacement;
	Vector3d normal;
	Vector3d cur_normal;
	Vector3d velocity;
	Vector3d static_position;
	Vector3d target_position;
	Vector3d force;
	Vector3d prescribed_position;
	Vector3d prescribed_angular_velocity;
	Vector3d prescribed_linear_velocity;
	Vector3d prescribed_preposition;
	Matrix3d prescribed_rotation;
	Matrix3d prescribed_prerotation;
	
	double mass;

	bool flag_visited;
	bool flag_anchor_node;
	bool flag_surface_node;
	bool flag_trigger_node;
	bool flag_constraint_node;
	int flag_constraint_type;//1 for position, 2 for orientation
	
	int external_node_idx;				
	int external_ele_idx;				
	
			
	int idx;

	// all the faces have this node
	vector<Face*> incident_faces;		

	// the one-ring neighbor nodes
	vector<Node*> one_ring_neighbor;	

	// all the clusters have this node
	vector<Cluster*> incident_cluster;

	// all the duplicates copy of this node
	vector<DuplicatedNode*> duplicates;

	// an axillary pointer, pointer to a duplicate node, this duplicate node is normally a copy of the node
	DuplicatedNode* linked_duplictae;

	// child node with same pos in next level
	Node* child_node;
	Node* parent_node;

	//interpolations using barycentric coordinates
	//Node * list_interpolation_nodes[8];
	//double para_interpolate[8];
	vector <Node *> list_interpolation_nodes[8];
	vector <double> para_interpolate[8];
	void clearPara()
	{
		incident_cluster.clear();

		for (int i = 0; i < 8; i++)
		{
			list_interpolation_nodes[i].clear();
			para_interpolate[i].clear();
		}

	}

	void clearPara(int begin_index)
	{
		incident_cluster.erase(incident_cluster.begin() + begin_index, incident_cluster.end());

		for (int i = 0; i < 8; i++)
		{
			list_interpolation_nodes[i].erase(list_interpolation_nodes[i].begin() + begin_index, list_interpolation_nodes[i].end());
			para_interpolate[i].erase(para_interpolate[i].begin() + begin_index, para_interpolate[i].end());
		}


	}

	void computePara(double vox_cube, int index)
	{

		double temppara = 0.0;
		temppara = (abs(list_interpolation_nodes[6][index]->coordinate.x() - coordinate.x()) 
			* abs(list_interpolation_nodes[6][index]->coordinate.y() - coordinate.y()) 
			* abs(list_interpolation_nodes[6][index]->coordinate.z() - coordinate.z())) / vox_cube;
		//para_interpolate_0.push_back(temppara);
		para_interpolate[0].push_back(temppara);

		temppara = (abs(list_interpolation_nodes[7][index]->coordinate.x() - coordinate.x()) 
			* abs(list_interpolation_nodes[7][index]->coordinate.y() - coordinate.y()) 
			* abs(list_interpolation_nodes[7][index]->coordinate.z() - coordinate.z())) / vox_cube;
		//para_interpolate_1.push_back(temppara);
		para_interpolate[1].push_back(temppara);

		temppara = (abs(list_interpolation_nodes[4][index]->coordinate.x() - coordinate.x()) 
			* abs(list_interpolation_nodes[4][index]->coordinate.y() - coordinate.y()) 
			* abs(list_interpolation_nodes[4][index]->coordinate.z() - coordinate.z())) / vox_cube;
		//para_interpolate_2.push_back(temppara);
		para_interpolate[2].push_back(temppara);

		temppara = (abs(list_interpolation_nodes[5][index]->coordinate.x() - coordinate.x()) 
			* abs(list_interpolation_nodes[5][index]->coordinate.y() - coordinate.y()) 
			* abs(list_interpolation_nodes[5][index]->coordinate.z() - coordinate.z())) / vox_cube;
		//para_interpolate_3.push_back(temppara);
		para_interpolate[3].push_back(temppara);

		temppara = (abs(list_interpolation_nodes[2][index]->coordinate.x() - coordinate.x()) 
			* abs(list_interpolation_nodes[2][index]->coordinate.y() - coordinate.y()) 
			* abs(list_interpolation_nodes[2][index]->coordinate.z() - coordinate.z())) / vox_cube;
		//para_interpolate_4.push_back(temppara); 
		para_interpolate[4].push_back(temppara);

		temppara = (abs(list_interpolation_nodes[3][index]->coordinate.x() - coordinate.x()) 
			* abs(list_interpolation_nodes[3][index]->coordinate.y() - coordinate.y()) 
			* abs(list_interpolation_nodes[3][index]->coordinate.z() - coordinate.z())) / vox_cube;
		//para_interpolate_5.push_back(temppara);
		para_interpolate[5].push_back(temppara);

		temppara = (abs(list_interpolation_nodes[0][index]->coordinate.x() - coordinate.x()) 
			* abs(list_interpolation_nodes[0][index]->coordinate.y() - coordinate.y()) 
			* abs(list_interpolation_nodes[0][index]->coordinate.z() - coordinate.z())) / vox_cube;
		//para_interpolate_6.push_back(temppara);
		para_interpolate[6].push_back(temppara);

		temppara = (abs(list_interpolation_nodes[1][index]->coordinate.x() - coordinate.x()) 
			* abs(list_interpolation_nodes[1][index]->coordinate.y() - coordinate.y()) 
			* abs(list_interpolation_nodes[1][index]->coordinate.z() - coordinate.z())) / vox_cube;
		//para_interpolate_7.push_back(temppara); 
		para_interpolate[7].push_back(temppara);
	}
	/*
	//add for networking
	virtual long marshallData(unsigned char *marshalledData)
	{
		int pos = 0;
		memcpy(marshalledData, &coordinate, sizeof(coordinate));
		pos += sizeof(coordinate);
		memcpy(marshalledData+pos, &displacement, sizeof(displacement));
		pos += sizeof(displacement);
		//cout << "marshall node" << endl;
		//cout << coordinate << endl;
		//cout << displacement << endl;
		return getMarshalledDataSize();
	
	}
	virtual long getMarshalledDataSize()
	{
		return sizeof(coordinate) + sizeof(displacement);
	}
	
	virtual void unmarshallData(unsigned char *marshalledData)
	{
		//cout << "unmarshall node" << endl;
		memcpy(&coordinate, marshalledData, sizeof(coordinate));
		memcpy(&displacement, marshalledData+sizeof(coordinate), sizeof(displacement));
		//cout << coordinate << endl;
		//cout << displacement << endl;
	}
	*/
};

struct DuplicatedNode
{
	DuplicatedNode();
	DuplicatedNode(Node* n);
	
	~DuplicatedNode(){}
	// duplicate node is a duplicates to a node
	Node* mapped_node;
	// duplicate node is also possible a child of another duplicate node
	DuplicatedNode* parent_dup;

	Cluster* master_cluster;

	Vector3d coordinate;
	Vector3d displacement;
	Vector3d normal;
	Vector3d cur_normal;
	Vector3d velocity;
	Vector3d static_position;
	Vector3d static_position_timestep;
	Vector3d target_position;
	Vector3d force;
	//new
	Vector3d rest_position;

	double mass;

	bool flag_visited;
	bool flag_constraint_node;
	
	int external_node_idx;				
	int external_ele_idx;				

	double barycentric_coord[4];

	int index;

	vector<Face*> incident_faces;		// all the faces having this node

};

struct Face//: public HNPacketLockedMarshallHandler
{
	Face();
	~Face(){}

	int index;
	double area;
	Node *node0, *node1, *node2;
	Vector3d normal;
	
	void updateArea();
	
	bool hasNode(Node* pNode)
	{
		return ( pNode==node0 || pNode==node1 || pNode==node2);
	}

	//add for networking
	int idx_node0, idx_node1, idx_node2;
	/*
	virtual long marshallData(unsigned char *marshalledData)
	{
		int pos = 0;
		memcpy(marshalledData+pos, &idx_node0, sizeof(idx_node0));
		pos += sizeof(idx_node0);
		memcpy(marshalledData+pos, &idx_node1, sizeof(idx_node1));
		pos += sizeof(idx_node1);
		memcpy(marshalledData+pos, &idx_node2, sizeof(idx_node2));
		//cout << "marshall face" << endl;
		//cout << idx_node0 << idx_node1 << idx_node2 << endl;
		return getMarshalledDataSize();
	
	}
	virtual long getMarshalledDataSize()
	{
		return sizeof(idx_node0) + sizeof(idx_node1) + sizeof(idx_node2);
	}
	
	virtual void unmarshallData(unsigned char *marshalledData)
	{
		//cout << "unmarshall face" << endl;
		int pos = 0;
		memcpy(&idx_node0, marshalledData+pos, sizeof(idx_node0));
		pos += sizeof(idx_node0);
		memcpy(&idx_node1, marshalledData+pos, sizeof(idx_node1));
		pos += sizeof(idx_node1);
		memcpy(&idx_node2, marshalledData+pos, sizeof(idx_node2));
		//cout << idx_node0 << idx_node1 << idx_node2 << endl;
	}
	*/
};

struct Vox
{
	Vox();
	~Vox(){}

	Node* node_0;		// up, left, back
	Node* node_1;		// up, right, back
	Node* node_2;		// up, right, front
	Node* node_3;		// up, left, front
	Node* node_4;		// down, left, back
	Node* node_5;		// down, right, back
	Node* node_6;		// down, right, front
	Node* node_7;		// down, left, front

	////next level node corresponding to node 0 - node 1
	//Node * node_0_child;
	//Node * node_1_child;
	//Node * node_2_child;
	//Node * node_3_child;
	//Node * node_4_child;
	//Node * node_5_child;
	//Node * node_6_child;
	//Node * node_7_child;

	// flags for surface voxel, if it is a surface voxel, find the face need to be rendered
	bool flag_surface_vox;
	bool flag_top_face;
	bool flag_bottom_face;
	bool flag_left_face;
	bool flag_right_face;
	bool flag_front_face;
	bool flag_back_face;
	

	Vox* left_neighbor;
	Vox* right_neighbor;
	Vox* front_neighbor;
	Vox* back_neighbor;
	Vox* left_back_neighbor;
	Vox* right_back_neighbor;
	Vox* left_front_neighbor;
	Vox* right_front_neighbor;

	Vox* top_neighbor;
	Vox* top_left_neighbor;
	Vox* top_right_neighbor;
	Vox* top_front_neighbor;
	Vox* top_back_neighbor;
	Vox* top_left_back_neighbor;
	Vox* top_right_back_neighbor;
	Vox* top_left_front_neighbor;
	Vox* top_right_front_neighbor;

	Vox* bottom_neighbor;
	Vox* bottom_left_neighbor;
	Vox* bottom_right_neighbor;
	Vox* bottom_front_neighbor;
	Vox* bottom_back_neighbor;
	Vox* bottom_left_back_neighbor;
	Vox* bottom_right_back_neighbor;
	Vox* bottom_left_front_neighbor;
	Vox* bottom_right_front_neighbor;


	bool flag_selected;
	bool flag_visited;

	Vector3d vox_center;
	double half_size;

	//cluster grid coordinates
	Vector3d coord_grid;
	Vox * parent_vox;
	int clusterid;
	//Cluster * pcluster;

	Vox * list_near_parentVox[8];
	double para_interpolate[8];
};


/// Cluster is also a vox mesh, theoretically, it does not need to be vox list, but vox make it easier for rendering
/// Cluster have duplicated copy of nodes, and the main vox mesh has the node
/// so it is more clear to make a new structure for cluster
struct Cluster
{
	Cluster();
	~Cluster();
	
	void clear();
		
	// generate node list from the vox list
	void initializeNodeList();
	
	void linkToMappedNode();
	
	void clearVoxVisit();
	
	void clearNodeVisit();
	
	void clearMappedNodeVisit();
	
	void markVoxVisit();
	
	void markNodeVisit();

	void markMappedNodeVisit();
	
	void inheritParentMotion();
	
	void uploadToParentMotion();
			
	void computeRestMassCentroid();
	
	void computeCurrentMassCentroid();
	void computeCurrentMassCentroid4Target();
	void computeCurrentMassCentroid4Static();
	
	void computeAQQ();
			
	void goRest();

	void buildVelocityMatrix();

	vector<DuplicatedNode> node_list;
	vector<DuplicatedNode*> trigger_node_list;
	vector<Vox*> vox_list;
	vector<DuplicatedNode*> surface_quads;						// for rendering, successive 4 duplicate nodes consist a quad
			
	int num_node;
	int num_vox;
	int num_surface_quads;							

	// the following parameters are named according to Muller's shape matching paper
	Vector3d original_center;
	Vector3d current_center;
	Matrix3d a_qq;
	Matrix3d a_pq;
	Matrix3d a;
	Matrix3d r;
	MatrixXd velocity_matrix;
	MatrixXd pair_matrix;
	double mass;

	Matrix<double, 3, 9> a_pq_tilde;
	Matrix<double, 3, 9> a_qq_tilde;
	Matrix<double, 3, 9> r_tilde;
	Matrix<double, 9, 9> a_tilde;

	// alpha is the parameter somehow similar to stiffness
	double alpha;

	// beta is the parameter control how much deformation is able to occur at each cluster
	double beta;
	
	// kappa is the damping constant
	double kappa;

	// the bounding cube of the cluster
	double max_x;
	double max_y;
	double max_z;
	double min_x;
	double min_y;
	double min_z;


	Vector3d linear_velocity;
	Vector3d angular_velocity;

	bool flag_inverted;
	bool flag_constrained;
	bool flag_cube_constrained;
	bool flag_cube_anchored;

	//penalty term
	double inertial_penalty;
	DuplicatedNode * constraint_node;
	vector<DuplicatedNode *> constraint_node_list;

	//parent cluster & children clusters
	Cluster * parent_cluster;
	vector<Cluster *> cluster_list_children;
	int level_index;
	//new for multigrid
	vector<Cluster *> leaf_list;
	Cluster * super_parent_cluster;
	vector<double> leaf_parameter;
	double term_normlize;

	Vector3d linear_displacement;
	Matrix3d angular_displacement;
	bool flag_isRendering;
	//Quaterniond angular_displacement;
};

struct Mesh//: public HNPacketLockedMarshallHandler
{
	Mesh();
	~Mesh()
	{
		clear();
	}

	int number_node;
	int number_face;

	vector<Face> face_list;
	vector<Node> node_list;
	vector<int> node_index_map;

	double max_x, max_y, max_z;
	double min_x, min_y, min_z;
	double dimension;
	double scalar;

	bool flag_normalized;

	Vector3d mesh_center;

	Mesh * mesh_simplified;

	void updateArea();
	
	void clearNodeVisit();
	void markNodeVisit();
	void clear();
	void scale();
	void scale(double scalar);
	void read(const char* filename);
	/*
	virtual long marshallData(unsigned char *marshalledData)
	{
		int pos = 0;
		memcpy(marshalledData+pos, &number_node, sizeof(number_node));
		pos += sizeof(number_node);
		memcpy(marshalledData+pos, &number_face, sizeof(number_face));
		pos += sizeof(number_face);
		memcpy(marshalledData+pos, &flag_normalized, sizeof(flag_normalized));
		pos += sizeof(flag_normalized);
		
		//cout << number_node << number_face <<endl;
		//cout << "marshall.." << endl;
		int i = 0; 
		for( i = 0; i < node_list.size(); i++)
		{
			node_list[i].marshallData(marshalledData+pos);
			pos += node_list[i].getMarshalledDataSize();
		}
		for( i = 0; i < face_list.size(); i++)
		{
			face_list[i].marshallData(marshalledData+pos);
			pos += face_list[i].getMarshalledDataSize();
		}
		//cout << getMarshalledDataSize() << endl;
		return getMarshalledDataSize();
	
	}
	virtual long getMarshalledDataSize()
	{
		long size = 0;
		size += sizeof(number_node);
		size += sizeof(number_face);
		size += sizeof(flag_normalized);
		int i = 0;
		for(i = 0; i < node_list.size(); i++)
		{
			size += node_list[i].getMarshalledDataSize();
		}
		for( i = 0; i < face_list.size(); i++)
		{
			size += face_list[i].getMarshalledDataSize();
		}
		return size;
	}
	
	virtual void unmarshallData(unsigned char *marshalledData)
	{
		//cout << "unmarshall.." << endl;

		int pos = 0;
		memcpy(&number_node, marshalledData, sizeof(number_node));
		pos += sizeof(number_node);
		memcpy(&number_face, marshalledData+pos, sizeof(number_face));
		pos += sizeof(number_face);
		memcpy(&flag_normalized, marshalledData+pos, sizeof(flag_normalized));
		pos += sizeof(flag_normalized);
		
		//cout << number_node << " " << number_face << endl;
		if(node_list.size() == 0)
		{
			int i = 0;
			for( i = 0; i < number_node; i++)
			{
				Node newnode;
				newnode.unmarshallData(marshalledData + pos);
				node_list.push_back(newnode);
				pos += newnode.getMarshalledDataSize();
			}
			for( i = 0; i < number_face; i++)
			{
				Face newface;
				newface.unmarshallData(marshalledData + pos);
				newface.node0 = &node_list[newface.idx_node0];
				newface.node1 = &node_list[newface.idx_node1];
				newface.node2 = &node_list[newface.idx_node2];
				face_list.push_back(newface);
				pos += newface.getMarshalledDataSize();
			}
		}
		else
		{
			int i = 0;
			for( i = 0; i < number_node; i++)
			{
				node_list[i].unmarshallData(marshalledData + pos);
				pos += node_list[i].getMarshalledDataSize();
			}
			for( i = 0; i < number_face; i++)
			{
				face_list[i].unmarshallData(marshalledData + pos);
				face_list[i].node0 = &node_list[face_list[i].idx_node0];
				face_list[i].node1 = &node_list[face_list[i].idx_node1];
				face_list[i].node2 = &node_list[face_list[i].idx_node2];
				pos += face_list[i].getMarshalledDataSize();
			}
		}
	}
	*/
};

// VoxMesh is a bunch of connected voxels, each pair of neighbor voxels share a face of 4 nodes
struct VoxMesh
{
	VoxMesh();
	~VoxMesh()
	{
		clear();
	}
	
	vector<Node>  node_list;
	vector<Vox>   vox_list;

	// vox_locator has the same dimensional of the vol, so you can fast get the vox for the x, y and z index
	vector<Vox*>  vox_locator;

	vector<Vox*>  surface_vox_list;
	vector<Node*> surface_node_list;
	vector<Node*> anchor_node_list;
	vector<Node*> mass_node_list;
	vector<Cluster> cluster_list;										// cluster is linear, for native shape matching
	//vector<Node*> active_node_list;										// for more than one active (external force)
	vector<Node*> constraint_node_list;                                 // for more than one constraint
	vector<Node*> another_constraint_node_list;
	vector<Cluster *>constraint_cluster_list;                           //clusters which have constraints
	vector<Vector3d> constraint_displacement;
	vector<Vector3d> another_constraint_displacement;
	Vector3d constraint_center;
	Vector3d another_constraint_center;
	vector<Cluster *> static_cube_list;
	vector<Cluster *> active_cube_list;
	
	vector<Cluster*> root_to_leaf_cluster_list;							// store the clusters in the BFS order -> from root to leaf, level by level
	vector<Cluster*> leaf_to_root_cluster_list;							
	
	Node* active_node;													// the node where the external force applies to
	Node* constraint_node;												// the node that needs to be at some specified position
	Cluster* inverted_cluster;

	double old_energy;
	double new_energy;

	int num_node;
	int num_vox;
	int num_surface_vox;
	int num_surface_node;
	int num_cluster;
	double vox_size;
	int num_pair;

	int current_sibling_index;											// used for visualized cluster sibling
	int current_cluster_level;

	bool* neighbor_flag;												// for any two clusters i, j (i>j), if neighbor_flag[i*number_clust + j] is true, which means i and j are connected by sharing at least one node
	MatrixXd system_matrix;

	void clear();
		
	void clearVoxSelect();
	void clearNodeVisit();
	void clearNodeTrigger();
	//void markNodeVisit();
	//void updateVoxVisit();
	//void initializeClusterHierarchy();
	//void appendMaxConnectedComponent(vector<Vox*> &v_list);
	//void createBFSOrder();
	//void initializeNeighborFlag();
	//void buildSystemMatrix();
};

struct Level{
	Level();
	~Level();

	void clear();

	int gridDensity_total;
	int gridDensity;
	int level_index;
	VoxMesh * voxmesh_level;
	int times_ShapeMatching;

	int num_sim_vox;
	int num_surface_vox;
	int num_sim_node;
};

# endif
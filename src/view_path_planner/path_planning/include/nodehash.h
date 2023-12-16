#ifndef NODEHASH_H 
#define NODEHASH_H
#include <unordered_map>
#include <fstream>
#include <iostream>
using namespace std;

struct Node_hash
{
    int x;
    int y;
    int z;
 
    Node_hash(int a, int b, int c) : x(a), y(b), z(c) {}
 
    bool operator==(const Node_hash& p) const {
        return x == p.x && y == p.y && z == p.z;
    }

    friend std::ostream &operator<<( std::ostream &output, const Node_hash& p )
    { 
      output << " " << p.x << " " << p.y << " " << p.z;
      return output;            
    }

};
 
// Way-1: specialized hash function for unordered_map keys
struct NodeHash
{
    std::size_t operator() (const Node_hash & node) const {
        return  (node.x * 1483 + node.y * 2791 + node.z*4363);
    }
};

unordered_map<Node_hash, int, NodeHash> density_map;
unordered_map<Node_hash, int, NodeHash> Color_density_map;

bool ColorIsFeatureRich(Eigen::Vector3i _query_color);
bool ColorIsFeatureLess(Eigen::Vector3i _query_color);

#endif
// -*- C++ -*-

#ifndef _VIS_3D_NODE
#define _VIS_3D_NODE

#include <set>
#include "Vector.h"

using namespace std;

class Node3D;

typedef enum {
    NT_3_PLUS_CLAUSE, NT_2_CLAUSE
} EdgeAttribute;
typedef pair<Node3D *, EdgeAttribute> ExtNode3D;
// extended node: node with additional edge attribute; used in neighbor set

class Node3D {
private:
    unsigned long ident;
    int mark;    // mark bit for different purposes

    Vector3D node_pos;

    set<ExtNode3D> neighbor_nodes;

    int c_weight; // used for graph coarsening (weight: number of merged nodes)

public:
    explicit Node3D(long id = 0) : ident(id), mark(0), c_weight(1) {}

    ~Node3D() = default;

    // modifiers
    void set_pos(double x, double y, double z);

    void set_pos(const Vector3D &p);

    void set_id(unsigned long i);

    void set_weight(int i);

    bool add_neighbor(Node3D *n, EdgeAttribute a = NT_3_PLUS_CLAUSE);
    // returns true iff element was not already present

    // observables
    [[nodiscard]] const Vector3D &position() const;

    [[nodiscard]] unsigned long id() const;

    [[nodiscard]] int weight() const;
    // EdgeAttribute edge_attribute(Node3D* other_end) const;

    [[nodiscard]] const set<ExtNode3D> &neighbors() const;

    friend class Graph3D;
};

#endif

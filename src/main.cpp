#include <cmath>
#include <iostream>
#include <vector>

class Vector3 {
  public:
    
    float x;
    float y;
    float z;

    Vector3() {
        this->x = 0;
        this->y = 0;
        this->z = 0;
    }

    Vector3(float _x, float _y, float _z ) {
        this->x = _x;
        this->y = _y;
        this->z = _z;
    }

};

enum PrimitiveType {
    GeometricSphere
};

class Primitive {

    public:
        virtual PrimitiveType get_type() = 0;

};

class Sphere: public Primitive {

  public:
    float radius;
    Vector3 center_position = Vector3();

    PrimitiveType get_type() override { return PrimitiveType::GeometricSphere; }

    Sphere(Vector3 c, float r ) {
        this->center_position = c;
        this->radius = r;
    }

};

enum OctreeNodeCode {
    gray,
    black,
    white
};

class OctreeNode {

public:

    OctreeNodeCode code;
    OctreeNode *children[8];

    Vector3 min_pos = Vector3(0.0, 0.0, 0.0);
    Vector3 max_pos = Vector3(0.0, 0.0, 0.0);

    OctreeNode(){}

};

class Octree {
    
  public:

    OctreeNode* root;

    std::vector<Primitive*> inner_primitives;

    Octree( std::vector<Primitive*> _inner_primitives ){
        root = new OctreeNode();
        inner_primitives = _inner_primitives;
        this->set_dimensions_by_primitive_limits(this->inner_primitives);
    }

    void set_dimensions_by_primitive_limits( const std::vector<Primitive*> primitives ) {
        float x_max, x_min, y_max, y_min, z_max, z_min;

        for (int i=0; i<primitives.size(); i++) {
            //TODO: improve primitive type identification or generalize dimensions of bounding box
            if ( primitives[i]->get_type() == PrimitiveType::GeometricSphere ) {

                const Sphere* sphere = dynamic_cast<const Sphere*>(primitives[i]);

                x_max = x_max < (sphere->center_position.x + sphere->radius) ? (sphere->center_position.x + sphere->radius) : x_max;
                x_min = x_min > -(sphere->center_position.x + sphere->radius) ? -(sphere->center_position.x + sphere->radius) : x_min;
                y_max = y_max < (sphere->center_position.x + sphere->radius) ? (sphere->center_position.x + sphere->radius) : y_max;
                y_min = y_min > -(sphere->center_position.x + sphere->radius) ? -(sphere->center_position.x + sphere->radius) : y_min;
                z_max = z_max < (sphere->center_position.x + sphere->radius) ? (sphere->center_position.x + sphere->radius) : z_max;
                z_min = z_min > -(sphere->center_position.x + sphere->radius) ? -(sphere->center_position.x + sphere->radius) : z_min;

                this->root->max_pos = Vector3(x_max, y_max, z_max);
                this->root->min_pos = Vector3(x_min, y_min, z_min);

            }
        }

    }

};

OctreeNodeCode classify_against_primitive(  OctreeNode* node, Primitive* primitive ) {

    switch ( primitive->get_type() ) {

        case PrimitiveType::GeometricSphere:

            const Sphere* sphere = dynamic_cast<const Sphere*>(primitive);
            float d_x, d_y, d_z, d_min, d_max, d_mid;

            d_x = sphere->center_position.x - node->min_pos.x;
            d_y = sphere->center_position.y - node->min_pos.y;
            d_z = sphere->center_position.z - node->min_pos.z;

            d_min= sqrt( d_x * d_x + d_y * d_y + d_z * d_z );

            d_x = sphere->center_position.x - node->max_pos.x;
            d_y = sphere->center_position.y - node->max_pos.y;
            d_z = sphere->center_position.z - node->max_pos.z;

            d_max= sqrt( d_x * d_x + d_y * d_y + d_z * d_z );

            d_x = sphere->center_position.x - ( node->min_pos.x + node->max_pos.x ) * .5;
            d_y = sphere->center_position.y - ( node->min_pos.y + node->max_pos.y ) * .5;
            d_z = sphere->center_position.z - ( node->min_pos.z + node->max_pos.z ) * .5;

            d_mid= sqrt( d_x * d_x + d_y * d_y + d_z * d_z );

            if ( d_min < sphere->radius && d_max < sphere->radius && d_mid < sphere->radius ) return OctreeNodeCode::black;
            if ( d_min < sphere->radius || d_max < sphere->radius || d_mid < sphere->radius ) return OctreeNodeCode::gray;
            return OctreeNodeCode::white;

        break;
    }

    return OctreeNodeCode();
}

void subdivide( OctreeNode* node ) {

    float l = abs(node->max_pos.x - node->min_pos.x);

    for (int i=0; i<8; i++) {

        node->children[i] = new OctreeNode();

        if ( i == 0 || i == 1 || i == 4 || i == 5 ) {
            node->children[i]->min_pos.x = node->min_pos.x;
            node->children[i]->max_pos.x = node->max_pos.x - .5 * l;
        } else {
            node->children[i]->max_pos.x = node->max_pos.x;
            node->children[i]->min_pos.x = node->min_pos.x + .5 * l;
        }

        if ( i % 2 == 0 ) {
            node->children[i]->max_pos.y = node->max_pos.y;
            node->children[i]->min_pos.y = node->min_pos.y + .5 * l;
        } else {
            node->children[i]->min_pos.y = node->min_pos.y;
            node->children[i]->max_pos.y = node->max_pos.y - .5 * l;
        }

        if ( i <= 3 ) {
            node->children[i]->min_pos.z = node->min_pos.z;
            node->children[i]->max_pos.z = node->max_pos.z - .5 * l;
        } else {
            node->children[i]->max_pos.z = node->max_pos.z;
            node->children[i]->min_pos.z = node->min_pos.z + .5 * l;
        }

    }
}

std::string build_tree( Primitive* primitive, OctreeNode* node, int depth ) {
        
        std::string result = "";

        switch ( classify_against_primitive(node, primitive) ) {
            case black:
                node->code = OctreeNodeCode::black;
                result+="B";
                break;
            case white:
                node->code = OctreeNodeCode::white;
                result+="W";
                break;
            case gray:
                if( depth == 0 ) {
                    node->code = OctreeNodeCode::black;
                    result+="B";
                }
                else {
                    result+="(";
                    subdivide(node);
                    for (int i=0; i<8; i++) {
                       result += build_tree(primitive, node->children[i], depth - 1);
                    }
                    result+=")";
                }
                break;
        }

        return result;
}

int main(){

    std::cout << "Hello Octree Modeling" << std::endl;

    Sphere* sphereA = new Sphere( Vector3(0,0,0), 12);

    Octree* octree = new Octree({sphereA});

    std::string result = build_tree(sphereA, octree->root, 2);

    std::cout << result << std::endl;

    return 0;
}
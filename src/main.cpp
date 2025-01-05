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

float distanceBetween(const Vector3& a, const Vector3& b) {
    return std::sqrt((b.x - a.x) * (b.x - a.x) +
                     (b.y - a.y) * (b.y - a.y) +
                     (b.z - a.z) * (b.z - a.z));
}

enum PrimitiveType {
    GeometricCone,
    GeometricCube,
    GeometricSphere,
    GeometricCylinder
};

class Primitive {

    public:
        virtual PrimitiveType get_type() = 0;

};

class Sphere: public Primitive {

  public:
    float radius;
    Vector3 center_position = Vector3();

    Sphere(Vector3 c, float r ) {
        this->center_position = c;
        this->radius = r;
    }

    PrimitiveType get_type() override { return PrimitiveType::GeometricSphere; }

    bool is_point_inside(Vector3 point) {

        float d_x = this->center_position.x - point.x;
        float d_y = this->center_position.y - point.y;
        float d_z = this->center_position.z - point.z;

        float d = sqrt( d_x * d_x + d_y * d_y + d_z * d_z );

        return d < this->radius;

    }

};

class Cube: public Primitive {

  public:
    Vector3 min;
    Vector3 max;

    Cube( Vector3 _min, Vector3 _max ){
        this->min = _min;
        this->max = _max;
    }

    PrimitiveType get_type() override { return PrimitiveType::GeometricCube; }

    bool is_point_inside(Vector3 point) {
        return this->min.x < point.x && point.x < this->max.x &&
               this->min.y < point.y && point.y < this->max.y &&
               this->min.z < point.z && point.z < this->max.z;
    }

};

class Cylinder: public Primitive {
  
  public:
    Vector3 shaft;
    float base_ray;
    Vector3 top_center;
    Vector3 bottom_center;

    Cylinder( float _base_ray, Vector3 _top_center, Vector3 _bottom_center ){
        this->base_ray = _base_ray;
        this->top_center = _top_center;
        this->bottom_center = _bottom_center;
        this->shaft = Vector3(top_center.x - _bottom_center.x, top_center.y - _bottom_center.y, top_center.z - _bottom_center.z);
    }

    PrimitiveType get_type() override { return PrimitiveType::GeometricCylinder; }

    bool is_point_inside( const Vector3 point ) { 

        // Vetor do ponto à base do cilindro
        Vector3 P_minus_CB = Vector3 {
            point.x - this->bottom_center.x,
            point.y - this->bottom_center.y,
            point.z - this->bottom_center.z
        };

        // Produto escalar e norma do vetor V
        float V_dot_V = this->shaft.x * this->shaft.x + this->shaft.y * this->shaft.y + this->shaft.z * this->shaft.z;
        float t = (P_minus_CB.x * this->shaft.x + P_minus_CB.y * this->shaft.y + P_minus_CB.z * this->shaft.z) / V_dot_V;

        // Verifica se está dentro dos limites do cilindro
        if (t < 0 || t > 1)
            return false;

        // Ponto projetado no eixo do cilindro
        Vector3 proj = {
            this->bottom_center.x + t * this->shaft.x,
            this->bottom_center.y + t * this->shaft.y,
            this->bottom_center.z + t * this->shaft.z
        };

        // Distância ao eixo do cilindro
        float dist2 = distanceBetween(point, proj); // Quadrado da distância
        return dist2 <= this->base_ray * this->base_ray;
    }

};

class Cone: public Primitive {

  public:
    float height;
    Vector3 vertex;
    float base_radius;
    Vector3 base_center;
    Vector3 shaft;

    Cone( Vector3 _base_center, float _base_radius, Vector3 _vertex ) {
        base_center = _base_center;
        base_radius = _base_radius;
        vertex = _vertex;
        height = distanceBetween(_base_center, _vertex );
        this->shaft = Vector3(vertex.x - base_center.x, vertex.y - base_center.y, vertex.z - base_center.z);
    }

    PrimitiveType get_type() override { return PrimitiveType::GeometricCone; }
 
    bool is_point_inside(Vector3 point) {

        // Vetor do ponto ao vértice do cone
        Vector3 P_minus_V = {
            point.x - vertex.x,
            point.y - vertex.y,
            point.z - vertex.z
        };

        // Produto escalar e norma do vetor V
        float V_dot_V = this->shaft.x * this->shaft.x + this->shaft.y * this->shaft.y + this->shaft.z * this->shaft.z;
        float t = (P_minus_V.x * shaft.x + P_minus_V.y * shaft.y + P_minus_V.z * shaft.z) / V_dot_V;

        // Verifica se o ponto está dentro dos limites de altura do cone
        if (t < 0 || t > 1)
            return false;

        // Ponto projetado no eixo do cone
        Vector3 proj = {
            this->vertex.x + t * this->shaft.x,
            this->vertex.y + t * this->shaft.y,
            this->vertex.z + t * this->shaft.z
        };

        // Raio no ponto projetado
        float r_proj = t * this->base_radius;

        // Distância do ponto ao eixo
        float dist2 = distanceBetween(point, proj);
        return dist2 <= r_proj * r_proj;
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
        float inner_x_max, inner_x_min, inner_y_max, inner_y_min, inner_z_max, inner_z_min;

        for (int i=0; i<primitives.size(); i++) {
             switch ( primitives[i]->get_type() ) {
                case PrimitiveType::GeometricSphere:
                {
                    Sphere* sphere = dynamic_cast<Sphere*>(primitives[i]);

                    inner_x_max = (sphere->center_position.x + sphere->radius);
                    inner_x_min = (sphere->center_position.x - sphere->radius);
                    inner_y_max = (sphere->center_position.y + sphere->radius);
                    inner_y_min = (sphere->center_position.y - sphere->radius);
                    inner_z_max = (sphere->center_position.z + sphere->radius);
                    inner_z_min = (sphere->center_position.z - sphere->radius);

                }
                break;
                case PrimitiveType::GeometricCube:
                {
                    Cube* cube = dynamic_cast<Cube*>(primitives[i]);

                    inner_x_max = cube->max.x;
                    inner_x_min = cube->min.x;
                    inner_y_max = cube->max.y;
                    inner_y_min = cube->min.y;
                    inner_z_max = cube->max.z;
                    inner_z_min = cube->min.z;

                }
                break;
                case PrimitiveType::GeometricCone:
                {
                    Cone* cone = dynamic_cast<Cone*>(primitives[i]);

                    inner_x_max = cone->base_center.x + cone->base_radius;
                    inner_x_min = cone->base_center.x - cone->base_radius;
                    inner_y_max = cone->vertex.y;
                    inner_y_min = cone->base_center.y;
                    inner_z_max = cone->base_center.z + cone->base_radius;
                    inner_z_min = cone->base_center.z - cone->base_radius;
                }
                break;
                case PrimitiveType::GeometricCylinder:
                {
                    Cylinder* cylinder = dynamic_cast<Cylinder*>(primitives[i]);

                    inner_x_max = cylinder->bottom_center.x + cylinder->base_ray;
                    inner_x_min = cylinder->bottom_center.x - cylinder->base_ray;
                    inner_y_max = cylinder->top_center.y;
                    inner_y_min = cylinder->bottom_center.y;
                    inner_z_max = cylinder->bottom_center.z + cylinder->base_ray;
                    inner_z_min = cylinder->bottom_center.z + cylinder->base_ray;
                }
                break;
             }

            x_max = x_max < inner_x_max ? inner_x_max : x_max;
            x_min = x_min > inner_x_min ? inner_x_min : x_min;
            y_max = y_max < inner_y_max ? inner_y_max : y_max;
            y_min = y_min > inner_y_min ? inner_y_min : y_min;
            z_max = z_max < inner_z_max ? inner_z_max : z_max;
            z_min = z_min > inner_z_min ? inner_z_min : z_min;

            this->root->max_pos = Vector3(x_max, y_max, z_max);
            this->root->min_pos = Vector3(x_min, y_min, z_min);

        }

    }

};

OctreeNodeCode classify_against_primitive(  OctreeNode* node, Primitive* primitive ) {

    bool is_mid_inside, is_min_inside, is_max_inside;

    float mid_x = ( node->min_pos.x + node->max_pos.x ) * .5;
    float mid_y = ( node->min_pos.y + node->max_pos.y ) * .5;
    float mid_z = ( node->min_pos.z + node->max_pos.z ) * .5;

    switch ( primitive->get_type() ) {

        case PrimitiveType::GeometricSphere:
        {
            Sphere* sphere = dynamic_cast<Sphere*>(primitive);

            is_max_inside = sphere->is_point_inside( node->max_pos );
            is_mid_inside = sphere->is_point_inside( Vector3(mid_x, mid_y, mid_z) );
            is_min_inside = sphere->is_point_inside( node->min_pos );

            break;
        }
        case PrimitiveType::GeometricCube:
        {
            Cube* cube = dynamic_cast<Cube*>(primitive);

            is_max_inside = cube->is_point_inside( node->max_pos );
            is_mid_inside = cube->is_point_inside( Vector3(mid_x, mid_y, mid_z) );
            is_min_inside = cube->is_point_inside( node->min_pos );
        
            break;
        }
        case PrimitiveType::GeometricCone:
        {
            Cone* cone = dynamic_cast<Cone*>(primitive);

            is_min_inside = cone->is_point_inside( node->min_pos );
            is_mid_inside = cone->is_point_inside( Vector3(mid_x, mid_y, mid_z) );
            is_max_inside = cone->is_point_inside( node->max_pos );

            break;
        }
        case PrimitiveType::GeometricCylinder:
        {
            Cylinder* cylinder = dynamic_cast<Cylinder*>(primitive);
            
            is_min_inside = cylinder->is_point_inside( node->min_pos );
            is_mid_inside = cylinder->is_point_inside( Vector3(mid_x, mid_y, mid_z) );
            is_max_inside = cylinder->is_point_inside( node->max_pos );
            
            break;
        }
    }

    if ( is_mid_inside && is_min_inside && is_max_inside ) return OctreeNodeCode::black;
    if ( is_mid_inside || is_min_inside || is_max_inside ) return OctreeNodeCode::gray;
    return OctreeNodeCode::white;

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
    Cube* cubeA = new Cube( Vector3(-5,-5,-5), Vector3(5,5,5));
    Cylinder* cylinderA = new Cylinder( 5, Vector3(0,10,0), Vector3(0,0,0) );
    Cone* coneA = new Cone( Vector3(0,0,0), 5, Vector3(0,10,0) );

    Octree* octree = new Octree({coneA});

    std::string result = build_tree(coneA, octree->root, 5);

    std::cout << result << std::endl;

    return 0;
}
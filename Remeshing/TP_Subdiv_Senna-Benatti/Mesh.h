#ifndef MESH_H
#define MESH_H

#include <glad/glad.h>
#include <vector>
#include <memory>

#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include <glm/gtx/string_cast.hpp>

#include <map>
#include <set>

#include <iostream>

class Mesh {
public:
  virtual ~Mesh();

  const std::vector<glm::vec3> &vertexPositions() const { return _vertexPositions; }
  std::vector<glm::vec3> &vertexPositions() { return _vertexPositions; }

  const std::vector<glm::vec3> &vertexNormals() const { return _vertexNormals; }
  std::vector<glm::vec3> &vertexNormals() { return _vertexNormals; }

  const std::vector<glm::vec2> &vertexTexCoords() const { return _vertexTexCoords; }
  std::vector<glm::vec2> &vertexTexCoords() { return _vertexTexCoords; }

  const std::vector<glm::uvec3> &triangleIndices() const { return _triangleIndices; }
  std::vector<glm::uvec3> &triangleIndices() { return _triangleIndices; }

  /// Compute the parameters of a sphere which bounds the mesh
  void computeBoundingSphere(glm::vec3 &center, float &radius) const;

  void recomputePerVertexNormals(bool angleBased = false);
  void recomputePerVertexTextureCoordinates( );

  void init();
  void initOldGL();
  void render();
  void clear();

  void addPlan(float square_half_side = 1.0f);

  void subdivideLinear() {
    std::vector<glm::vec3> newVertices = _vertexPositions;
    std::vector<glm::uvec3> newTriangles;

    struct Edge {
      unsigned int a , b;
      Edge( unsigned int c , unsigned int d ) : a( std::min<unsigned int>(c,d) ) , b( std::max<unsigned int>(c,d) ) {}
      bool operator < ( Edge const & o ) const {   return a < o.a  ||  (a == o.a && b < o.b);  }
      bool operator == ( Edge const & o ) const {   return a == o.a  &&  b == o.b;  }
    };
    std::map<Edge , unsigned int> newVertexOnEdge;
    for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
      unsigned int a = _triangleIndices[tIt][0];
      unsigned int b = _triangleIndices[tIt][1];
      unsigned int c = _triangleIndices[tIt][2];


      Edge Eab(a,b);
      unsigned int oddVertexOnEdgeEab = 0;
      if( newVertexOnEdge.find( Eab ) == newVertexOnEdge.end() ) {
        newVertices.push_back( (_vertexPositions[ a ] + _vertexPositions[ b ]) / 2.f );
        oddVertexOnEdgeEab = newVertices.size() - 1;
        newVertexOnEdge[Eab] = oddVertexOnEdgeEab;
      }
      else { oddVertexOnEdgeEab = newVertexOnEdge[Eab]; }


      Edge Ebc(b,c);
      unsigned int oddVertexOnEdgeEbc = 0;
      if( newVertexOnEdge.find( Ebc ) == newVertexOnEdge.end() ) {
        newVertices.push_back( (_vertexPositions[ b ] + _vertexPositions[ c ]) / 2.f );
        oddVertexOnEdgeEbc = newVertices.size() - 1;
        newVertexOnEdge[Ebc] = oddVertexOnEdgeEbc;
      }
      else { oddVertexOnEdgeEbc = newVertexOnEdge[Ebc]; }

      Edge Eca(c,a);
      unsigned int oddVertexOnEdgeEca = 0;
      if( newVertexOnEdge.find( Eca ) == newVertexOnEdge.end() ) {
        newVertices.push_back( (_vertexPositions[ c ] + _vertexPositions[ a ]) / 2.f );
        oddVertexOnEdgeEca = newVertices.size() - 1;
        newVertexOnEdge[Eca] = oddVertexOnEdgeEca;
      }
      else { oddVertexOnEdgeEca = newVertexOnEdge[Eca]; }

      // set new triangles :
      newTriangles.push_back( glm::uvec3( a , oddVertexOnEdgeEab , oddVertexOnEdgeEca ) );
      newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEab , b , oddVertexOnEdgeEbc ) );
      newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEca , oddVertexOnEdgeEbc , c ) );
      newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEab , oddVertexOnEdgeEbc , oddVertexOnEdgeEca ) );
    }

    // after that:
    _triangleIndices = newTriangles;
    _vertexPositions = newVertices;
    recomputePerVertexNormals( );
    recomputePerVertexTextureCoordinates( );
  }

  void subdivideLoop() {
    std::vector<glm::vec3> newVertices = _vertexPositions;
    std::vector<glm::uvec3> newTriangles;

    // Here we create the struct that defines an edge
    struct Edge {
      unsigned int a , b;
      Edge( unsigned int c , unsigned int d ) : a( std::min<unsigned int>(c,d) ) , b( std::max<unsigned int>(c,d) ) {}
      bool operator < ( Edge const & o ) const {   return a < o.a  ||  (a == o.a && b < o.b);  }
      bool operator == ( Edge const & o ) const {   return a == o.a  &&  b == o.b;  }
    };

    std::vector<std::map<Edge, unsigned int>> edgesNeighbors(_vertexPositions.size());
    for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
      unsigned int a = _triangleIndices[tIt][0];
      unsigned int b = _triangleIndices[tIt][1];
      unsigned int c = _triangleIndices[tIt][2];
      Edge Eab(a, b);
      Edge Ebc(b, c);
      Edge Eac(a, c);

      if (edgesNeighbors[a].find(Eab) == edgesNeighbors[a].end()){
        edgesNeighbors[a][Eab] = 1;
      } else {
        edgesNeighbors[a][Eab] = 2;
      }
      if (edgesNeighbors[a].find(Eac) == edgesNeighbors[a].end()){
        edgesNeighbors[a][Eac] = 1;
      } else {
        edgesNeighbors[a][Eac] = 2;
      }


      if (edgesNeighbors[b].find(Eab) == edgesNeighbors[b].end()){
        edgesNeighbors[b][Eab] = 1;
      } else {
        edgesNeighbors[b][Eab] = 2;
      }
      if (edgesNeighbors[b].find(Ebc) == edgesNeighbors[b].end()){
        edgesNeighbors[b][Ebc] = 1;
      } else {
        edgesNeighbors[b][Ebc] = 2;
      }
      
      if (edgesNeighbors[c].find(Eac) == edgesNeighbors[c].end()){
        edgesNeighbors[c][Eac] = 1;
      } else {
        edgesNeighbors[c][Eac] = 2;
      }
      if (edgesNeighbors[c].find(Ebc) == edgesNeighbors[c].end()){
        edgesNeighbors[c][Ebc] = 1;
      } else {
        edgesNeighbors[c][Ebc] = 2;
      }
    }

    // Changing the position of the even vertices    
    for (unsigned int i = 0; i < _vertexPositions.size(); i++) {  
      // I chose this formule to calculate the value of alpha because sometimes I had n < 3 and it caused bugs in the app
      int n = edgesNeighbors[i].size(); 
      float alpha_n = (40.0 - pow(3.0 + 2.0*cos(2.f*M_PI/n), 2))/64.0;   

      
      newVertices[i] *= (1 - alpha_n);

      // If an edge is found only once for a vertice it means this vertice is inside a "open" mesh, i.e. an extraordinary mesh
      bool ordinary = true;
      for (auto it = edgesNeighbors[i].begin(); it != edgesNeighbors[i].end(); ++it) {
        // it->first is and Edge object, so it->first.a returns the a vertex of the Edge 
        //and it->first.b returns the b vertex
        unsigned int neighbor_vertex = (it->first.a == i) ? it->first.b : it->first.a;
        if (ordinary){
          if (it->second == 2){
            newVertices[i] += _vertexPositions[neighbor_vertex]*alpha_n/(float)n;
          }
          else{
            ordinary = false;
            newVertices[i] = _vertexPositions[i]*3.f/4.f + _vertexPositions[neighbor_vertex]/8.f;
          }
        }
        else{
          if(it->second != 2)
            newVertices[i] += _vertexPositions[neighbor_vertex]/8.f;
        }
      }
    }

    std::map< Edge , unsigned int > newVertexOnEdge;
    std::map<unsigned int, unsigned int> oddValence;
    for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
      unsigned int a = _triangleIndices[tIt][0];
      unsigned int b = _triangleIndices[tIt][1];
      unsigned int c = _triangleIndices[tIt][2];

      Edge Eab(a,b);
      unsigned int oddVertexOnEdgeEab = 0;
      if( newVertexOnEdge.find( Eab ) == newVertexOnEdge.end() ) {
        newVertices.push_back( (_vertexPositions[ a ] + _vertexPositions[ b ]) / 2.f );
        oddVertexOnEdgeEab = newVertices.size() - 1;
        newVertexOnEdge[Eab] = oddVertexOnEdgeEab;
        oddValence[oddVertexOnEdgeEab] = c; 
      }
      else {
        oddVertexOnEdgeEab = newVertexOnEdge[Eab];
        newVertices[oddVertexOnEdgeEab] = newVertices[oddVertexOnEdgeEab]*3.f/4.f + 
                                          _vertexPositions[oddValence[oddVertexOnEdgeEab]]/8.f +
                                          _vertexPositions[c]/8.f; 
      }


      Edge Ebc(b,c);
      unsigned int oddVertexOnEdgeEbc = 0;
      if( newVertexOnEdge.find( Ebc ) == newVertexOnEdge.end() ) {
        newVertices.push_back( (_vertexPositions[ b ] + _vertexPositions[ c ]) / 2.f );
        oddVertexOnEdgeEbc = newVertices.size() - 1;
        newVertexOnEdge[Ebc] = oddVertexOnEdgeEbc;
        oddValence[oddVertexOnEdgeEbc] = a; 
      }
      else { oddVertexOnEdgeEbc = newVertexOnEdge[Ebc];
      newVertices[oddVertexOnEdgeEbc] = newVertices[oddVertexOnEdgeEbc]*3.f/4.f +
                                        _vertexPositions[oddValence[oddVertexOnEdgeEbc]]/8.f +
                                        _vertexPositions[a]/8.f; 
      }

      Edge Eca(c,a);
      unsigned int oddVertexOnEdgeEca = 0;
      if( newVertexOnEdge.find( Eca ) == newVertexOnEdge.end() ) {
        newVertices.push_back( (_vertexPositions[ c ] + _vertexPositions[ a ]) / 2.f );
        oddVertexOnEdgeEca = newVertices.size() - 1;
        newVertexOnEdge[Eca] = oddVertexOnEdgeEca;
        oddValence[oddVertexOnEdgeEca] = b; 
      }
      else { oddVertexOnEdgeEca = newVertexOnEdge[Eca];
      newVertices[oddVertexOnEdgeEca] = newVertices[oddVertexOnEdgeEca]*3.f/4.f +
                                        _vertexPositions[oddValence[oddVertexOnEdgeEca]]/8.f +
                                        _vertexPositions[b]/8.f;
      }
      newTriangles.push_back( glm::uvec3( a , oddVertexOnEdgeEab , oddVertexOnEdgeEca ) );
      newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEab , b , oddVertexOnEdgeEbc ) );
      newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEca , oddVertexOnEdgeEbc , c ) );
      newTriangles.push_back( glm::uvec3( oddVertexOnEdgeEab , oddVertexOnEdgeEbc , oddVertexOnEdgeEca ) );
    }

    _triangleIndices = newTriangles;
    _vertexPositions = newVertices;
    recomputePerVertexNormals( );
    recomputePerVertexTextureCoordinates( );
  }





  void subdivideLoop_01(){
    subdivideLinear();
    // TODO: Implement here the Loop subdivision instead of the straightforward Linear Subdivision.
    // You can have a look at the Linear Subdivision function to take some inspiration from it.
    //
    // A few recommendations / advices (note that the following regards a simple implementation that does not handle boundaries, you can adapt it if you want to handle those):
    // I) start by declaring a vector of new positions "newVertices" and a vector of new triangles 
    //    "newTriangles".
    //    Do not mix the new quantities and the old ones.
    //    At the end, replace _vertexPositions by newVertices and _triangleIndices by newTriangles,
    //    just as it is done in subdivideLinear().
    //    This will help you writing clean code.
    //    Remember: In the Loop subdivision scheme, a new position (in the output mesh at level k+1) 
    //    is a linear combination of the old vertices positions (at level k).
    //    So, you should NEVER (!!!!!) have in your code something like: 
    //    newVertices[ v ] += newVertices[ v_neighbor ] * weight;
    // II) Compute the neighbors of all the even vertices. You can use a structure such as 
    //    "std::vector< std::set< unsigned int > > vertex_neighbors" for example.
    //    This will give you the valence n of a given even vertex v, and the value of the 
    //    coefficient alpha_n that you need to use in the computation of the new position for v.
    // III) Compute the new positions for the even vertices. If you compute the even vertices first, 
    //    you will not be tempted to consider the odd vertices as their neighbors (that would 
    //    be a -- very common, mistake).
    // IV) Process all triangles, insert the odd vertices, compute their position using the 
    //    subdivision mask, and create four new triangles per old triangle.
    //    You can get inspiration from subdivideLinear() for that part.
    //
    // Good luck! Do not hesitate asking questions, we are here to help you.
  }

private:
  std::vector<glm::vec3> _vertexPositions;
  std::vector<glm::vec3> _vertexNormals;
  std::vector<glm::vec2> _vertexTexCoords;
  std::vector<glm::uvec3> _triangleIndices;

  GLuint _vao = 0;
  GLuint _posVbo = 0;
  GLuint _normalVbo = 0;
  GLuint _texCoordVbo = 0;
  GLuint _ibo = 0;
};

// utility: loader
void loadOFF(const std::string &filename, std::shared_ptr<Mesh> meshPtr);

#endif  // MESH_H

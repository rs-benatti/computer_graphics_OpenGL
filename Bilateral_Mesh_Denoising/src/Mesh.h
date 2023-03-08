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
#include <cmath>

#include <iostream>

class Mesh {
public:
  virtual ~Mesh();
  std::vector<glm::vec3> _vertexWeightedNormals;
  const int N = 5; // Number of iterations
  float sigma_s = 0.001f; 
  float sigma_c; 
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
    std::cout << "Number of points: " << _vertexPositions.size() << std::endl;
  }

  void bilateralFiltering(){
    std::cout << "Value of sigma_s: " << sigma_s << std::endl;
    if (_noisyVertexPositions.empty()){
      for(unsigned int i = 0 ; i < _vertexPositions.size() ; ++i) {
        _noisyVertexPositions.push_back(_vertexPositions[i]);
      }
    }
    calculateTriangleNeighboord();
    calculateSigmac();
    for (unsigned int j = 0; j < N; ++j){
      calculateDistanceNeighborhood(2.0f * sigma_c);
      calculateTriangleNeighboord();
      calculateTrianglesAreas();
      calculateVertexWeightedNormals();
      for (unsigned int i = 0; i < _vertexPositions.size(); ++i){
        denoisePoint(i);
      }
    }
    std::cout << "Bilateral Filtering Applied" << std::endl;
    computeError();
  }

  void computeError(){
    if (_noNoiseVertexPositions.empty() + _noisyVertexPositions.empty() == 0){
      double sum_noisy = 0;
      double sum_filtered = 0;
      for(unsigned int i = 0 ; i < _vertexPositions.size() ; ++i) {
        sum_noisy += glm::length(_noisyVertexPositions[i] - _noNoiseVertexPositions[i]);
        if (std::isnan(glm::length(_vertexPositions[i] - _noNoiseVertexPositions[i]))){
          std::cout << "Position of the vertex: " << _vertexPositions[i].x << _vertexPositions[i].y << _vertexPositions[i].z << std::endl;
        } else {
          sum_filtered += glm::length(_vertexPositions[i] - _noNoiseVertexPositions[i]);  
        }
        
      }
      std::cout << "Before applying the bilateral filtering we had an error of: " << sum_noisy << std::endl;
      std::cout << "After applying the bilateral filtering we had an error of: " << sum_filtered << std::endl;
    }
  }

  void setSigma_s(float userSigma_s){
    sigma_s = userSigma_s;
  }

  // The variance was really close to 0, so I could only see zeros. If I increase the noise in such a way that it becomes too big I can see some variance
  void calculateVariance(){
    oneRingNeighboorhood.clear();
    for (unsigned int pointIndex = 0; pointIndex < _triangleNeighborhood.size(); ++pointIndex){
      std::vector<unsigned int> points;
      int a = _triangleIndices[_triangleNeighborhood[pointIndex][0]][0];
      int b = _triangleIndices[_triangleNeighborhood[pointIndex][0]][1];
      int c = _triangleIndices[_triangleNeighborhood[pointIndex][0]][2];
      points.push_back(a);
      points.push_back(b);
      points.push_back(c);
      oneRingNeighboorhood.push_back(points);  
      points.clear();

      for(unsigned int triangle = 1; triangle < _triangleNeighborhood[pointIndex].size(); ++triangle){
        int a = _triangleIndices[_triangleNeighborhood[pointIndex][triangle]][0];
        int b = _triangleIndices[_triangleNeighborhood[pointIndex][triangle]][1];
        int c = _triangleIndices[_triangleNeighborhood[pointIndex][triangle]][2];
        int flag_a = 0;
        int flag_b = 0;
        int flag_c = 0;
        for (unsigned int i = 0; i <= oneRingNeighboorhood[pointIndex].size(); i++){
          if (a == oneRingNeighboorhood[pointIndex][i]) flag_a++;
          if (b == oneRingNeighboorhood[pointIndex][i]) flag_b++;
          if (c == oneRingNeighboorhood[pointIndex][i]) flag_c++;
        }
        if (flag_a == 0) oneRingNeighboorhood[pointIndex].push_back(a);
        if (flag_b == 0) oneRingNeighboorhood[pointIndex].push_back(b);
        if (flag_c == 0) oneRingNeighboorhood[pointIndex].push_back(c);
      }
      }
      std::vector<glm::vec3> meanNeighborhoodVertices;
      for (const auto &vec : oneRingNeighboorhood) {
        glm::vec3 mean;
        for (int i : vec) {
            mean += _vertexPositions[i];
        }
        mean = mean/(float)vec.size();
        meanNeighborhoodVertices.push_back(mean);
      }
      for (const auto &vec : oneRingNeighboorhood) {
        float squaredDistancesSum = 0.0f;
        for (int i: vec) {
          glm::vec3 diff = _vertexPositions[i] - meanNeighborhoodVertices[i];
          squaredDistancesSum += dot(diff, diff);
        }
        float variance = squaredDistancesSum / (float)vec.size();
        _variance.push_back(variance);
      }

      for (int var : _variance) {
        //std::cout << var << std::endl;
      }
  }

  void addNormalNoise(){
    calculateTriangleNeighboord();
    calculateTrianglesAreas();
    calculateVertexWeightedNormals();
    for(unsigned int i = 0 ; i < _vertexPositions.size() ; ++i) {
      glm::vec3 normal = _vertexWeightedNormals[i];
      _vertexPositions[i] = _vertexPositions[i] + (normal * (float)(((rand() % 100) - 50) * 0.0001));
    }
    recomputePerVertexNormals( );
    recomputePerVertexTextureCoordinates( );
    calculateTriangleNeighboord();
  }
  
  void calculateSigmac(){
    int pointIndex = rand() % _vertexPositions.size();
    float distance = 0;
    for(unsigned int triangle = 0; triangle < _triangleNeighborhood[pointIndex].size(); ++triangle){
      glm::vec3 a = _vertexPositions[_triangleIndices[_triangleNeighborhood[pointIndex][triangle]][0]];
      glm::vec3 b = _vertexPositions[_triangleIndices[_triangleNeighborhood[pointIndex][triangle]][1]];
      glm::vec3 c = _vertexPositions[_triangleIndices[_triangleNeighborhood[pointIndex][triangle]][2]];
      if (glm::length(a - _vertexPositions[pointIndex])  > distance){
        distance = glm::length(a - _vertexPositions[pointIndex]);
      }
      if (glm::length(b - _vertexPositions[pointIndex])  > distance){
        distance = glm::length(b - _vertexPositions[pointIndex]);
      }
      if (glm::length(c - _vertexPositions[pointIndex])  > distance){
        distance = glm::length(c - _vertexPositions[pointIndex]);
      }
    }    
    sigma_c = distance;
    std::cout << "Value of sigma_c: " << sigma_c << std::endl;
  }

  void addNoise(){
    if (_noNoiseVertexPositions.empty()){
      for(unsigned int i = 0 ; i < _vertexPositions.size() ; ++i) {
        _noNoiseVertexPositions.push_back(_vertexPositions[i]);
      }
    }
    for(unsigned int i = 0 ; i < _vertexPositions.size() ; ++i) {
      _vertexPositions[i].x += ((rand() % 100) - 50) * 0.0001;
      _vertexPositions[i].y += ((rand() % 100) - 50) * 0.0001;
      _vertexPositions[i].z += ((rand() % 100) - 50) * 0.0001;
    }
    recomputePerVertexNormals( );
    recomputePerVertexTextureCoordinates( );
    calculateTriangleNeighboord();
    calculateVariance();
    
  }

  void calculateDistanceNeighborhood(float d){
    _distanceNeighborhood.clear();
    for(unsigned int i = 0 ; i < _vertexPositions.size() ; ++i) {
      std::vector<unsigned int> neighboors;
      for(unsigned int j = 0 ; j < _vertexPositions.size() ; ++j) {
        if (i==j) continue;
        float dist = sqrt((_vertexPositions[i].x - _vertexPositions[j].x) * (_vertexPositions[i].x - _vertexPositions[j].x) +
                          (_vertexPositions[i].y - _vertexPositions[j].y) * (_vertexPositions[i].y - _vertexPositions[j].y) +
                          (_vertexPositions[i].z - _vertexPositions[j].z) * (_vertexPositions[i].z - _vertexPositions[j].z));
        if (dist <= d){
          neighboors.push_back(j);
        }
      }
      _distanceNeighborhood.push_back(neighboors);
    }
  }

  void calculateTriangleNeighboord(){
    _triangleNeighborhood.clear();
    for(unsigned int i = 0 ; i < _vertexPositions.size() ; ++i) {
      std::vector<unsigned int> neighboors;
      for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
        unsigned int a = _triangleIndices[tIt][0];
        unsigned int b = _triangleIndices[tIt][1];
        unsigned int c = _triangleIndices[tIt][2];
        if (a == i || b == i || c == i){
          neighboors.push_back(tIt);
        }
      }
      _triangleNeighborhood.push_back(neighboors);
    }
  
  }

  void calculateTrianglesAreas(){
    _triangleNormals.clear();
    _triangleArea.clear();
    for(unsigned int tIt = 0 ; tIt < _triangleIndices.size() ; ++tIt) {
      glm::vec3 a = _vertexPositions[_triangleIndices[tIt][0]];
      glm::vec3 b = _vertexPositions[_triangleIndices[tIt][1]];
      glm::vec3 c = _vertexPositions[_triangleIndices[tIt][2]];
      glm::vec3 ab = b - a;
      glm::vec3 ac = c - a;

      glm::vec3 normal = glm::cross(ab, ac);
      float area = glm::length(normal) / 2.0f;
      normal = normal/glm::length(normal); // Normalizing the normals
      _triangleNormals.push_back(normal);
      _triangleArea.push_back(area);
    }
  }

  void calculateVertexWeightedNormals(){
    _vertexWeightedNormals.clear();
    for(unsigned int i = 0; i < _triangleNeighborhood.size(); ++i){
           
      float totalArea = 0;
      glm::vec3 weightedNormal = glm::vec3(0.0f, 0.0f, 0.0f);
      
      for(unsigned int triangle = 0; triangle < _triangleNeighborhood[i].size(); ++triangle){
        totalArea += _triangleArea[_triangleNeighborhood[i][triangle]];
        weightedNormal += _triangleNormals[_triangleNeighborhood[i][triangle]]*_triangleArea[_triangleNeighborhood[i][triangle]];
      }

      
        weightedNormal = weightedNormal/totalArea;
      _vertexWeightedNormals.push_back(weightedNormal);      
    }
  }

  void denoisePoint(int vertexIndex){
    glm::vec3 point = _vertexPositions[vertexIndex];
    
    std::vector<unsigned int> Q = _distanceNeighborhood[vertexIndex];
    float weighted_sum = 0;
    float normalizer = 0;
    glm::vec3 normal = _vertexWeightedNormals[vertexIndex];
    double t = 0, h = 0, w_c = 0, w_s = 0, h_sum = 0;
    for (unsigned int i = 0; i < Q.size(); ++i){
      glm::vec3 neighboor = _vertexPositions[Q[i]];
      t = glm::length(point - neighboor);
      h = glm::dot(neighboor - point, normal);
      h_sum += h;     
      w_c = exp(-t*t/(2.0f*sigma_c*sigma_c));
      w_s = exp(-h*h/(2.0f*sigma_s*sigma_s)); 
      weighted_sum += (w_c*w_s)*h;
      normalizer += w_c*w_s;
    }
    if (!Q.empty()){
      _vertexPositions[vertexIndex] = point + (normal * (weighted_sum/normalizer));
      if(std::isnan(_vertexPositions[vertexIndex].x)){
        std::cout << "NaN problem!!" << std::endl;
        //_vertexPositions[vertexIndex] = point + (normal * (float)h_sum/(float)Q.size());
        _vertexPositions[vertexIndex] = point;
      }
    }
    
  }

private:
  std::vector<glm::vec3> _vertexPositions;
  std::vector<glm::vec3> _noNoiseVertexPositions;
  std::vector<glm::vec3> _noisyVertexPositions;
  std::vector<glm::vec3> _vertexNormals;
  std::vector<glm::vec2> _vertexTexCoords;
  std::vector<glm::uvec3> _triangleIndices;
  std::vector<std::vector<unsigned int>> _distanceNeighborhood;
  std::vector<std::vector<unsigned int>> _triangleNeighborhood;
  std::vector<float> _triangleArea;
  std::vector<glm::vec3> _triangleNormals;
  std::vector<std::vector<unsigned int>> oneRingNeighboorhood;
  std::vector<unsigned int> _variance;

  GLuint _vao = 0;
  GLuint _posVbo = 0;
  GLuint _normalVbo = 0;
  GLuint _texCoordVbo = 0;
  GLuint _ibo = 0;
};

// utility: loader
void loadOFF(const std::string &filename, std::shared_ptr<Mesh> meshPtr);
#endif  // MESH_H

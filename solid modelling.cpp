#include <iostream>
using std::cout;
using std::endl;
using std::cin;
#include <cmath>
#include <fstream>
using std::ofstream;
using std::ifstream;

const int horizontalResolution = 1920;
const int verticalResolution = 1080;
const double epsilon = 0.001;

const double xyzVertexList [8][3] = {{ 0,  0,  0},  // southeast floor
                                  {25,  0,  0},  // southwest floor
                                  {25, 10,  0},  // southwest ceiling
                                  { 0, 10,  0},  // southeast ceiling
                                  { 0,  0, 40},  // northeast floor
                                  {25,  0, 40},  // northwest floor
                                  {25, 10, 40},  // northwest ceiling
                                  { 0, 10, 40}}; // northeast ceiling
                            
// no edge list (no lines will be drawn)

const int faceList [6][4] = {{0, 1, 2, 3},  // south (front) wall (w door)
                             {7, 6, 5, 4},  // north (back) wall (w windows)
                             {0, 3, 7, 4},  // east (left) wall (w chalkboard)
                             {5, 6, 2, 1},  // west (right) wall (w windows)
                             {7, 3, 2, 6},  // ceiling
                             {0, 4, 5, 1}}; // floor

const char faceColors [6][3] = {{  0, 255,   0},  // front wall = green
                                {  0, 255,   0},  // back wall = green
                                {  0,   0, 255},  // left wall = red
                                {  0,   0, 255},  // right wall = red
                                {255,   0,   0},  // ceiling = blue
                                {255,   0,   0}}; // floor = blue
                   
enum rgb {R, G, B};
                       
enum xyzCoordinates {X, Y, Z};
enum uvnCoordinates {U, V, N};
                       
void crossProduct (const double a[3], const double b[3], double c[3]) // c = a x b
{
    c[X] =   a[Y]*b[Z] - a[Z]*b[Y];
    c[Y] = -(a[X]*b[Z] - a[Z]*b[X]);
    c[Z] =   a[X]*b[Y] - a[Y]*b[X];
    return;
}  // end function crossProduct               
   
struct Plane
{
    double n [3];
    double d;
};

void subtract (const double a[3], const double b[3], double c[3]) // c = a - b
//void subtract (double* a,   b,    c)
{
    for (int i = X; i <= Z; i++)
       c[i] = a[i] - b[i];
    return;
} // end function subtract

double dotProduct (const double a[3], const double b[3]) // dp = a dot b
{
    double result = 0;
    for (int i = X; i <= Z; i++)
       result += a[i] * b[i];
    return result;
}  // end function dotProduct

double magnitude (const double a[3]) // |a|
{
    return sqrt (a[X]*a[X] + a[Y]*a[Y] + a[Z]*a[Z]);
}  // end function magnitude

void normalize (const double a[3], double b[3])  // b = a / |a|
{
    for (int i = X; i <= Z; i++)
       b[i] = a[i] / magnitude(a);
}  // end function normalize

bool inside (const double hitPoint[3], const int face[4], const double uvnVertexList[8][3])
{
    double edgeVectors[4][3]; // from each corner to the next corner
    for (int i = 0; i <= 3; i++)
       for (int j = U; j <= N; j++)
          edgeVectors[i][j] =   uvnVertexList[face[(i+1)%4]][j]
                              - uvnVertexList[face[ i     ]][j];
          
    double hitPointVectors[4][3]; // from each corner to hitPoint
    for (int i = 0; i <= 3; i++)
       for (int j = U; j <= N; j++)
          hitPointVectors[i][j] =   hitPoint[j]
                                  - uvnVertexList[face[i]][j];

    double cPs[4][3];
    for (int i = 0; i <= 3; i++)
       crossProduct (edgeVectors[i], hitPointVectors[i], cPs[i]);
       
     return (    (    cPs[0][U] > +epsilon  &&  cPs[1][U] > +epsilon
                  &&  cPs[2][U] > +epsilon  &&  cPs[3][U] > +epsilon)
             ||  (    cPs[0][U] < -epsilon  &&  cPs[1][U] < -epsilon
                  &&  cPs[2][U] < -epsilon  &&  cPs[3][U] < -epsilon)
             ||  (    cPs[0][V] > +epsilon  &&  cPs[1][V] > +epsilon
                  &&  cPs[2][V] > +epsilon  &&  cPs[3][V] > +epsilon)
             ||  (    cPs[0][V] < -epsilon  &&  cPs[1][V] < -epsilon
                  &&  cPs[2][V] < -epsilon  &&  cPs[3][V] < -epsilon)
             ||  (    cPs[0][N] > +epsilon  &&  cPs[1][N] > +epsilon
                  &&  cPs[2][N] > +epsilon  &&  cPs[3][N] > +epsilon)
             ||  (    cPs[0][N] < -epsilon  &&  cPs[1][N] < -epsilon
                  &&  cPs[2][N] < -epsilon  &&  cPs[3][N] < -epsilon));
}  // end function inside

void createPlanes (double uvnVertexList [8][3], Plane planes[6])
{
    for (int i = 0; i <= 5; i++)  // one plane per face
    {
       double v1 [3], v2[3];
       subtract (uvnVertexList[faceList[i][1]],
                 uvnVertexList[faceList[i][0]], v1);
       subtract (uvnVertexList[faceList[i][2]],
                 uvnVertexList[faceList[i][1]], v2);
       crossProduct (v1, v2, planes[i].n);
       planes[i].d = dotProduct (planes[i].n,
                                     uvnVertexList[faceList[i][0]]);
    }  // end for i = 0 to 5
   return;    
}  // end function createPlanes

struct CoordinateFrame
{
    double origin [3];
    double u[3];
    double v[3];
    double n[3];
};
struct SyntheticCamera
{
    CoordinateFrame frame;
    double eye [3];
    double uL;
    double uR;
    double vB;
    double vT;
};

void setUpSyntheticCamera (SyntheticCamera& synCam) 
{
    synCam.frame.origin[X] = 24;  // camera VRP
    synCam.frame.origin[Y] =  5;
    synCam.frame.origin[Z] = 20;
    
    double vpn [3]; // from camera to pt being looked at (0, 5, 0)
    vpn[X] =  0 - synCam.frame.origin[X];
    vpn[Y] =  5 - synCam.frame.origin[Y];
    vpn[Z] =  0 - synCam.frame.origin[Z];
    
    normalize (vpn, synCam.frame.n);
    
    double vup [3] = {0, 1, 0}; // default = opposite gravity
    
    crossProduct (synCam.frame.n, vup, synCam.frame.u);
    
    crossProduct (synCam.frame.u, synCam.frame.n, synCam.frame.v);

    int e = 1;
    synCam.eye[U] = 0;
    synCam.eye[V] = 0;
    synCam.eye[N] = -e;  // choose eye position; E = 1; eye = {0, 0, -E}
    
//    cout << "enter e: ";
//    cin >> synCam.e;  // choose eye position; E = 1; eye = {0, 0, -E}
       
    synCam.uL = -0.885;
    synCam.uR = +0.885;
    synCam.vB = -0.5;
    synCam.vT = +0.5;
    
    double aspectRatio =   (synCam.uR - synCam.uL)
                         / (synCam.vT - synCam.vB);
    cout << aspectRatio;
    // if aspectRatio not close to 16/9, cout warning

   return;
}

void changeCoordinateFrame (CoordinateFrame frame,
                            double uvnVertexList [8][3])
{
    for (int i = 0; i <= 7; i++)
    {
        double pOffset [3];
        for (int j = X; j <= Z; j++)
           pOffset[j] = xyzVertexList[i][j] - frame.origin[j];
        uvnVertexList[i][X] = dotProduct (frame.u, pOffset);
        uvnVertexList[i][Y] = dotProduct (frame.v, pOffset);
        uvnVertexList[i][Z] = dotProduct (frame.n, pOffset);
    }
    return;
}  // end function changeCoordinateFrame

void fileSetUp (ofstream& imageFile)
{    
    ifstream testFile ("cs6823paint.bmp");
    char header [54];
    testFile.read (header, 54);
    testFile.close ();
    
    imageFile.open ("cs6823try2.bmp");
    imageFile.write (header, 54);
    
    return;
}   // end function fileSetUp

void checkFaceForIntersection (Plane plane, SyntheticCamera synCam,
                               double lineOfSight[3], double u, double v,
                               const int faceList [4],
                               const double uvnVertexList[8][3],
                               int k,
                               double& tHmin, int& kMin)
{
   double numerator = plane.d - plane.n[N]*synCam.eye[N];
   double denominator = dotProduct (plane.n, lineOfSight);
   if ( fabs(denominator) > epsilon) // not close to zero
   {
      double tH = numerator / denominator;
      double pH [3]; // parametric line is from eye through (u, v, 0)
                     //   p(t) = (1-t)*eye + t*uv0
                     //        = (1-t)*(0,0,eN) + t*(u,v,0)
                     //        = (ut, vt, (1-t)*eN)
      pH[U] = u * tH;
      pH[V] = v * tH;
      pH[N] = (1 - tH) * synCam.eye[N];
      if (tH > 0 && tHmin > tH  &&  inside (pH, faceList, uvnVertexList))
      {
         tHmin = tH; // want smallest positive tH
         kMin = k;
      }  // end if tH is new min  && pt is inside
   }  // end if line of sight not parallel to plane
   return;
}  // end function checkFaceForIntersection
    
void traceOneRay (double u, double v, SyntheticCamera synCam,
                  Plane planes[6], double uvnVertexList[8][3], int &kMin)
{
    double uv0 [3] = {u, v, 0};
    double lineOfSight [3];
    subtract (uv0, synCam.eye, lineOfSight);
    double tHmin = 10000000; // impossibly large hit time
    kMin = -1;
    for (int k = 0; k <= 5; k++) // each rectangle
       checkFaceForIntersection (planes[k], synCam,
                                 lineOfSight, u, v, faceList[k],
                                 uvnVertexList, k, tHmin, kMin);
    return;
}  // end function traceOneRay

void generateImage (SyntheticCamera synCam,
                    double uvnVertexList [8][3],
                    Plane planes [6])
{
    double deltaU = (synCam.uR - synCam.uL) / horizontalResolution;
    double deltaV = (synCam.vT - synCam.vB) / verticalResolution;
    
    ofstream imageFile;
    fileSetUp (imageFile);
      
    double v = synCam.vB;
    for (int j = 0; j <= verticalResolution - 1; j++, v += deltaV)
    {
       double u = synCam.uL;
       for (int i = 0; i <= horizontalResolution - 1; i++, u += deltaU)
       {
          int kMin;
          traceOneRay (u, v, synCam, planes, uvnVertexList, kMin);
          imageFile.write (faceColors[kMin], 3);
       }
    }  // end for j
    imageFile.close ();
    return;
}  // end function GenerateImage

int main ()
{
    SyntheticCamera synCam;
    setUpSyntheticCamera (synCam);
    
    double uvnVertexList [8][3];
    changeCoordinateFrame (synCam.frame, uvnVertexList);
    
    Plane planes [6];
    createPlanes (uvnVertexList, planes);
    
    generateImage (synCam, uvnVertexList, planes);
         
    return 0;
}   // end function main
#include <iostream>
#include <SDL2/SDL.h>
#include <vector>
#include <algorithm>
#include <cmath>

const float PI = 3.141592653;
const int WIDTH = 1600, HEIGHT = 900;

std::vector<std::vector<float>> matrixMultiply(const std::vector<std::vector<float>>& matrix1,
                                               const std::vector<std::vector<float>>& matrix2) {
    int m = matrix1.size();
    int n = matrix2[0].size();
    int p = matrix2.size();

    std::vector<std::vector<float>> result(m, std::vector<float>(n, 0));

    for (int i = 0; i < m; ++i) {
        for (int j = 0; j < n; ++j) {
            for (int k = 0; k < p; ++k) {
                result[i][j] += matrix1[i][k] * matrix2[k][j];
            }
        }
    }

    return result;
}

bool checkBboxCollision(std::vector<float> A, std::vector<float> B){
    if (A[0] <= B[1] && A[1] >= B[0]){
        if (A[2] <= B[3] && A[3] >= B[2]){
            if (A[4] <= B[5] && A[5] >= B[4]){
                return true;
            }
        }
    }
    return false;
}

class Vector3{
    public:
        float x;
        float y;
        float z;
        Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z){}

        float mag(){
            return sqrt(pow(x,2) + pow(y,2) + pow(z,2));
        }

        Vector3 normalize(){
            float magnitude = mag();
            return Vector3(x / magnitude, y / magnitude, z / magnitude);
        }
    
        Vector3 operator+(const Vector3& v){
            return Vector3(x + v.x, y + v.y, z + v.z);
        }

        Vector3 operator-(const Vector3& v){
            return Vector3(x - v.x, y - v.y, z - v.z);
        }

        Vector3 operator*(const float n){
            return Vector3(x * n, y * n, z  * n);
        }

        Vector3 operator/(const float n){
            return Vector3(x / n, y / n, z / n);
        }

        Vector3 operator+=(const Vector3& v){
            x += v.x;
            y += v.y;
            z += v.z;
        }

        float dot(const Vector3& v){
            return (x*v.x) + (y*v.y) + (z*v.z);
        }

        Vector3 cross(const Vector3& v){
            return Vector3((y * v.z) - (z * v.y), (z * v.x) - (x * v.z), (x * v.y) - (y * v.x));
        }

        float distSquared(const Vector3& v){
            return pow(x - v.x, 2) + pow(y - v.y, 2) + pow(z - v.z, 2);
        }
};

class Vector2{
    public:
        float x, y;
        Vector2(float X, float Y) : x(X), y(Y){}

        float mag(){
            return sqrt(pow(x,2) + pow(y,2));
        }

        Vector2 normalize(){
            float magnitude = mag();
            return Vector2(x / magnitude, y / magnitude);
        }
    
        Vector2 operator+(const Vector2& v){
            return Vector2(x + v.x, y + v.y);
        }

        Vector2 operator-(const Vector2& v){
            return Vector2(x - v.x, y - v.y);
        }

        Vector2 operator*(const float n){
            return Vector2(x * n, y * n);
        }

        Vector2 operator/(const float n){
            return Vector2(x / n, y / n);
        }

        float dot(const Vector2& v){
            return (x*v.x) + (y*v.y);
        }

        float distSquared(const Vector2& v){
            return pow(x - v.x, 2) + pow(y - v.y, 2);
        }
};

class Camera{
    public:
        Vector3 pos;
        Vector3 rot;
        float fov = PI * 0.5;
        float near = 0.01;
        float far = 1000;
        float aspect_ratio;
        std::vector<std::vector<float>> projectionMatrix = {{-0.0f}};

        Camera(Vector3& _pos, Vector3& _rot, float aspectRatio = 16/9) : pos(_pos), rot(_rot), aspect_ratio(aspectRatio){}

        Vector3 fV(){
            return Vector3(cos(rot.y)*cos(rot.x), sin(rot.x), sin(rot.x)*cos(rot.x));
        }

        Vector3 rV(){
            return fV().cross(Vector3(0, 1, 0)).normalize();
        }

        Vector3 uV(){
            return rV().cross(fV()).normalize();
        }

        std::vector<std::vector<float>> transformationMatrix(){
            Vector3 frwd = fV(), rt = rV(), up = uV();
            std::vector<std::vector<float>> viewMatrix = {
                {rt.x, rt.y, rt.z, pos.dot(rt) * -1},
                {up.x, up.y, up.z, pos.dot(up) * -1},
                {frwd.x, frwd.y, frwd.z, pos.dot(frwd) * -1},
                {0, 0, 0, 1}
            };

            if (projectionMatrix.size() == 1){
                float h = tan(fov / 2) * near;
                float w = h * aspect_ratio;
                projectionMatrix = {
                    {2 * near / w, 0, 0, 0},
                    {0, 2 * near / h, 0, 0},
                    {0, 0, -1*(far + near) / (far - near), -2 * far * near / (far - near)},
                    {0, 0, -1, 0}
                };
            }

            return matrixMultiply(projectionMatrix, viewMatrix);
        }

        Vector2 project(const Vector3& p, SDL_Window* window){
            int width, height;
            SDL_GetWindowSize(window, &width, &height);

            std::vector<std::vector<float>> point = {{p.x},{p.y},{p.z},{1}};
            std::vector<std::vector<float>> hPoint = matrixMultiply(transformationMatrix(), point);
            Vector2 nDc = Vector2(hPoint[0][0]/hPoint[3][0], hPoint[1][0]/hPoint[3][0]);
            Vector2 viewPoint = Vector2(((nDc.x + 1)/2)*width, ((1 - nDc.y)/2)*height);
            return viewPoint;
        }

        bool tryCulling(std::vector<Vector3> triPoints, Vector3 triNormal){
            Vector3 cToTri = triPoints[0] - pos;
            if (cToTri.dot(triNormal) >= 0){return true;}

            for(Vector3 point : triPoints){
                if((point - pos).dot(fV()) <= 0){
                    return true;
                }
            }
            return false;
        }
};

class Tri{
    public:
        std::vector<Vector3> points;
        SDL_Color color;
        bool flippedNormal = false;

        Tri(std::vector<Vector3> _points, SDL_Color _color) : points(_points), color(_color){}

        Vector3 normal(){
            if (!flippedNormal){
                return (points[1] - points[0]).cross(points[2] - points[0]).normalize();
            }else{if(flippedNormal){
                return (points[0] - points[1]).cross(points[0] - points[2]).normalize();
            }}
            
        }

        Vector3 centroid(){
            return (points[0] + points[1] + points[2]) / 3;
        }

        void draw(SDL_Window* win, SDL_Renderer* ren, Camera& camera){
            if(!camera.tryCulling(points, normal())){
                std::vector<Vector2> pointConv(3, Vector2(0, 0));
                for(int i=0;i<points.size();i++){
                    pointConv[i] = camera.project(points[i], win);
                }
                std::vector<SDL_Vertex> pointConv2(3, SDL_Vertex{});
                for(int i=0;i<pointConv.size();i++){
                    pointConv2[i] = {SDL_FPoint{pointConv[i].x, pointConv[i].y}, color, SDL_FPoint{0}};
                }
                SDL_RenderGeometry(ren, nullptr, pointConv2.data(), pointConv2.size(), nullptr, 0);
            }
        }
};

class Object{
    public:
        std::vector<Vector3> verts;
        std::vector<SDL_Color> triColors;
        std::vector<Tri> tris;

        Object(std::vector<Vector3> _verts, std::vector<SDL_Color> _triColors) : verts(_verts), triColors(_triColors){
            createTris();
            checkNormals();
        }

        void checkNormals(){
            for (Tri t: tris){
                if(t.normal().dot(t.centroid() - centroid()) <= 0){
                    t.flippedNormal = !t.flippedNormal;
                }
            }
        }

        std::vector<Tri> createTris(){
            std::vector<Tri> objectTris(floor(verts.size()/3), Tri({Vector3(0, 0, 0)}, {}));
            for(int i=0;i<objectTris.size();i++){
                objectTris[i] = Tri({verts[3*i], verts[3*i+1], verts[3*i+2]}, triColors[i]);
            }
            tris = objectTris;
        }

        Vector3 centroid(){
            Vector3 centroid = Vector3(0, 0, 0);
            for(Tri t : tris){
                centroid += t.centroid();
            }
            return centroid / tris.size();
        }

        std::vector<float> bBox(){
            std::vector<float> rX(tris.size()*3, 0);
            std::vector<float> rY(tris.size()*3, 0);
            std::vector<float> rZ(tris.size()*3, 0);
            for(int t=0;t<tris.size();t++){
                for(int p=0;p<tris[t].points.size();p++){
                    rX[t*3 + p] = tris[t].points[p].x;
                    rY[t*3 + p] = tris[t].points[p].y;
                    rZ[t*3 + p] = tris[t].points[p].z;
                }
            }
            std::vector<float> bb = {*min_element(rX.begin(), rX.end()), *max_element(rX.begin(), rX.end()),
                                     *min_element(rY.begin(), rY.end()), *max_element(rY.begin(), rY.end()),
                                     *min_element(rZ.begin(), rZ.end()), *max_element(rZ.begin(), rZ.end())};
            
            return bb;
        }

        void move(const Vector3& moveVector){
            for(Tri t : tris){
                for(int p=0;p<t.points.size();p++){
                    t.points[p] += moveVector;
                }
            }
            checkNormals();
        }

        void rotate(const Vector3& rotationVector){
            std::vector<std::vector<float>> rotationZ = {{cos(rotationVector.z), -1*sin(rotationVector.z), 0},
                                                         {sin(rotationVector.z), cos(rotationVector.z), 0},
                                                         {0, 0, 1}};
            std::vector<std::vector<float>> rotationY = {{cos(rotationVector.y), 0, sin(rotationVector.y)},
                                                         {0, 1, 0},
                                                         {-1*sin(rotationVector.y), 0, cos(rotationVector.y)}};
            std::vector<std::vector<float>> rotationX = {{1, 0, 0},
                                                         {0, cos(rotationVector.x), -1*sin(rotationVector.x)},
                                                         {0, sin(rotationVector.x), cos(rotationVector.x)}};
            
            std::vector<std::vector<float>> rotationMatrix = matrixMultiply(matrixMultiply(rotationZ, rotationY), rotationX);

            for(Tri t: tris){
                for(int p=0;p<t.points.size();p++){
                    Vector3 relativePoint = t.points[p] - centroid();
                    std::vector<std::vector<float>> rotatedPoint = matrixMultiply(rotationMatrix, {{relativePoint.x},{relativePoint.y},{relativePoint.z}});
                    t.points[p] = Vector3(rotatedPoint[0][0], rotatedPoint[1][0], rotatedPoint[2][0]);
                }
            }
            checkNormals();
        }


};
class Scene{
    public:
        std::vector<Object> objects;
        int numTris = 0;
        std::vector<Tri> sceneTris;
        Scene(std::vector<Object> _objects = {}) : objects(_objects){
            for(Object o : objects){
                for (Tri t : o.tris){
                    numTris++;
                }
            }
            std::vector<Tri> localTris(numTris, Tri({Vector3(0, 0, 0)}, {0, 0, 0}));
            for(Object o : objects){
                for(Tri t : o.tris){
                    localTris.insert(localTris.end(), t);
                }
            }
            sceneTris = localTris;
        }



};

int main(int argc, char* argv[]){
    SDL_Init(SDL_INIT_EVERYTHING);
    SDL_Window* window = SDL_CreateWindow("main", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WIDTH, HEIGHT, SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);


    if(NULL == window){
        std::cout << "Could not create window: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Event windowEvent;

    while(true){
        if(SDL_PollEvent(&windowEvent)){
            if(SDL_QUIT == windowEvent.type){break;}
        }

        SDL_Surface* surface = SDL_GetWindowSurface(window);
        Uint32 white = SDL_MapRGB(surface->format, 255, 255, 255);
        SDL_FillRect(surface, NULL, white);
        SDL_UpdateWindowSurface(window);
    }

    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}

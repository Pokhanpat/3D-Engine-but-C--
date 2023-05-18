#include <iostream>
#include <SDL2/SDL.h>
#include <vector>
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
        Vector3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

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

        Tri(std::vector<Vector3> _points, SDL_Color _color) : points(_points), color(_color) {}

        Vector3 normal(){
            return (points[1] - points[0]).cross(points[2] - points[0]).normalize();
        }

        Vector3 centroid(){
            return (points[0] + points[1] + points[2]) / 3;
        }
};



int main(int argc, char* argv[]){
    SDL_Init(SDL_INIT_EVERYTHING);
    SDL_Window* window = SDL_CreateWindow("main", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WIDTH, HEIGHT, SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    std::cout << sqrt(2.0);

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
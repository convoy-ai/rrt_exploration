#include "functions.h"


// rdm class, to generate random float numbers
rdm::rdm() {
    i = time(0);
}
float rdm::randomize() {
    i = i + 1;
    srand (i);
    return float(rand())/float(RAND_MAX);
}



// Norm function
float Norm(std::vector<float> x1,std::vector<float> x2) {
    float a2 = pow((x2[0]-x1[0]),2);
    float b2 = pow((x2[1]-x1[1]),2);
    return pow((a2+b2),0.5);
}


// sign function
float sign(float n) {
    if (n < 0.0) {
        return -1.0;
    }
    else {
        return 1.0;
    }
}


// Nearest function, to find the nearest vector in V to x
std::vector<float> Nearest(std::vector<std::vector<float>> V, std::vector<float> x) {
    float min = Norm(V[0], x);
    int min_index = 0;
    float temp;

    for (int j=0; j < V.size(); j++) {
        temp = Norm(V[j], x);
        if (temp <= min){
            min = temp;
            min_index = j;
        }
    }

    return V[min_index];
}



// Steer function, to compute the new vector x_new
std::vector<float> Steer(std::vector<float> x_nearest, std::vector<float> x_rand, float eta) {
    std::vector<float> x_new;

    if (Norm(x_nearest, x_rand) <= eta) {
        // x_rand is within eta distance of x_nearest
        // so x_new is x_rand
        x_new.push_back(x_rand[0]);
        x_new.push_back(x_rand[1]);
    } else if (x_rand[0] == x_nearest[0]) {
        // x_rand and x_nearest are on the same vertical line
        // so x_new is on the same vertical line, eta distance away from x_nearest
        x_new.push_back(x_nearest[0]);
        x_new.push_back(x_nearest[1] + (sign(x_rand[1]-x_nearest[1])) * eta);
    } else {
        // compute gradient between x_nearest and x_rand
        float m = (x_rand[1]-x_nearest[1])/(x_rand[0]-x_nearest[0]);

        // x_new is on the line between x_nearest and x_rand, a fraction of eta distance away from x_nearest
        x_new.push_back(  (sign(x_rand[0]-x_nearest[0])) * (   sqrt( (pow(eta,2)) / ((pow(m,2))+1) )   ) + x_nearest[0] );
        x_new.push_back(  m * (x_new[0]-x_nearest[0]) + x_nearest[1] );
    }

    return x_new;
}





// gridValue function
int gridValue(nav_msgs::OccupancyGrid &mapData, std::vector<float> Xp) {

    float resolution = mapData.info.resolution;
    float originX = mapData.info.origin.position.x;
    float originY = mapData.info.origin.position.y;

    float width = mapData.info.width;
    std::vector<signed int> Data = mapData.data;

    // returns grid value at "Xp" location
    // map data: 100 occupied, -1 unknown, 0 free
    float index = floor((Xp[1]-originY)/resolution) * width  + floor((Xp[0]-originX)/resolution);

    return Data[int(index)];
}




// ObstacleFree function
int ObstacleFree(std::vector<float> x_near, std::vector<float> &x_new, nav_msgs::OccupancyGrid map){
    float rez = float(map.info.resolution) * 0.2;
    float stepz = int(ceil(Norm(x_new, x_near)) / rez);
    std::vector<float> x_i = x_near;
    int obs=0; int unk=0;

    for (int c=0; c < stepz; c++) {
        x_i = Steer(x_i, x_new, rez);

        if (gridValue(map,x_i) == 100) {
            obs = 1;
        }

        if (gridValue(map,x_i) == -1){
            unk = 1;
            break;
        }
    }

    char out = 0;
    x_new = x_i;

    if (unk == 1) {
        out = -1;
    }

    if (obs == 1) {
        out = 0;
    }

    if (obs != 1 && unk != 1) {
        out = 1;
    }

    return out;
}

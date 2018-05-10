#include "visualizer.h"
#include "Astar.h"
#include <fstream>
#include <string>
#include <vector>
#include <exception>
#include <iostream> 



#ifdef _MSC_VER
static const char* PATH_SEP = "\\";
#else
static const char* PATH_SEP = "/";
#endif

// Bits used in the overrides image bytes
enum OverrideFlags
{
    OF_RIVER_MARSH = 0x10,
    OF_INLAND = 0x20,
    OF_WATER_BASIN = 0x40
};

// Some constants
enum {
    IMAGE_DIM = 2048, // Width and height of the elevation and overrides image
    
    ROVER_X = 159,
    ROVER_Y = 1520,
    BACHELOR_X = 1303,
    BACHELOR_Y = 85,
    WEDDING_X = 1577,
    WEDDING_Y = 1294,
    TIME_PENALIZE = 60 // penalization
};

std::ifstream::pos_type fileSize(const std::string& filename)
{
    std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
    if (!in.good())
    {
        throw std::exception();
    }
    return in.tellg(); 
}

std::vector<uint8_t> loadFile(const std::string& filename, size_t expectedFileSize)
{
    size_t fsize = fileSize(filename);
    if (fsize != expectedFileSize)
    {
        throw std::exception();
    }
    std::vector<uint8_t> data(fsize);
    std::ifstream ifile(filename, std::ifstream::binary);
    if (!ifile.good())
    {
        throw std::exception();
    }
    ifile.read((char*)&data[0], fsize);
    return data;
}

bool donut(int x, int y, int x1, int y1)
{
    int dx = x - x1;
    int dy = y - y1;
    int r2 = dx * dx + dy * dy;
    return r2 >= 150 && r2 <= 400;
}


// we update the results in each path

void UpdateResults(const int* path, int index_start, int index_goal, uint8_t* &result )
{

    // the queau has the last element that 
    // are the optimal path
    while(index_start != index_goal)
    {
        int y = index_goal/IMAGE_DIM;
        int x = index_goal%IMAGE_DIM;


        result[index_goal] = 255;

        index_goal = path[index_goal];

    }          


}

/* main program */
int main(int argc, char** argv)
{

    const size_t expectedFileSize = IMAGE_DIM * IMAGE_DIM;
    // Address assets relative to application location
    std::string anchor = std::string(".") + PATH_SEP;
    std::string pname = argv[0];
    auto lastpos = pname.find_last_of("/\\");
    if (lastpos != std::string::npos)
    {
        anchor = pname.substr(0, lastpos) + PATH_SEP;
    }
    auto elevation = loadFile(anchor + "assets" + PATH_SEP + "elevation.data", expectedFileSize);
    auto overrides = loadFile(anchor + "assets" + PATH_SEP + "overrides.data", expectedFileSize);
    std::ofstream of("pic.bmp", std::ofstream::binary);
    // results contains the path
    auto results = new uint8_t[expectedFileSize];
    memset(results, 0, sizeof(results));

    
    auto grid = new float[expectedFileSize];

    //indexes for our points for the path planner
    int idx_rover;
    int idx_bachelor;
    int idx_wed;

    // output our path
    auto path = new int[expectedFileSize];

    float cost1;// 
    float cost2;// 
    // values infinites per water and sea infinity
    const float INF = std::numeric_limits<float>::infinity();


    // prepare data or grid map
    // we put 1 second to each cell
    // TODO: how to avoid this 2 loops
    // takes time and memory consuption
    for(size_t i = 0 ; i<IMAGE_DIM; i++)
    {
        for(size_t j = 0 ; j<IMAGE_DIM; j++)
        {
            if ((overrides[i * IMAGE_DIM + j] & (OF_WATER_BASIN | OF_RIVER_MARSH) )   ||
                 elevation[i * IMAGE_DIM + j] == 0 )
                grid[i * IMAGE_DIM + j] = INF;
            else
                // here we penalize that depends of the
                // elevation
               grid[i * IMAGE_DIM + j] =elevation[i * IMAGE_DIM + j]*(TIME_PENALIZE/255.) +1.;
     
        }

    }

    // path planner based in A*
    // here we start A*
    PP::Astar *AStar = new PP::Astar(2048,2048);
    AStar->setDiagonalMovement(true);
    AStar->setMap(grid);

    // our index for the path planner
    // just variables to have batter control of
    // the inputs
    idx_rover = ROVER_Y * IMAGE_DIM + ROVER_X;
    idx_bachelor = BACHELOR_Y * IMAGE_DIM + BACHELOR_X;
    idx_wed =   WEDDING_Y * IMAGE_DIM +   WEDDING_X;


     //compute paths 
    //TODO: add threads to accelerate path computation
    // another idea reduce grid dimentions and adjust weights
     if(AStar->findPath(idx_rover, idx_bachelor ,path, cost1))
         UpdateResults(&path[0], idx_rover, idx_bachelor, results);
   
     if(AStar->findPath(idx_bachelor, idx_wed,path, cost2))
         UpdateResults(&path[0], idx_bachelor, idx_wed, results);
   

    // we dont need it more free memory
    delete AStar;
    delete path;
    delete grid;
    // create image
    visualizer::writeBMP(
        of,
        &elevation[0],
        IMAGE_DIM,
        IMAGE_DIM,
        [&] (size_t x, size_t y, uint8_t elevation) {
            // Marks interesting positions on the map
            if (donut(x, y, ROVER_X, ROVER_Y) ||
                donut(x, y, BACHELOR_X, BACHELOR_Y) ||
                donut(x, y, WEDDING_X, WEDDING_Y) ||
                results[y * IMAGE_DIM + x] == 255)
            {
                // here we visualize data for the path

                return uint8_t(visualizer::IPV_PATH);
            }
            
            // Signifies water
            if ((overrides[y * IMAGE_DIM + x] & (OF_WATER_BASIN | OF_RIVER_MARSH)) ||
                elevation == 0)
            {
            
                return uint8_t(visualizer::IPV_WATER);
            }
            
            // Signifies normal ground color
            if (elevation < visualizer::IPV_ELEVATION_BEGIN)
            {
               elevation = visualizer::IPV_ELEVATION_BEGIN;
            }
            return elevation;
    });

    of.flush();

    // finally cost functions show the time
    std::cout<<"total time of the path in seconds: "<<cost1+cost2<<std::endl;



#if __APPLE__
    auto res = system("open pic.bmp");
    (void)res;
#endif

    return 0;
}


/* road_generator.h */

#ifndef ROAD_GENERATOR_H
#define ROAD_GENERATOR_H

#define MAP_SIZE 700

#include <map>
#include "core/reference.h"
#include "core/math/vector2.h"
#include "core/dictionary.h"
#include "core/array.h"
#include "modules/opensimplex/open_simplex_noise.h"
#include <vector>
#include <cstdlib>
#include <cmath>
#include <ctime>
#include <algorithm>
#include <vector>
#include <iostream>


class Dictionary;
struct Vector2;
class OpenSimplexNoise;

class road_generator : public Reference {
    GDCLASS(road_generator, Reference);
    public:
      struct road_tile {
          double density = -1.0;
          double height = -1.0;
          bool is_accessible = true;
          bool is_road = false;
      };
    private:
        int min_seg_length = 25;
        int max_seg_length = 40;
        int max_road_size = 4;
        int DEFAULT_SUB_SQUARE_SIZE = 50;
        double DEFAULT_MAX_DISTANCE = 30;
        Vector2 map_size = Vector2();
        road_tile road_map[MAP_SIZE][MAP_SIZE];
    protected:
      static void _bind_methods();

    public:
        enum prop_enum{
            S,
            P,
            M,
            O,
            C,
            s
        };
        struct road_node {
            Vector2 p = Vector2(-1,-1);
            int road_size = -1;
        };
        struct road_edge {
            Vector2 start = Vector2(-1,-1);
            Vector2 end = Vector2(-1,-1);
            int size = -1;
        };
        struct L_element{
            prop_enum element = S;
            L_element() : element(S){
            }
            L_element(prop_enum new_elem) : element(new_elem){
            }
        };

        Vector2 action(L_element &le, double &angle,double delta_angle, int length,Vector2 previous_point);
        struct proposition {
            L_element left_side = S;
            std::vector<L_element> right_side;
            double p = -1.0;
        };
        struct L_system {
            Vector2 start_pos = Vector2(-1,-1);
            double base_angle = -1.0;
            int min_length = -1;
            int max_length = -1;
            int nb_iteration = -1;
            double delta_angle = 90;
            int base_road_size = 4;
            std::vector<proposition> props;
        };

        L_system generate_l_system( Vector2 start_pos, double base_angle, int min_length, int max_length, int nb_iteration, int base_road_size );

        struct road_network {
            std::vector<road_node> nodes;
            std::vector<road_edge> edges;
            int distance_to_close_node = 10;
        };
        void mark_road_network(road_network &network, road_tile road_map[MAP_SIZE][MAP_SIZE]);
        road_node is_there_close_node(road_network &network, road_node pos, int dist_max, int road_size);
        Vector2 find_closest_node(road_network &network, Vector2 pos, int road_size, road_tile road_map[MAP_SIZE][MAP_SIZE] );
        Vector2 add_edge_to_network(road_network &network, Vector2 start,Vector2 end,int size);

        void generate_network(L_system &ls, road_tile road_map[MAP_SIZE][MAP_SIZE], road_network &network);

        std::vector<road_generator::L_element> generate_chain(L_system &ls);
        void mark_chain_as_road(L_system &ls, std::vector<L_element> chain,Vector2 start_position,double angle,road_tile road_map[MAP_SIZE][MAP_SIZE],int road_size,road_network &network);
        proposition get_props_from_rhs(L_system &ls, L_element rhs);
        std::vector<L_element> get_sub_chain(std::vector<L_element> &chain,int start);


        void generate_partial_network(std::vector<Vector2> positions, road_network &network, int road_size, const double max_distance);
        void main_algorithm(int square, road_network &network, const double max_distance );
        void generate_new_network(int _min_seg_length = 25, int _max_seg_length = 40, int _max_road_size = 4, Vector2 _map_size = Vector2(700,700));
        static bool comp_function(std::pair<Vector2, double> a, std::pair<Vector2, double> b);
        road_generator();

        int get_min_seg_length(){
            return min_seg_length;
        }

        int get_max_seg_length(){
            return max_seg_length;
        }
        int get_max_road_size(){
            return max_road_size;
        }
        Vector2 get_map_size(){
            return map_size;
        }
        Dictionary get_road_map(){
            Dictionary ret;
            for (unsigned int i = 0; i < MAP_SIZE; ++i) {
                for(unsigned int j = 0; j < MAP_SIZE; ++j ){
                    Dictionary inner;
                    inner["is_road"] = road_map[i][j].is_road;
                    inner["is_accessible"] = road_map[i][j].is_accessible;
                    inner["height"] = road_map[i][j].height;
                    inner["density"] = road_map[i][j].density;
                    ret[Vector2(i, j)] = inner;
                }
            }
            return ret;
        }
        float convert_range(float input, float max_input = 1.0,float min_input = -1.0, float max_output = 1.0 ,float min_output = 0.0);
        void Interpolate( Vector2 a, Vector2 b, std::vector<Vector2>& result );
        Vector2 find_next_point(Vector2 start,double angle, int length);
        bool is_into_the_map(Vector2 p);
        void mark_road(road_tile road_map[MAP_SIZE][MAP_SIZE],Vector2 start,Vector2 end,int size);
        void mark_base_square(road_tile road_map[MAP_SIZE][MAP_SIZE], Vector2 start, Vector2 end,int size);
        double find_angle(Vector2 start, Vector2 end);
        double distance(Vector2 start,Vector2 end);
        double distance_and_height(Vector2 start,Vector2 end,road_tile road_map[MAP_SIZE][MAP_SIZE]);
        std::pair<Vector2, double> get_average_pop(road_tile road_map[MAP_SIZE][MAP_SIZE], Vector2 left_top, int width, int height);
        std::vector<std::pair<Vector2, double> > get_subdivided_map(road_tile road_map[MAP_SIZE][MAP_SIZE], int square_size);
};


#endif // road_generator_h

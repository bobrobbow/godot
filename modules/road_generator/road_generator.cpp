/* road_generator.cpp */

#include <unistd.h>
#include "road_generator.h"
#include <time.h>

void road_generator::generate_new_network(int _min_seg_length, int _max_seg_length, int _max_road_size, Vector2 _map_size )
{
    min_seg_length = _min_seg_length;
    max_seg_length = _max_seg_length;
    max_road_size = _max_road_size;
    map_size = _map_size;
    Vector2 start_position = Vector2(0, map_size.y/2);
    road_node n;
    OpenSimplexNoise height_noise;
    srand(time(NULL));
    int seed1 = rand()%9999;
    int seed2 = rand()%9999;
    height_noise.set_seed(seed1);
    height_noise.set_octaves(3);
    height_noise.set_period(100.0);
    height_noise.set_persistence(2.0);
    height_noise.set_lacunarity(0.6);
    OpenSimplexNoise density_noise;
    density_noise.set_seed(seed2);
    density_noise.set_octaves(3);
    density_noise.set_period(100.0);
    density_noise.set_persistence(2.0);
    density_noise.set_lacunarity(0.6);
    for (unsigned int i = 0; i < MAP_SIZE; ++i) {
        for(unsigned int j = 0; j < MAP_SIZE; ++j ){
            road_map[i][j].height = convert_range(height_noise.get_noise_2d(i,j));
            road_map[i][j].density = convert_range(density_noise.get_noise_2d(i,j));
        }
    }
    n.p = start_position;
    n.road_size = 3;
    road_network network;
    network.nodes.push_back(n);
    network.distance_to_close_node = (min_seg_length + max_seg_length) / 4;
    main_algorithm(DEFAULT_SUB_SQUARE_SIZE, network, DEFAULT_MAX_DISTANCE);
    mark_road_network(network, road_map);
}

float road_generator::convert_range(float input, float max_input,float min_input, float max_output,float min_output) {
  return (input - min_input) / (max_input - min_input) * (max_output - min_output) + min_output;
}

void road_generator::_bind_methods() {
    ClassDB::bind_method(D_METHOD("generate_new_network"), &road_generator::generate_new_network);
    ClassDB::bind_method(D_METHOD("get_road_map"), &road_generator::get_road_map);
}

void road_generator::generate_partial_network(std::vector<Vector2> positions, road_network &network, int road_size, const double max_distance){
    for (unsigned int i = 0; i < positions.size(); ++i) {
        Vector2 end = positions[i];
        Vector2 start = find_closest_node(network, end, road_size,road_map);

        while(distance(start, end) > max_distance){
            int length = distance(start, end);
            double angle = find_angle(start, end);

            int nb_iter =  ceil(log((double)length / (max_seg_length + min_seg_length)));

            if(nb_iter > 5){
                nb_iter = 5;
            }else if(nb_iter <= 0){
                nb_iter = 1;
            }

            nb_iter = ceil((double)nb_iter*1.5);
            L_system sys2 = generate_l_system(start,angle,min_seg_length,max_seg_length,nb_iter,road_size);
            generate_network(sys2, road_map, network);

            start = find_closest_node(network, end, road_size,road_map);
        }
    }
}

void road_generator::main_algorithm(int sub_square_size, road_network &network, const double max_distance){
    std::vector<std::pair<Vector2, double>> s_map = get_subdivided_map(road_map, sub_square_size);
    double max_pop = s_map[0].second;

    //Generate a road network of size 3 for area where the population is greater than 60% of max_pop
    unsigned int i = 0;
    for(int size = 3; size > 0; --size){
        double percent = 0.6;
        std::vector<Vector2> positions;
        while(i < s_map.size() && s_map[i].second > percent*max_pop){
            positions.push_back(s_map[i].first);
            i++;
        }
        generate_partial_network(positions, network, size, max_distance);
        positions.clear();
        percent -= 0.2;
    }

}

road_generator::road_generator(){

    min_seg_length = 25;
}


void road_generator::mark_road_network(road_network &network, road_tile road_map[MAP_SIZE][MAP_SIZE] ){

    for (unsigned int i = 0; i < network.edges.size(); ++i) {
        road_edge e = network.edges[i];
        mark_road(road_map,e.start,e.end,e.size);
    }

}

road_generator::road_node road_generator::is_there_close_node(road_network &network, road_generator::road_node pos, int dist_max,int road_size){
    int index = -1;
    int min_length = dist_max;

    for (unsigned int i = 0; i < network.nodes.size(); ++i) {
        if(distance(pos.p, network.nodes[i].p) < min_length){
            index = i;
            min_length = distance(pos.p, network.nodes[i].p);
        }
    }

    if(index != -1){
        if(road_size > network.nodes[index].road_size){
            network.nodes[index].road_size = road_size;
        }

        return network.nodes[index];
    }

    return pos;
}

Vector2 road_generator::find_closest_node(road_network &network, Vector2 pos, int road_size,road_tile road_map[MAP_SIZE][MAP_SIZE]){
    double min_dist = map_size.x;
    Vector2 result;

    for (unsigned int i = 0; i < network.nodes.size(); ++i) {
        if(distance_and_height(pos, network.nodes[i].p,road_map) < min_dist && network.nodes[i].road_size >= road_size){
            min_dist = distance_and_height(pos, network.nodes[i].p,road_map);
            result = network.nodes[i].p;
        }
    }

    return result;
}

Vector2 road_generator::add_edge_to_network(road_network &network, Vector2 start, Vector2 end, int size){
    road_edge e;
    e.start = start;
    e.size = size;

    road_node en;
    en.p = end;
    en.road_size = size;

    road_node corrected_end = is_there_close_node(network, en, network.distance_to_close_node+size,size);

    if(end == corrected_end.p){
        e.end = end;

        road_node n;
        n.p = end;
        n.road_size = size;

        network.nodes.push_back(n);

    }else{
        e.end = corrected_end.p;
    }

    network.edges.push_back(e);
    return corrected_end.p;
}

road_generator::L_system road_generator::generate_l_system(Vector2 start_pos, double base_angle, int min_length, int max_length, int nb_iteration, int base_road_size){
    L_system sys2;
    sys2.start_pos = start_pos;
    sys2.base_angle = base_angle;
    sys2.min_length = min_length;
    sys2.max_length = max_length;
    sys2.nb_iteration = nb_iteration;
    sys2.base_road_size = base_road_size;
    std::vector<L_element> v;

    proposition p1;
    p1.left_side = L_element();
    v = {L_element(), L_element()};
    p1.right_side = v;
    p1.p = 0.25;
    sys2.props.push_back(p1);


    proposition p2;
    p2.left_side = L_element();
    v = {L_element(),L_element(O),L_element(P),L_element(s),L_element(C),L_element(S)};
    p2.right_side = v;
    p2.p = 0.25;
    sys2.props.push_back(p2);


    proposition p3;
    p3.left_side = L_element();
    v = {L_element(),L_element(O),L_element(M),L_element(s),L_element(C),L_element()};
    p3.right_side = v;
    p3.p = 0.25;
    sys2.props.push_back(p3);

    proposition p32;
    p32.left_side = L_element();
    v = {L_element(),L_element(O),L_element(P),L_element(s),L_element(C),L_element(O),L_element(M),L_element(s),L_element(C),L_element()};
    p32.right_side = v;
    p32.p = 0.25;
    sys2.props.push_back(p32);


    proposition p4;
    p4.left_side = L_element(O);
    v = {L_element(O)};
    p4.right_side = v;
    p4.p = 1;
    sys2.props.push_back(p4);


    proposition p5;
    p5.left_side = L_element(C);
    v = {L_element(C)};
    p5.right_side = v;
    p5.p = 1;
    sys2.props.push_back(p5);


    proposition p6;
    p6.left_side = L_element(P);
    v = {L_element(P)};
    p6.right_side = v;
    p6.p = 1;
    sys2.props.push_back(p6);


    proposition p7;
    p7.left_side = L_element(M);
    v = {L_element(M)};
    p7.right_side = v;
    p7.p = 1;
    sys2.props.push_back(p7);


    proposition p8;
    p8.left_side = L_element(s);
    v = {L_element(s),L_element(S)};
    p8.right_side = v;
    p8.p = 1;
    sys2.props.push_back(p8);


    srand(time(NULL));
    return sys2;
}

void road_generator::generate_network(L_system &sys2, road_tile road_map[MAP_SIZE][MAP_SIZE], road_network &network)
{
    std::vector<L_element> chain = generate_chain(sys2);

    mark_chain_as_road(sys2, chain,sys2.start_pos,sys2.base_angle,road_map,sys2.base_road_size, network);
}

std::vector<road_generator::L_element> road_generator::generate_chain(L_system &sys2 )
{
    std::vector<L_element> result;
    L_element new_elem = L_element();
    result.push_back(new_elem);

    std::vector<L_element> temp_result;

    for (int i = 0; i < sys2.nb_iteration; ++i) {
        for (unsigned int j = 0; j < result.size(); ++j) {
            std::vector<L_element> elements = get_props_from_rhs(sys2, result[j]).right_side;
            temp_result.insert(temp_result.end(),elements.begin(),elements.end());
        }

        result = temp_result;
        temp_result.clear();
    }

    return result;
}

void road_generator::mark_chain_as_road(L_system &sys2, std::vector<L_element> chain, Vector2 start_position, double angle, road_tile road_map[MAP_SIZE][MAP_SIZE], int road_size, road_network &network)
{
    Vector2 current_pos = start_position;
    double current_angle = angle;
    for (unsigned int i = 0; i < chain.size(); ++i) {
        switch (chain[i].element) {
        case O:
            if(road_size > 0)
                mark_chain_as_road(sys2, get_sub_chain(chain,i),current_pos,current_angle,road_map,road_size-1,network);
            else
                mark_chain_as_road(sys2, get_sub_chain(chain,i),current_pos,current_angle,road_map,road_size,network);
            break;
        default:
            double length = rand() % (sys2.max_length-sys2.min_length) + sys2.min_length;
            Vector2 next_pos = action(chain[i], current_angle,sys2.delta_angle,length,current_pos);

            current_pos = add_edge_to_network(network, current_pos,next_pos,road_size);

            //Correcting the angle when a point is replaced by an existing point close by.
            L_element new_elem = L_element();
            Vector2 old_next_pos = action(new_elem, current_angle,sys2.delta_angle,100,next_pos);
            angle = find_angle(current_pos,old_next_pos);
            break;
        }
    }
}

road_generator::proposition road_generator::get_props_from_rhs(L_system &sys2, L_element rhs)
{
    std::vector<proposition> result;
    for (unsigned int i = 0; i < sys2.props.size(); ++i) {
        if(rhs.element == sys2.props[i].left_side.element){
            result.push_back(sys2.props[i]);
        }
    }

    double r = (rand() % 1000 + 1) / 1000.0;
    double t = 0;

    for (unsigned int i = 0; i < result.size(); ++i) {
        t += result[i].p;

        if(r <= t){
            return result[i];
        }
    }

    return result[0];
}

std::vector<road_generator::L_element> road_generator::get_sub_chain(std::vector<L_element> &chain, int start)
{
    int nb_bracket = 0;
    int end = start;
    for (unsigned int i = start; i < chain.size(); ++i) {
        if(chain[i].element == O){
            nb_bracket += 1;
        }else if(chain[i].element == C){
            nb_bracket -=1;
            if(nb_bracket == 0){
                end = i;
                break;
            }
        }
    }

    std::vector<L_element> result(chain.begin()+start,chain.begin()+end);
    chain.erase(chain.begin()+start,chain.begin()+end);

    return result;
}



Vector2 road_generator::action(L_element &le, double &angle,double delta_angle, int length, Vector2 previous_point){
    switch (le.element) {
    case S:
        return find_next_point(previous_point,angle,length);
        break;
    case s:
        return find_next_point(previous_point,angle,length);
        break;
    case P:
        angle += delta_angle;
        break;
    case M:
        angle -= delta_angle;
        break;
    default:
        break;
    }

    return previous_point;
}

// utility functions

void road_generator::Interpolate( Vector2 a, Vector2 b, std::vector<Vector2>& result ){
    int N = result.size();
    Vector2 step = Vector2(b-a) / float(std::max(N-1,1));
    Vector2 current( a );
    for( int i=0; i<N; ++i )
    {
        result[i] = current;
        current += step;
    }
}

Vector2 road_generator::find_next_point(Vector2 start,double angle, int length){
    double angle_rad = angle*M_PI/180;
    Vector2 result(cos(angle_rad)*length,sin(angle_rad)*length);
    result = result+start;

    return result;
}

bool road_generator::is_into_the_map(Vector2 p){
    return (p.x >= 0 && p.x < map_size.x && p.y >= 0 && p.y < map_size.y);
}

void road_generator::mark_road(road_tile road_map[MAP_SIZE][MAP_SIZE], Vector2 start,Vector2 end,int size){
    std::vector<Vector2> start_positions(4 * size + 1);
    std::vector<Vector2> end_positions(4 * size + 1);


    if(!is_into_the_map(start)){
        int absx = std::abs(start.x - end.x);
        int absy = std::abs(start.y - end.y);
        Vector2 delta = Vector2(absx, absy);
        int pixels = std::max( delta.x, delta.y ) + 1;

        std::vector<Vector2> line( pixels );
        Interpolate( start, end, line );

        for (unsigned int i = 0; i < line.size(); ++i) {
            if(is_into_the_map(line[i])){
                start = line[i];
                break;
            }
        }
    }else if(!is_into_the_map(end)){
        int absx = std::abs(start.x - end.x);
        int absy = std::abs(start.y - end.y);
        Vector2 delta = Vector2(absx, absy);
        int pixels = std::max( delta.x, delta.y ) + 1;

        std::vector<Vector2> line( pixels );
        Interpolate( start, end, line );

        for (int i = line.size()-1; i <= 0; --i) {
            if(is_into_the_map(line[i])){
                end = line[i];
                break;
            }
        }
    }

    if(!is_into_the_map(start) || !is_into_the_map(end)){
        return;
    }

    start_positions[0] = start;
    end_positions[0] = end;

    for (int s = 1; s <= size; ++s) {
        start_positions[s*4] = Vector2(start) + s*Vector2(0,1);
        start_positions[s*4-1] = Vector2(start) + s*Vector2(1,0);
        start_positions[s*4-2] = Vector2(start) + s*Vector2(0,-1);
        start_positions[s*4-3] = Vector2(start) + s*Vector2(-1,0);

        end_positions[s*4] = Vector2(end) + s*Vector2(0,1);
        end_positions[s*4-1] = Vector2(end) + s*Vector2(1,0);
        end_positions[s*4-2] = Vector2(end) + s*Vector2(0,-1);
        end_positions[s*4-3] = Vector2(end) + s*Vector2(-1,0);
    }

    mark_base_square(road_map,start,end,size);
    for (unsigned int i = 0; i < start_positions.size(); ++i) {
        //We check if all the point are in the map if not we replace them with the base point
        if(!is_into_the_map(start_positions[i])){
            start_positions[i] = start_positions[0];
        }if(!is_into_the_map(end_positions[i])){
            end_positions[i] = end_positions[0];
        }
        int absx = std::abs( start_positions[i].x - end_positions[i].x );
        int absy = std::abs( start_positions[i].y - end_positions[i].y );
        Vector2 delta = Vector2(absx, absy);
        int pixels = std::max( delta.x, delta.y ) + 1;

        std::vector<Vector2> line( pixels );
        Interpolate( start_positions[i], end_positions[i], line );

        for (unsigned int j = 0; j < line.size(); ++j) {
            Vector2 t = Vector2(int(line[j].x), int(line[j].y));
            road_map[int(t.x)][int(t.y)].is_road = true;
        }
    }
}


void road_generator::mark_base_square(road_tile road_map[MAP_SIZE][MAP_SIZE], Vector2 start, Vector2 end, int size)
{
    Vector2 a = Vector2(start) + size*Vector2(-1,-1);
    Vector2 b = Vector2(end) + size*Vector2(-1,-1);

    for (int j = 0; j < 2*size+1; ++j) {
        for (int i = 0; i < 2*size+1; ++i) {
            if(is_into_the_map(Vector2(a.x+i,a.y+j))){
                Vector2 point_there = Vector2(a.x+i,a.y+j);
                road_map[int(point_there.x)][int(point_there.y)].is_road = true;
            }
            if(is_into_the_map(Vector2(b.x+i,b.y+j))){
                Vector2 point_there = Vector2(b.x+i,b.y+j);
                road_map[int(point_there.x)][int(point_there.y)].is_road = true;
            }
        }
    }
}

double road_generator::find_angle(Vector2 start, Vector2 end){

    double alpha;
    double angle_degre = 90;
    end = end - start;
    start = start - start;

    if(end.x >= start.x && end.y < start.y){
        alpha = atan((double)end.y/end.x);
        angle_degre = alpha*180/M_PI;
    }else if(end.x > start.x && end.y >= start.y){
        alpha = atan((double)end.y/end.x);
        angle_degre = alpha*180/M_PI;
    }else if(end.x <= start.x && end.y > start.y){
        alpha = atan((double)-end.x/end.y);
        angle_degre = 90 + alpha*180/M_PI;
    }else if(end.x < start.x && end.y <= start.y){
        if(end.y != 0){
            alpha = atan(-(double)end.x/end.y);
            angle_degre = -90 + alpha*180/M_PI;
        }else{
            angle_degre = 180;
        }
    }

    return angle_degre;
}

double road_generator::distance(Vector2 start,Vector2 end){
    return sqrt(pow((end.x - start.x),2) + pow((end.y - start.y),2));
}

double road_generator::distance_and_height(Vector2 start,Vector2 end,road_tile road_map[MAP_SIZE][MAP_SIZE]){
    if(!is_into_the_map(start) || !is_into_the_map(end)){
        return distance(start,end);
    }

    double result = sqrt(pow((end.x - start.x),2) + pow((end.y - start.y),2));
    int absx = std::abs( start.x - end.x );
    int absy = std::abs( start.y - end.y );
    Vector2 delta = Vector2(absx, absy);
    int pixels = std::max( delta.x, delta.y ) + 1;

    std::vector<Vector2> line( pixels );
    Interpolate( start, end, line );
    double previous = road_map[int(line[0].x)][int(line[0].y)].height;

    for (unsigned int i = 0; i < line.size(); ++i) {
        Vector2 p = line[i];
        result += 10*abs(road_map[int(p.x)][int(p.y)].height - previous);
        previous = road_map[int(p.x)][int(p.y)].height;
    }


    return result;
}

std::pair<Vector2, double> road_generator::get_average_pop(road_tile road_map[MAP_SIZE][MAP_SIZE], Vector2 left_top, int width, int height)
{
    double w = width / 2;
    double h = height / 2;

    Vector2 center = left_top + Vector2(w,h);

    double total = 0;
    double nb_tile = 0;

    for (int x = left_top.x; x < left_top.x+width; ++x) {
        for (int y = left_top.y; y < left_top.y+height; ++y) {
            Vector2 vec_there = Vector2(int(x),int(y));
            total += road_map[int(vec_there.x)][int(vec_there.y)].density;
            nb_tile++;
        }
    }

    return std::pair<Vector2, double>(center,total/nb_tile);
}

std::vector<std::pair<Vector2, double> > road_generator::get_subdivided_map(road_tile road_map[MAP_SIZE][MAP_SIZE], int square_size)
{
    std::vector<std::pair<Vector2, double> > result;

    for (int x = 0; x < map_size.x; x += square_size) {
        for (int y = 0; y < map_size.y; y += square_size) {
            // If we are at the end of the map..
            if((x+square_size) < map_size.x && (y+square_size) < map_size.y){
                Vector2 res = Vector2(x,y);
                result.push_back(get_average_pop(road_map,res,square_size, square_size));
            }else{
                int width = square_size;
                int heigth = square_size;

                if((x+square_size) >= map_size.x){
                    width = map_size.x - x - 1;
                }

                if((y+square_size) >= map_size.y){
                    heigth = map_size.y - y - 1;
                }
                Vector2 res2 = Vector2(x,y);
                result.push_back(get_average_pop(road_map,res2, width, heigth));

            }
        }
    }

    sort(result.begin(),result.end(),comp_function);

    return result;
}


bool road_generator::comp_function(std::pair<Vector2, double> a, std::pair<Vector2, double> b)
{
    return a.second > b.second;
}

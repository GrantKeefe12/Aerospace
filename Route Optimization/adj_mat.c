#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "adj_mat.h"

#define PI 3.141592654
#define drone_speed  20.0
#define drone_max_w  7.5
#define drone_max_p  6.0

//Coordinates of waypoints
double coordinates[27][3] = {
    {97,	 48.51667069999998,  -71.6375025},
    {98,	 48.5060947       ,  -71.63175180000002},
    {99,	 48.49211590000004,  -71.63400689999999},
    {100,	 48.51503409999999,  -71.6404442},
    {101,	 48.50053369999999,   -71.6782955},
    {102,	 48.508839500000015,  -71.6040591},
    {103,	 48.5101473,          -71.65221010000002},
    {104,    48.512991700000015,  -71.64260060000001},
    {105,	 48.5117408,          -71.6428152},
    {106,	 48.51933110000004,   -71.6229056},
    {107,	 48.49846230000001,   -71.65680879999998},
    {108,	 48.5019885,          -71.62530889999998},
    {109,	 48.520525000000006,  -71.67200080000002},
    {110,	 48.5090567,          -71.6461702},
    {111,	 48.51070570000003,   -71.6516848},
    {112,	 48.50396670000003,   -71.62981979999999},
    {113,	 48.526230800000015,  -71.6345802},
    {114,	 48.49842660000004,   -71.6425625},
    {115,	 48.5258329,          -71.6320911},
    {116,	 48.49967789999998,   -71.6758648},
    {117,	 48.49370579999999,   -71.6290012},
    {118,	 48.51035300000003,   -71.6228085},
    {119,	 48.5093153,          -71.62160689999999},
    {120,	 48.496924800000016,  -71.60340180000001},
    {121,	 48.51125569999999,   -71.6312968},
    {122,	 48.49328459999999,   -71.6664874},
    {18,     48.51156300000004,   -71.68049960000002}
};

//linked list for task information storage
typedef struct routeinfo{
    int numroute;
    int start;
    int finish;
    int numpeople;
    int weight;
    int profit;
    route *next;
}route;
route *head = NULL;

//structure of edge allows for information to be held
typedef struct edgeifo{
    float value;
    float time;
}edge;

//graph structure (adjacency matrix)
typedef struct mygraph {
    int numnodes;
    edge ***edges;
}graph;

//reads data from txt file and creates a graph from the given information
graph *generate_graph(){

    int r, pe, w, pr;
    float distance, value;
    char *p,*numroute,*numpeople,*start,*finish,*weight,*obstacles,*profit, *pbuff;
    FILE *fp;

    // Get buffer to read data file into.
    pbuff = (char *) malloc(256);
    if (pbuff == NULL) {
        printf("ERROR: Cannot get memory for buffer\n");
    }
    fp = fopen("route_info.txt", "r");
    if (fp == NULL) {
        printf("ERROR: Cannot open data file\n");
    }

    while (1) {
        p = fgets(pbuff, 256, fp);
        if (p == NULL) break;
        //retreive data
        p++;
        while (*p != ' ') p++;
        p++;
        while (*p != ' ') p++;
        numroute = p;
        while (*p != ':') p++;
        *p = '\0';
        p++;
        p++;
        numpeople = p;
        while (*p != ' ') p++;
        *p = '\0';
        p++;
        while (*p == ' ') p++;
        while (*p != ' ') p++;
        start = p;
        while (*p != ';') p++;
        *p = '\0';
        p++;
        finish = p;
        while (*p != ';') p++;
        *p = '\0';
        p++;
        weight = p;
        while (*p != 'k') p++;
        *p = '\0';
        p++;
        while (*p != ' ') p++;
        while (*p == ' ') p++;
        obstacles = p;
        while (*p != ';') p++;
        *p = '\0';
        p++;
        while (*p != '$') p++;
        p++;
        profit = p;
        while (*p != '\n' && *p != '\r') p++;
        *p = '\0';

        //cast strings to needed format
        r = atoi(numroute);
        pe = atoi(numpeople);
        w = atoi(weight);
        pr = atoi(profit);

        //store information in linked list
        add_route(r,pe, w, pr, start, finish);
    }
    //create a blank graph
    graph *g = create_graph(r*2);
    //use information stored in the linked list to generate the edges
    automate_edges(g);
    print_graph(g);
    fclose(fp);
    return g;
}

//finds the profit of each trip in dollars per hour
float find_value(float distance, float weight, float people, float profit){

    float tot_time;

    tot_time = find_time(distance, weight, people);

    return (profit/tot_time);
}

//uses the distance and drone speed to get a time estimate for a trip
float find_time(float distance, float weight, float people){
    int numtrips_people;
    int numtrips_weight;
    int numtrips;
    float trip_time, tot_time;

    numtrips_people = ceil(people/drone_max_p);
    numtrips_weight = ceil(weight/drone_max_w);

    if (numtrips_people>numtrips_weight){
        numtrips = numtrips_people;
    }else {
        numtrips = numtrips_weight;
    }

    trip_time = (distance/drone_speed);

    tot_time = (trip_time*numtrips);

    return tot_time;

}

//finds the distance between two locations
float find_distance(int s, int f){

    double startlong;
    double startlat;

    double finishlong;
    double finishlat;

    for (int i = 0; i <= 27; i++){
        if (s == coordinates[i][0]){
            startlat = coordinates[i][1];
            startlong = coordinates[i][2];
        }
    }
    for (int i = 0; i <= 27; i++){
        if (f == coordinates[i][0]){
            finishlat = coordinates[i][1];
            finishlong = coordinates[i][2];
        }
    }

    return havershine(startlong, startlat, finishlong, finishlat);
}

//helper function for find_distance
float havershine(double startlong, double startlat, double finishlong, double finishlat){

    double R, phi_1, phi_2, delta_phi, delta_lamda, a1, a2, b, c;
    float meters, km;

    R = 6371000.0;

    phi_1 = startlat*(PI / 180.0);
    phi_2 = finishlat*(PI / 180.0);

    delta_phi = (finishlat - startlat)*(PI / 180.0);
    delta_lamda = (finishlong - startlong)*(PI /180.0);

    a1 = ((sin(delta_phi/2.0))*(sin(delta_phi/2.0)));
    b = (cos(phi_1))*(cos(phi_2))*(sin(delta_lamda/2.0)*sin(delta_lamda/2.0));
    a2 = a1 + b;

    c = 2.0* atan2(sqrt(a2),sqrt(1-a2));

    meters = R*c;

    km = meters/1000.0;

    return km;
}

//linked list data storage
void add_route(int numroute, int numpeople, float w, float pr, char* start, char* finish){

    route *r = (route*)malloc(sizeof(route));

    int s = start[1];
    int f = finish[1];

    r-> numroute = numroute;
    r-> numpeople = numpeople;
    r-> start = s;
    r-> finish = f;
    r-> weight = w;
    r-> profit = pr;
    r-> next = NULL;

    if (head == NULL){
        head = r;
    }else{
        route* temp = head;
        while (temp->next != NULL){
            temp = temp->next;
        }
        temp -> next = r;
    }
}

//prints linked list for error checking
void print_route_info(void){

    printf("all route info\n");
    route* temp;
    temp = head;

    if (temp == NULL){
        printf("list is empty\n");
    }
    while (temp != NULL){
            printf("r = %d people = %d weight = %d profit =%d start = %d finish = %d\n" 
            ,temp->numroute,temp->numpeople,temp->weight,temp->profit, temp -> start, temp -> finish);
            temp = temp->next;
    }
}

//allocates the needed memory to the graph
graph *create_graph(int numnodes){

    graph *g = malloc(sizeof(*g));

    if (g==NULL){
        return NULL;
    }
    g->numnodes = numnodes;

    //allocate the matrix
    g->edges = calloc(sizeof(edge*),g->numnodes);
    if (g->edges==NULL){
        free(g);
        return NULL;
    }
    for (int i = 0; i < g->numnodes; i++){
        //calloc initializes as 0
        g->edges[i] = calloc(sizeof(edge), g-> numnodes);
        if (g->edges[i]==NULL){
            destroy_graph(g);
            return NULL;
        }
    }
    return g;
}

//frees the graph memory when finished
void destroy_graph(graph *g){

    if (g-> edges == NULL) return;
    for (int i = 0; i < g->numnodes; i++){
        if (g-> edges != NULL){
            free(g->edges[i]);
        }
    }
    free(g->edges);
    free(g);
}

//prints the vertices and edges of the graph
void print_graph(graph *g){

    printf("digraph {\n");

    for (int from = 0; from < g->numnodes; from++){
        for (int to = 0; to < g->numnodes; to++){
            if (g->edges[from][to]){
                printf("%d --(value = %0.2f time = %0.2f)-> %d\n", 
                from,
                g->edges[from][to]->value,
                g->edges[from][to]->time,
                to);
            }
        }
    }
    printf("}\n");
}

//follows a pattern from the given task to generate the appropriate edges
void automate_edges(graph *g){
    route* temp;
    route* temp2;
    route* temp3;
    temp = head;

    float value;
    int count = 0;
    float time;
    float distance;

    while (temp != NULL){
        if(temp -> next != NULL){

            //creates main routes
            distance = find_distance(temp->start,temp->finish);
            value = temp->profit;
            time = find_time(distance, temp->weight, temp-> numpeople);
            add_edge(g, value, time, (temp->numroute)+count-1,temp->numroute+count);


            //add connecting routes
            temp2 = temp->next;
            temp3 = temp2->next;

            distance = find_distance(temp->start,temp2->start);
            value = 0;

            time = find_time(distance,1, 1);
            //printf("adding down node %f from %d to %d\n",value, (temp->numroute) + count -1, ((temp2->numroute) + count));
            add_edge(g, value, time, ((temp->numroute) + count -1) ,((temp2->numroute) + count));

            distance = find_distance(temp->finish,temp2->start);
            time = find_time(distance,1, 1);
            add_edge(g, value, time, (temp->numroute) + count, (temp->numroute) * 2);

            while (temp2 != NULL && temp3 != NULL){

                distance = find_distance(temp->start,temp3->start);
                time = find_time(distance, 1, 1);
                add_edge(g, value, time, (temp->numroute + count -1),(temp2->numroute)*2);

                distance = find_distance(temp->finish,temp3->start);
                time = find_time(distance, 1, 1);
                add_edge(g, value, time, temp->numroute + count,(temp2->numroute)*2);
                temp2 = temp2 -> next;
                temp3 = temp3-> next;  
            }
        }
        else{
            distance = find_distance(temp->start,temp->finish);
            value = temp -> profit;
            time = find_time(distance, 1, 1);
            add_edge(g, value, time, (temp->numroute)+count-1,temp->numroute+count);
        }
        temp = temp -> next;
        count ++;
    }
}

//allocates memory for an edge and assigns vlalues
edge *create_edge(float value,float time){
    edge *e = malloc(sizeof(*e));
    if (e == NULL){
        return NULL;
    }
    e -> value = value;
    e -> time = time;

    return e;
}

//helper for automate_edges
bool add_edge(graph *g, float value, float time, unsigned int from_node,unsigned int to_node){

    assert(g != NULL);
    assert(from_node < g-> numnodes);
    assert(to_node < g-> numnodes);

    if (has_edge(g, from_node,to_node)){
        return false;
    }

    edge *e = create_edge(value,time);

    g->edges[from_node][to_node] = e;
  
    return true;
}
//helper for automate_edges
bool has_edge(graph *g, unsigned int from_node,unsigned int to_node){

    assert(g != NULL);
    assert(from_node < g-> numnodes);
    assert(to_node < g-> numnodes);

    return g->edges[from_node][to_node];
}


// BFT implementation //


typedef struct node{
    int from;
    int to;
    float value;
    float time;
    waypoint *next;
    waypoint *previous;
}waypoint;

waypoint *top = NULL;

//queue implementation
waypoint *create_node(int from, int to, float time, float value){
    waypoint *w = (waypoint*)malloc(sizeof(waypoint));
 
    if (w==NULL) return NULL;
    w -> from = from;
    w -> to = to;
    w -> time = time;
    w -> value = value;
    w -> next = NULL;
    w -> previous = NULL;
}

void add_node(waypoint *node){
    if (top == NULL){
        top = node;
    }else{
        waypoint* temp = top;
        while (temp->next != NULL){
            temp = temp->next;
        }
        temp -> next = node;
    }
}
waypoint *pop_node(){
    if (top == NULL) return NULL;
    waypoint* temp = top;
    top = top -> next;
    return temp;
}

//searching algorithm
waypoint *search_graph(graph *g){
    waypoint *best = create_node(0,0,0,0);
    int from = 0;
    for (int to = 0; to < g->numnodes; to++){
        if (g->edges[from][to]){ 
            waypoint *new = create_node(from, to, g->edges[from][to]->time, g->edges[from][to]->value);
            printf("adding initial node to q time:%f\n",new->time);
            add_node(new);
        }
    }
    while (top != NULL){
        waypoint *temp = pop_node();
        printf("popping from q time:%f to %d\n", temp->time, temp-> to);
        int from = temp->to;
        for (int to = 0; to < g->numnodes; to++){
            if (g->edges[from][to]){ 
                waypoint *new = create_node(from, to, g->edges[from][to]->time, g->edges[from][to]->value);
                new -> previous = temp;
                new -> time = g->edges[from][to]->time + temp->time;
                new -> value = g->edges[from][to]->value + temp->value;
                if (new -> time < 1){
                    printf("adding node to q time:%f\n",new->time);
                    add_node(new);
                }
                else{
                    if (new->value > best->value){
                        best = new;
                        printf("NEW BEST %f\n", best->value);
                    }
                    else{
                        free(new);
                    }
                }
            }
        }
    }
    printf("search finished\n");
    return best;
}